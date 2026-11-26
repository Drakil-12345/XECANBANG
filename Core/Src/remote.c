#include "remote.h"
#include "uart_link.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REMOTE_BAUD       115200U
#define LINE_MAX_LEN      48U
#define JOY_TIMEOUT_MS    400U
#define POLL_MAX_BYTES    96U

/* Tunables that live in main.c (same variables you can edit in Live Expressions). */
extern volatile float   g_kp;
extern volatile float   g_kd;
extern volatile float   g_ki;
extern volatile float   g_target_angle_deg;
extern volatile float   g_speed_kp;
extern volatile float   g_speed_ki;
extern volatile float   g_cmd_speed_max_cps;
extern volatile float   g_cmd_turn_max_pwm;
extern volatile uint8_t g_arm_request;
extern volatile uint8_t g_disarm_request;

typedef struct
{
    const char *key;
    volatile float *var;
    float lo;
    float hi;
} Param_t;

/* Safety limits for values coming over wifi. */
static const Param_t k_params[] =
{
    { "kp",  &g_kp,                 0.0f,  3000.0f },
    { "kd",  &g_kd,                 0.0f,   200.0f },
    { "ki",  &g_ki,                 0.0f,    50.0f },
    { "ta",  &g_target_angle_deg,  -15.0f,   15.0f },
    { "skp", &g_speed_kp,           0.0f,     0.1f },
    { "ski", &g_speed_ki,           0.0f,     1.0f },
    { "ms",  &g_cmd_speed_max_cps,  0.0f,  3000.0f },
    { "mt",  &g_cmd_turn_max_pwm,   0.0f,  3000.0f },
};
#define PARAM_COUNT (sizeof(k_params) / sizeof(k_params[0]))

static char     s_line[LINE_MAX_LEN];
static uint8_t  s_line_len = 0U;
static bool     s_line_overflow = false;

static int8_t   s_fwd = 0;
static int8_t   s_turn = 0;
static uint32_t s_last_joy_ms = 0U;
static bool     s_joy_seen = false;
static uint32_t s_now_ms = 0U;

volatile uint32_t g_remote_lines_ok = 0U;
volatile uint32_t g_remote_lines_bad = 0U;

/* ---- float -> text without printf("%f") (newlib-nano has no float printf) ---- */
static void fmt_f(char *out, size_t n, float v, uint8_t decimals)
{
    uint32_t scale = 1U;
    uint32_t ip;
    uint32_t fp;
    bool neg = (v < 0.0f);
    char frac[8];
    int len;

    if (neg) v = -v;
    if (v > 1.0e6f) v = 1.0e6f;
    if (decimals > 6U) decimals = 6U;
    for (uint8_t i = 0U; i < decimals; i++) scale *= 10U;

    ip = (uint32_t)v;
    fp = (uint32_t)((v - (float)ip) * (float)scale + 0.5f);
    if (fp >= scale) { ip++; fp -= scale; }

    if (decimals == 0U)
    {
        (void)snprintf(out, n, "%s%lu", neg ? "-" : "", (unsigned long)ip);
        return;
    }

    len = snprintf(frac, sizeof(frac), "%0*lu", (int)decimals, (unsigned long)fp);
    while ((len > 1) && (frac[len - 1] == '0')) frac[--len] = '\0';  /* trim zeros */
    (void)snprintf(out, n, "%s%lu.%s", neg ? "-" : "", (unsigned long)ip, frac);
}

static void send_text(const char *s)
{
    (void)UartLink_Write((const uint8_t *)s, (uint16_t)strlen(s));
}

static void send_params(void)
{
    char msg[160];
    char v[PARAM_COUNT][16];
    for (size_t i = 0U; i < PARAM_COUNT; i++)
    {
        fmt_f(v[i], sizeof(v[i]), *k_params[i].var, 6U);
    }
    (void)snprintf(msg, sizeof(msg), "P,%s,%s,%s,%s,%s,%s,%s,%s\n",
                   v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
    send_text(msg);
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static bool handle_set(char *args)
{
    char *comma = strchr(args, ',');
    char *end = NULL;
    float value;

    if (comma == NULL) return false;
    *comma = '\0';
    value = strtof(comma + 1, &end);
    if ((end == (comma + 1)) || (*end != '\0')) return false;
    if (!(value == value)) return false;   /* reject NaN */

    for (size_t i = 0U; i < PARAM_COUNT; i++)
    {
        if (strcmp(args, k_params[i].key) == 0)
        {
            *k_params[i].var = clampf(value, k_params[i].lo, k_params[i].hi);
            send_params();
            return true;
        }
    }
    return false;
}

static bool handle_line(char *s)
{
    switch (s[0])
    {
    case 'J':
    {
        char *p = NULL;
        long f;
        long t;
        if (s[1] != ',') return false;
        f = strtol(s + 2, &p, 10);
        if (*p != ',') return false;
        t = strtol(p + 1, &p, 10);
        if (*p != '\0') return false;
        if (f > 100) f = 100;
        if (f < -100) f = -100;
        if (t > 100) t = 100;
        if (t < -100) t = -100;
        s_fwd = (int8_t)f;
        s_turn = (int8_t)t;
        s_last_joy_ms = s_now_ms;
        s_joy_seen = true;
        return true;
    }
    case 'A':
        if (s[1] != '\0') return false;
        g_arm_request = 1U;
        return true;
    case 'D':
        if (s[1] != '\0') return false;
        g_disarm_request = 1U;
        s_fwd = 0;
        s_turn = 0;
        return true;
    case 'G':
        if (s[1] != '\0') return false;
        send_params();
        return true;
    case 'S':
        if (s[1] != ',') return false;
        return handle_set(s + 2);
    default:
        return false;
    }
}

void Remote_Init(void)
{
    s_line_len = 0U;
    s_line_overflow = false;
    s_fwd = 0;
    s_turn = 0;
    s_joy_seen = false;
    UartLink_Init(REMOTE_BAUD);
}

void Remote_Poll(uint32_t now_ms)
{
    int c;

    s_now_ms = now_ms;

    for (uint16_t n = 0U; n < POLL_MAX_BYTES; n++)
    {
        c = UartLink_ReadByte();
        if (c < 0) break;

        if ((c == '\n') || (c == '\r'))
        {
            if ((s_line_len > 0U) && !s_line_overflow)
            {
                s_line[s_line_len] = '\0';
                if (handle_line(s_line)) g_remote_lines_ok++;
                else g_remote_lines_bad++;
            }
            s_line_len = 0U;
            s_line_overflow = false;
        }
        else if (s_line_len < (LINE_MAX_LEN - 1U))
        {
            s_line[s_line_len++] = (char)c;
        }
        else
        {
            s_line_overflow = true;   /* drop the rest of this line */
        }
    }
}

void Remote_GetCommand(Remote_Cmd_t *cmd, uint32_t now_ms)
{
    if (cmd == NULL) return;

    if (s_joy_seen && ((uint32_t)(now_ms - s_last_joy_ms) < JOY_TIMEOUT_MS))
    {
        cmd->fwd = s_fwd;
        cmd->turn = s_turn;
        cmd->link_ok = true;
    }
    else
    {
        cmd->fwd = 0;
        cmd->turn = 0;
        cmd->link_ok = false;
    }
}

void Remote_SendTelemetry(float angle_deg, float speed_cps,
                          uint8_t state, uint8_t fault, float trim_deg)
{
    char msg[64];
    (void)snprintf(msg, sizeof(msg), "T,%d,%d,%u,%u,%d\n",
                   (int)(angle_deg * 100.0f),
                   (int)speed_cps,
                   (unsigned)state,
                   (unsigned)fault,
                   (int)(trim_deg * 100.0f));
    send_text(msg);
}
