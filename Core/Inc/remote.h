#ifndef REMOTE_H
#define REMOTE_H

/*
 * Remote-control protocol with the ESP32 (text lines ending in '\n', 115200 8N1).
 *
 *  ESP32 -> STM32
 *    J,<fwd>,<turn>        joystick, each integer -100..100 (fwd>0 = forward, turn>0 = right)
 *    A                     arm (start balancing)
 *    D                     disarm (motors off)
 *    S,<key>,<value>       set a parameter: kp kd ki ta skp ski ms mt
 *    G                     ask for the current parameters
 *
 *  STM32 -> ESP32
 *    T,<angle*100>,<speed_cps>,<state>,<fault>,<trim*100>   10 Hz telemetry
 *    P,<kp>,<kd>,<ki>,<ta>,<skp>,<ski>,<ms>,<mt>           reply to G / S
 *
 * If no valid J line arrives for 400 ms the command falls back to 0,0
 * (fail-safe: the robot just stands still).
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int8_t fwd;      /* -100..100 */
    int8_t turn;     /* -100..100 */
    bool   link_ok;  /* a fresh J line was received */
} Remote_Cmd_t;

void Remote_Init(void);
void Remote_Poll(uint32_t now_ms);                       /* call every control tick */
void Remote_GetCommand(Remote_Cmd_t *cmd, uint32_t now_ms);
void Remote_SendTelemetry(float angle_deg, float speed_cps,
                          uint8_t state, uint8_t fault, float trim_deg);

#endif /* REMOTE_H */
