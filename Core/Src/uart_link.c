#include "uart_link.h"
#include "main.h"

#define RX_SIZE 256U   /* power of two */
#define TX_SIZE 512U   /* power of two */

static volatile uint8_t  s_rx[RX_SIZE];
static volatile uint16_t s_rx_head = 0U;   /* written by ISR */
static volatile uint16_t s_rx_tail = 0U;   /* written by main loop */

static volatile uint8_t  s_tx[TX_SIZE];
static volatile uint16_t s_tx_head = 0U;   /* written by main loop */
static volatile uint16_t s_tx_tail = 0U;   /* written by ISR */

volatile uint32_t g_uart_rx_bytes = 0U;
volatile uint32_t g_uart_rx_dropped = 0U;

void UartLink_Init(uint32_t baud)
{
    uint32_t pclk1;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR; /* read-back: let the clock settle */

    /* PA2 (TX) and PA3 (RX) -> alternate function 7 (USART2). */
    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));
    GPIOA->OSPEEDR |= ((2U << (2U * 2U)) | (2U << (3U * 2U)));
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->PUPDR |=  (1U << (3U * 2U));            /* RX pull-up: idle high if ESP32 is off */
    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U << (2U * 4U)) | (7U << (3U * 4U)));

    pclk1 = HAL_RCC_GetPCLK1Freq();                /* USART2 is on APB1 */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = (pclk1 + (baud / 2U)) / baud;    /* oversampling by 16 */

    s_rx_head = s_rx_tail = 0U;
    s_tx_head = s_tx_tail = 0U;

    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART2_IRQn, 2U);
    NVIC_EnableIRQ(USART2_IRQn);
}

int UartLink_ReadByte(void)
{
    uint8_t value;

    if (s_rx_tail == s_rx_head) return -1;
    value = s_rx[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (RX_SIZE - 1U));
    return (int)value;
}

bool UartLink_Write(const uint8_t *data, uint16_t len)
{
    uint16_t head = s_tx_head;
    uint16_t used = (uint16_t)((head - s_tx_tail) & (TX_SIZE - 1U));
    uint16_t free_space = (uint16_t)(TX_SIZE - 1U - used);

    if (len > free_space) return false;

    for (uint16_t i = 0U; i < len; i++)
    {
        s_tx[head] = data[i];
        head = (uint16_t)((head + 1U) & (TX_SIZE - 1U));
    }
    s_tx_head = head;
    USART2->CR1 |= USART_CR1_TXEIE;   /* start / continue transmission */
    return true;
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE))
    {
        uint8_t byte = (uint8_t)USART2->DR;   /* reading DR also clears ORE/FE/NE */
        if ((sr & (USART_SR_FE | USART_SR_NE | USART_SR_ORE)) == 0U)
        {
            uint16_t next = (uint16_t)((s_rx_head + 1U) & (RX_SIZE - 1U));
            if (next != s_rx_tail)
            {
                s_rx[s_rx_head] = byte;
                s_rx_head = next;
                g_uart_rx_bytes++;
            }
            else
            {
                g_uart_rx_dropped++;
            }
        }
        else
        {
            g_uart_rx_dropped++;
        }
    }

    if ((sr & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))
    {
        if (s_tx_tail != s_tx_head)
        {
            USART2->DR = s_tx[s_tx_tail];
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) & (TX_SIZE - 1U));
        }
        else
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}
