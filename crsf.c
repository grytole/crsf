/* crsf implementation for hardware usart (stm32f103c8t6) */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ring.h"
#include "crc8.h"
#include "crsf.h"

#define CRSF_USART (USART3)

#define CRSF_BAUDRATE       (420000)
#define CRSF_CRC_POLY       (0xD5)
#define CRSF_MAX_PACKET_LEN (64)
#define CRSF_SYNC           (0xC8)

#define CRSF_BUFFER_POS_SYNC     (0)
#define CRSF_BUFFER_POS_LEN      (1)
#define CRSF_BUFFER_POS_TYPE     (2)
#define CRSF_BUFFER_POS_PAYLOAD  (3)

#define CRSF_TYPE_RC_CHANNELS_PACKED (0x16)

#define CRSF_LEN_RC_CHANNELS_PACKED  (22)

#define CRSF_FAILSAFE_TIMEOUT_MS (100)

#if (CRSF_USART == USART1)
 #define CRSF_USART_RCC    (RCC_USART1)
 #define CRSF_GPIO_PORT_RX (GPIO_BANK_USART1_RX)
 #define CRSF_GPIO_PORT_TX (GPIO_BANK_USART1_TX)
 #define CRSF_GPIO_PIN_RX  (GPIO_USART1_RX)
 #define CRSF_GPIO_PIN_TX  (GPIO_USART1_TX)
 #define CRSF_USART_ISR    (usart1_isr)
 #define CRSF_USART_IRQ    (NVIC_USART1_IRQ)
#elif (CRSF_USART == USART2)
 #define CRSF_USART_RCC    (RCC_USART2)
 #define CRSF_GPIO_PORT_RX (GPIO_BANK_USART2_RX)
 #define CRSF_GPIO_PORT_TX (GPIO_BANK_USART2_TX)
 #define CRSF_GPIO_PIN_RX  (GPIO_USART2_RX)
 #define CRSF_GPIO_PIN_TX  (GPIO_USART2_TX)
 #define CRSF_USART_ISR    (usart2_isr)
 #define CRSF_USART_IRQ    (NVIC_USART2_IRQ)
#elif (CRSF_USART == USART3)
 #define CRSF_USART_RCC    (RCC_USART3)
 #define CRSF_GPIO_PORT_RX (GPIO_BANK_USART3_RX)
 #define CRSF_GPIO_PORT_TX (GPIO_BANK_USART3_TX)
 #define CRSF_GPIO_PIN_RX  (GPIO_USART3_RX)
 #define CRSF_GPIO_PIN_TX  (GPIO_USART3_TX)
 #define CRSF_USART_ISR    (usart3_isr)
 #define CRSF_USART_IRQ    (NVIC_USART3_IRQ)
#else
 #error Invalid USART device selected
#endif

typedef struct __attribute__((packed)) {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
  uint16_t ch8 : 11;
  uint16_t ch9 : 11;
  uint16_t ch10 : 11;
  uint16_t ch11 : 11;
  uint16_t ch12 : 11;
  uint16_t ch13 : 11;
  uint16_t ch14 : 11;
  uint16_t ch15 : 11;
} crsf_rc_channels_t;

static uint8_t rx_buf[255] = {};
static uint8_t tx_buf[255] = {};
static Ring rx_ring;
static Ring tx_ring;
static uint8_t crsf_buffer[CRSF_MAX_PACKET_LEN] = {};
static uint8_t crsf_buffer_pos = 0;
static uint16_t crsf_rc_channels[16] = {};

static bool crsf_parse_input(void);
static void crsf_parse_packet(uint8_t type, uint8_t *payload, uint8_t len);
static void crsf_reset_rc_channels_values(void);
static void crsf_store_rc_channels(uint8_t *p_payload, uint8_t len);

static bool crsf_parse_input(void)
{
  uint8_t ch = 0;
  uint8_t packet_len = 0;
  bool has_valid_crc = false;
  uint8_t *payload_ptr = NULL;

  while (true == ring_pop(&rx_ring, &ch))
  {
    if ((0 == crsf_buffer_pos) && (CRSF_SYNC != ch))
    {
      /* skip incoming data until sync byte */
      continue;
    }
    else
    {
      /* fill buffer with incoming data */
      crsf_buffer[crsf_buffer_pos++] = ch;
      if (CRSF_BUFFER_POS_LEN < crsf_buffer_pos)
      {
        /* data length value received */
        packet_len = crsf_buffer[CRSF_BUFFER_POS_LEN];
        if ((packet_len + 2) == crsf_buffer_pos)
        {
          /* full packet received */
          crsf_buffer_pos = 0;
          has_valid_crc = (crsf_buffer[packet_len + 1] == crc8_calc(&crsf_buffer[CRSF_BUFFER_POS_TYPE], packet_len - 1));
          if (true == has_valid_crc)
          {
            /* packet has valid checksum */
            payload_ptr = &crsf_buffer[CRSF_BUFFER_POS_PAYLOAD];
            crsf_parse_packet(crsf_buffer[CRSF_BUFFER_POS_TYPE], payload_ptr, packet_len - 2);
          }
        }
      }
    }
  }

  return has_valid_crc;
}

static void crsf_parse_packet(uint8_t type, uint8_t *payload, uint8_t len)
{
  if (NULL != payload)
  {
    switch (type)
    {
      case CRSF_TYPE_RC_CHANNELS_PACKED:
      {
        crsf_store_rc_channels(payload, len);
        break;
      }

      default:
      {
        break;
      }
    }
  }
}

static void crsf_reset_rc_channels_values(void)
{
  uint8_t i = 0;

  for (i = 0; i < 16; i++)
  { 
    crsf_rc_channels[i] = 0;
  }
}

static void crsf_store_rc_channels(uint8_t *p_payload, uint8_t len)
{
  if (NULL != p_payload)
  {
    crsf_rc_channels_t *p_rc_channels_packed = (crsf_rc_channels_t *)(p_payload);
    if (CRSF_LEN_RC_CHANNELS_PACKED == len)
    {
      crsf_rc_channels[0]  = p_rc_channels_packed->ch0;
      crsf_rc_channels[1]  = p_rc_channels_packed->ch1;
      crsf_rc_channels[2]  = p_rc_channels_packed->ch2;
      crsf_rc_channels[3]  = p_rc_channels_packed->ch3;
      crsf_rc_channels[4]  = p_rc_channels_packed->ch4;
      crsf_rc_channels[5]  = p_rc_channels_packed->ch5;
      crsf_rc_channels[6]  = p_rc_channels_packed->ch6;
      crsf_rc_channels[7]  = p_rc_channels_packed->ch7;
      crsf_rc_channels[8]  = p_rc_channels_packed->ch8;
      crsf_rc_channels[9]  = p_rc_channels_packed->ch9;
      crsf_rc_channels[10] = p_rc_channels_packed->ch10;
      crsf_rc_channels[11] = p_rc_channels_packed->ch11;
      crsf_rc_channels[12] = p_rc_channels_packed->ch12;
      crsf_rc_channels[13] = p_rc_channels_packed->ch13;
      crsf_rc_channels[14] = p_rc_channels_packed->ch14;
      crsf_rc_channels[15] = p_rc_channels_packed->ch15;
    }
  }
}

void CRSF_USART_ISR(void)
{
  uint8_t ch = 0;

  /* RX not empty */
  if (true == usart_get_flag(CRSF_USART, USART_SR_RXNE))
  {
    ch = (uint8_t)usart_recv(CRSF_USART);
    ring_push(&rx_ring, ch);
  }

  /* TX empty */
  if (true == usart_get_flag(CRSF_USART, USART_SR_TXE))
  {
    if (true == ring_pop(&tx_ring, &ch))
    {
      usart_send(CRSF_USART, ch);
    }
    else
    {
      usart_disable_tx_interrupt(CRSF_USART);
    }
  }
}

void crsf_init(void)
{
  /* init ring buffers */
  ring_init(&rx_ring, rx_buf, sizeof(rx_buf));
  ring_init(&tx_ring, tx_buf, sizeof(tx_buf));

  /* init crc8 lookup table */
  crc8_init(CRSF_CRC_POLY);

  /* configure usart */
  rcc_periph_clock_enable(CRSF_USART_RCC);
  gpio_set_mode(CRSF_GPIO_PORT_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, CRSF_GPIO_PIN_RX);
  gpio_set_mode(CRSF_GPIO_PORT_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, CRSF_GPIO_PIN_TX);
  usart_set_baudrate(CRSF_USART, CRSF_BAUDRATE);
  usart_set_databits(CRSF_USART, 8);
  usart_set_parity(CRSF_USART, USART_PARITY_NONE);
  usart_set_stopbits(CRSF_USART, USART_STOPBITS_1);
  usart_set_mode(CRSF_USART, USART_MODE_TX_RX);
  usart_set_flow_control(CRSF_USART, USART_FLOWCONTROL_NONE);
  nvic_enable_irq(CRSF_USART_IRQ);
  usart_enable_rx_interrupt(CRSF_USART);
  usart_enable(CRSF_USART);
}

void crsf_update(uint32_t timestamp_ms)
{
  static uint32_t last_valid_timestamp_ms = 0;

  if (CRSF_FAILSAFE_TIMEOUT_MS < (timestamp_ms - last_valid_timestamp_ms))
  {
    crsf_reset_rc_channels_values();
  }

  if (true == crsf_parse_input())
  {
    last_valid_timestamp_ms = timestamp_ms;
  }
}

uint16_t crsf_get_channel(uint8_t id)
{
  uint16_t val = 0;

  if (16 > id)
  {
    val = crsf_rc_channels[id];
  }

  return val;
}

