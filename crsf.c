/* crsf implementation for usart (stm32f103c8t6) */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include "ring.h"
#include "crc8.h"
#include "crsf.h"


/* defines */

#define CRSF_BAUDRATE       (420000)
#define CRSF_CRC_POLY       (0xD5)
#define CRSF_MAX_PACKET_LEN (64)
#define CRSF_POS_DEST       (0)
#define CRSF_POS_LEN        (1)
#define CRSF_POS_TYPE       (2)
#define CRSF_POS_PAYLOAD    (3)
#define CRSF_DEST_FC        (0xC8)
#define CRSF_TYPE_CHANNELS  (0x16)
#define CRSF_TYPE_HEARTBEAT (0x0B)
#define CRSF_TYPE_TEL_BATT  (0x08)
#define CRSF_LEN_CHANNELS   (22)
#define CRSF_LEN_HEARTBEAT  (2)
#define CRSF_LEN_TEL_BATT   (8)
#define CRSF_FAILSAFE_TIMEOUT_MS (100)


/* typedefs */

typedef struct __attribute__((packed)) {
  bool initialized;
  uint32_t usart;
  uint32_t gpio_port_rx;
  uint32_t gpio_port_tx;
  uint16_t gpio_pin_rx;
  uint16_t gpio_pin_tx;
  uint8_t irq;
  Ring rx_ring;
  Ring tx_ring;
} crsf_config_t;

typedef union {
  struct __attribute__((packed)) raw {
    uint8_t type;
    uint8_t data[61];
  };
  /* type: 0x02 */
  struct __attribute__((packed)) gps {
    uint8_t type;
    int32_t lat;
    int32_t lon;
    int16_t speed;
    int16_t course;
    uint16_t alt;
    uint8_t sat_count;
    uint8_t crc;
  };
  /* type: 0x08 */
  struct __attribute__((packed)) battery_sensor {
    uint8_t type;
    int16_t voltage;
    int16_t current;
    int32_t used_capacity : 24;
    int8_t battery_remaining;
    uint8_t crc;
  };
  /* type: 0x09 */
  struct __attribute__((packed)) baro_altitude {
    uint8_t type;
    uint16_t altitude;
    uint8_t crc;
  };
  /* type: 0x09 (alt) */
  struct __attribute__((packed)) baro_altitude_vario {
    uint8_t type;
    uint16_t altitude;
    int16_t vertical_speed;
    uint8_t crc;
  };
  /* type: 0x0B */
  struct __attribute__((packed)) heartbeat {
    uint8_t type;
    uint16_t origin_device_address;
    uint8_t crc;
  };
  /* type: 0x16 */
  struct __attribute__((packed)) rc_channels_packed {
    uint8_t type;
    uint16_t channels[16] : 11;
    uint8_t crc;
  };
  /* type: 0x1E */
  struct __attribute__((packed)) attitude {
    uint8_t type;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    uint8_t crc;
  };
  /* type: 0x21 (TODO) */
  struct __attribute__((packed)) flight_mode {
    uint8_t type;
    uint8_t mode_string[14];
    uint8_t crc;
  };
} crsf_payload_u;

typedef struct __attribute__((packed)) {
  uint8_t sync;
  uint8_t len;
  crsf_payload_u payload;
} crsf_packet_t;


/* variables */
static uint8_t rx_buf[256] = {};
static uint8_t tx_buf[256] = {};

static crsf_config_t crsf_config = {};


/* prototypes */

static bool crsf_init_config(crsf_config_t *p_config, uint32_t usart);
static bool crsf_check_packet_crc(void);


/* private functions */

static bool crsf_init_config(crsf_config_t *p_config, uint32_t usart)
{
  bool result = false;

  if (NULL != p_config)
  {
    p_config->usart = usart;
    switch (p_config->usart)
    {
      case USART1:
      {
        p_config->gpio_port_rx = GPIO_BANK_USART1_RX;
        p_config->gpio_port_tx = GPIO_BANK_USART1_TX;
        p_config->gpio_pin_rx = GPIO_USART1_RX;
        p_config->gpio_pin_tx = GPIO_USART1_TX;
        p_config->irq = NVIC_USART1_IRQ;
        p_config->initialized = true;
        break;
      }

      case USART2:
      {
        p_config->gpio_port_rx = GPIO_BANK_USART2_RX;
        p_config->gpio_port_tx = GPIO_BANK_USART2_TX;
        p_config->gpio_pin_rx = GPIO_USART2_RX;
        p_config->gpio_pin_tx = GPIO_USART2_TX;
        p_config->irq = NVIC_USART2_IRQ;
        p_config->initialized = true;
        break;
      }

      case USART3:
      {
        p_config->gpio_port_rx = GPIO_BANK_USART3_RX;
        p_config->gpio_port_tx = GPIO_BANK_USART3_TX;
        p_config->gpio_pin_rx = GPIO_USART3_RX;
        p_config->gpio_pin_tx = GPIO_USART3_TX;
        p_config->irq = NVIC_USART3_IRQ;
        p_config->initialized = true;
        break;
      }

      default:
      {
        break;
      }
    }
  }

  return result;
}

static bool crsf_check_packet_crc(void)
{
  uint8_t len = crsf_buf[CRSF_POS_LEN];
  uint8_t crc = crsf_buf[CRSF_POS_LEN + len];
  return (crc == crc8_calc(&crsf_buf[CRSF_POS_TYPE], len - 1));
}


/* public functions */

bool crsf_init(uint32_t usart)
{
  bool result = false;

  if (true == crsf_init_config(&crsf_config, usart))
  {
    /* init ring buffers */
    ring_init(&crsf_config.rx_ring, rx_buf, sizeof(rx_buf));
    ring_init(&crsf_config.tx_ring, tx_buf, sizeof(tx_buf));

    /* init crc8 lookup table */
    crc8_init(CRSF_CRC_POLY);

    /* configure usart */
    gpio_set_mode(crsf_config.gpio_port_rx, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, crsf_config.gpio_pin_rx);
    gpio_set_mode(crsf_config.gpio_port_tx, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, crsf_config.gpio_pin_tx);
    usart_set_baudrate(crsf_config.usart, CRSF_BAUDRATE);
    usart_set_databits(crsf_config.usart, 8);
    usart_set_parity(crsf_config.usart, USART_PARITY_NONE);
    usart_set_stopbits(crsf_config.usart, USART_STOPBITS_1);
    usart_set_mode(crsf_config.usart, USART_MODE_TX_RX);
    usart_set_flow_control(crsf_config.usart, USART_FLOWCONTROL_NONE);
    nvic_enable_irq(crsf_config.irq);
    usart_enable_rx_interrupt(crsf_config.usart);
    usart_enable(crsf_config.usart);

    result = true;
  }

  return result;
}

uint16_t crsf_get_channel(uint8_t id)
{
  uint16_t val = 0;
  /* TODO */
  return val;
}

void crsf_update(uint32_t timestamp_ms)
{
}
