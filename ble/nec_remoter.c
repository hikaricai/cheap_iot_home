#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nec_remoter.h"
#define NEC_DATA_DUTY 560/4
#define NEC_DATA_DUTY_REVERSE NEC_DATA_DUTY|0x8000
#define NEC_DATA0_PERIOD NEC_DATA_DUTY*2|0x8000
#define NEC_DATA1_PERIOD NEC_DATA_DUTY*4|0x8000
#define NEC_HEAD_PERIOD NEC_DATA_DUTY*24|0x8000
#define NEC_HEAD_DUTY NEC_DATA_DUTY*16|0x8000
#define NEC_38K_PERIOD 26
#define NEC_38K_DUTY NEC_38K_PERIOD*2/3
#define NEC_IR_PIN 23
#define NEC_38K_PIN 24

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
typedef enum
{
    NEC_HEAD,
    NEC_WAIT,
    NEC_DATA
}nec_state;
typedef struct 
{
    uint32_t data;
    uint8_t head_idx;
    nec_state state;
}nec_remoter;
static nec_remoter m_remoter;


// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
static nrf_pwm_values_individual_t m_nec_seq_values;
static nrf_pwm_values_individual_t m_38k_seq_values;
static nrf_pwm_sequence_t const    m_nec_seq =
{
    .values.p_individual = &m_nec_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_nec_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};
static nrf_pwm_sequence_t const    m_38k_seq =
{
    .values.p_individual = &m_38k_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_nec_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static bool m_send_ok=true;
static void nec_handler(nrf_drv_pwm_evt_type_t event_type)
{
    //nrf_gpio_pin_toggle(NEC_DEBUG_PIN);

    if(m_remoter.state==NEC_HEAD)
    {
        m_remoter.state = NEC_WAIT;
        m_nec_seq_values.channel_0 = NEC_HEAD_DUTY;
        //nrf_gpio_pin_toggle(NEC_DEBUG_PIN);
        return;
    }
    else if(m_remoter.state==NEC_WAIT)
    {
        m_remoter.state = NEC_DATA;
        m_nec_seq_values.channel_0 = NEC_DATA_DUTY_REVERSE;
        //nrf_gpio_pin_toggle(NEC_DEBUG_PIN);
        return;
    }

    if(m_remoter.head_idx<32)
    {
        if(m_remoter.head_idx==0)
        {
            m_nec_seq_values.channel_0 = NEC_DATA_DUTY_REVERSE;
            nrf_drv_pwm_simple_playback(&m_pwm1, &m_38k_seq, 1,
                                        NRF_DRV_PWM_FLAG_LOOP);     
        }
        
        if(m_remoter.data&(1<<m_remoter.head_idx))//send 1
        {
            m_pwm0.p_registers->COUNTERTOP = NEC_DATA1_PERIOD;
        }
        else
        {
            m_pwm0.p_registers->COUNTERTOP = NEC_DATA0_PERIOD;
        }

    }
    else if(m_remoter.head_idx==33)
    {
        m_send_ok = true;
        nrf_drv_pwm_stop(&m_pwm0,true);
        nrf_drv_pwm_stop(&m_pwm1,true);
    }

    m_remoter.head_idx ++;
    //nrf_gpio_pin_toggle(NEC_DEBUG_PIN);
}
void nec_init(void)
{
    uint32_t                   err_code;
    nrf_drv_pwm_config_t const config_nec =
    {
        .output_pins =
        {
            NEC_IR_PIN | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_250kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = NEC_HEAD_PERIOD,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config_nec, nec_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_pwm_config_t const config_38k =
    {
        .output_pins =
        {
            NEC_38K_PIN | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = NEC_38K_PERIOD,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    m_38k_seq_values.channel_0 = NEC_38K_DUTY;
    err_code = nrf_drv_pwm_init(&m_pwm1, &config_38k, NULL);
    APP_ERROR_CHECK(err_code);
}

void nec_send(uint8_t address,uint8_t comm)
{
    if(m_send_ok == false)
        return;
    m_send_ok = false;
    uint32_t* data = &m_remoter.data;
    m_remoter.head_idx = 0;
    m_remoter.state = NEC_HEAD;
    //remoter->data = 0x0f0f0f0f;
    *data = 0;
    *data |= address;
    *data |= (address^0xff)<<8;
    *data |= comm<<16;
    *data |= (comm^0xff)<<24;
    m_pwm0.p_registers->COUNTERTOP = NEC_HEAD_PERIOD;
    m_nec_seq_values.channel_0 = 0;
    nrf_drv_pwm_simple_playback(&m_pwm0, &m_nec_seq, 1,
                                NRF_DRV_PWM_FLAG_LOOP);
}

