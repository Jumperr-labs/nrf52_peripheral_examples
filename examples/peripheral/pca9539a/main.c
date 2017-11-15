/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup PCA9539A main.c
 * @{
 * @ingroup PCA9539A_example
 * @brief PCA9539A Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_drv_twi.h"
#include "app_uart.h"
#include <string.h>
#include <stdio.h>
#include "nrf_delay.h"


enum reg_type {
    INPUT_PORT_0,
    INPUT_PORT_1,
    OUTPUT_PORT_0,
    OUTPUT_PORT_1,
    POLARITY_INVERSION_PORT_0,
    POLARITY_INVERSION_PORT_1,
    CONFIGURATION_PORT_0,
    CONFIGURATION_PORT_1,
};



#define A0 31
#define A1 30
#define INT 29
#define RESET 28
#define P00 11
#define P01 12
#define P02 13
#define P03 14
#define P04 15
#define P05 16
#define P06 17
#define P07 18
#define P10 19
#define P11 20
#define P12 21
#define P13 22
#define P14 23
#define P15 24
#define P16 25
#define P17 3

#define P0 0
#define P1 1
#define P2 2
#define P3 3
#define P4 4
#define P5 5
#define P6 6
#define P7 7



#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256   /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256   /**< UART RX buffer size. */
/* pca9539a instance ID. */
#define PCA9539_INSTANCE_ID 0

#define RUN_TEST(TestFunc, TestLineNum) \
{ \
  Unity.CurrentTestName = #TestFunc; \
  Unity.CurrentTestLineNumber = TestLineNum; \
  Unity.NumberOfTests++; \
  if (TEST_PROTECT()) \
  { \
      setUp(); \
      TestFunc(); \
  } \
  if (TEST_PROTECT()) \
  { \
    tearDown(); \
  } \
  UnityConcludeTest(); \
}


/*=======Automagically Detected Files To Include=====*/
#include "unity.h"
#include "app_error.h"
#include <stdio.h>
#include <setjmp.h>

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/*=======Read write Functions =====*/

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(PCA9539_INSTANCE_ID);

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/*name PCA9539addresses */
uint8_t dev_id = 0x77;

int8_t my_i2c_rx(uint8_t addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, addr, reg_data, len);
    if (err_code != NRF_SUCCESS){
        rslt = -1;
    } else {
        // nrf_delay_ms(500);
        do
        {
            __WFE();
        } while (m_xfer_done == false);
        m_xfer_done = false;
    }
    return rslt;
}

int8_t my_i2c_tx(uint8_t addr, uint8_t *reg_data, uint16_t len, bool no_stop) {
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    m_xfer_done = false;
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, addr, reg_data, len, no_stop);
    APP_ERROR_CHECK(err_code);
    do
    {
        __WFE();
    } while (m_xfer_done == false);
    m_xfer_done = false;
    return rslt;
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    rslt = my_i2c_tx(dev_id, &reg_addr, 1, true);
    if (rslt == 0) {
        rslt = my_i2c_rx(dev_id, reg_data, len);
    }

    return rslt;
}

int8_t user_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t * reg_addr_and_data = (uint8_t *) malloc(sizeof(uint8_t) * (len + 1));
    memcpy(reg_addr_and_data , &reg_addr, sizeof(uint8_t));
    memcpy(reg_addr_and_data + sizeof(uint8_t), reg_data, sizeof(uint8_t) * len);
    rslt = my_i2c_tx(dev_id, reg_addr_and_data, len + 1, false);

    free(reg_addr_and_data);
    return rslt;
}


void pca9539_read(enum reg_type regType, uint8_t *reg_data, uint16_t len){
    uint8_t addr = 0;
    switch(regType){
        case INPUT_PORT_0               : addr=0x00; break;
        case INPUT_PORT_1               : addr=0x01; break;
        case OUTPUT_PORT_0              : addr=0x02; break;
        case OUTPUT_PORT_1              : addr=0x03; break;
        case POLARITY_INVERSION_PORT_0  : addr=0x04; break;
        case POLARITY_INVERSION_PORT_1  : addr=0x05; break;
        case CONFIGURATION_PORT_0       : addr=0x06; break;
        case CONFIGURATION_PORT_1       : addr=0x07; break;
        default                         : //NRF_LOG_ERROR("No such reg type for read action");
                                            //NRF_LOG_FLUSH();
                                            break;
    }

    int8_t rslt = user_i2c_read(addr, reg_data, len);
    if (rslt != 0) {
        return;
    }
}

void pca9539_write(enum reg_type regType, uint8_t *reg_data, uint16_t len){
    uint8_t addr = 0;
    switch(regType){
        case OUTPUT_PORT_0              : addr=0x02; break;
        case OUTPUT_PORT_1              : addr=0x03; break;
        case POLARITY_INVERSION_PORT_0  : addr=0x04; break;
        case POLARITY_INVERSION_PORT_1  : addr=0x05; break;
        case CONFIGURATION_PORT_0       : addr=0x06; break;
        case CONFIGURATION_PORT_1       : addr=0x07; break;
        default                         : //NRF_LOG_ERROR("No such reg type for write action");
                                            //NRF_LOG_FLUSH();
                                            break;

    }

    int8_t rslt = user_i2c_write(addr, reg_data, len);
    if (rslt != 0) {
        return;
    }
}

bool pca9539_read_pin(enum reg_type regType, uint32_t pinNumber){
    uint8_t one_mask = 1;
    bool isHigh = false;
    uint8_t * reg_data = (uint8_t *) malloc(sizeof(uint8_t));
    pca9539_read(regType, reg_data, 1);

    uint8_t mask = one_mask << pinNumber;
    if ((mask & (*reg_data)) != 0 ){
        isHigh = true;
    }
    free(reg_data);
    return isHigh;
}

void pca9539_set_pin(enum reg_type regType, int pinNumber){
    if (pinNumber > 7 || pinNumber < 0){
        return;
    }
    uint8_t one_mask = 1;
    uint8_t mask = one_mask << pinNumber;
    uint8_t * reg_data = (uint8_t *) malloc(sizeof(uint8_t));
    pca9539_read(regType, reg_data, 1);
    *(reg_data) |= mask;
    pca9539_write(regType, reg_data, 1);
    free(reg_data);
}

void pca9539_clear_pin(enum reg_type regType, int pinNumber){
    if (pinNumber > 7 || pinNumber < 0){
        return;
    }
    uint8_t one_mask = 1;
    uint8_t mask = one_mask << pinNumber;
    uint8_t * reg_data = (uint8_t *) malloc(sizeof(uint8_t));
    pca9539_read(regType, reg_data, 1);
    *(reg_data) &= ~mask;
    pca9539_write(regType, reg_data, 1);
    free(reg_data);
}


/*=======Read write Functions end =====*/

void pca_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){
    switch (p_event->type){
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief pca9539 initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
          .scl                = ARDUINO_SCL_PIN,
          .sda                = ARDUINO_SDA_PIN,
          .frequency          = NRF_TWI_FREQ_100K,
          .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
          .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, pca_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/*=======Test Functions=====*/
/* sometimes you may want to get at local data in a module.
 * for example: If you plan to pass by reference, this could be useful
 * however, it should often be avoided */

void setUp(void)
{
    nrf_gpio_cfg_output(RESET);
    nrf_gpio_pin_write(RESET, 1);
    nrf_gpio_cfg_output(A0);
    nrf_gpio_pin_write(A0, 1);
    nrf_gpio_cfg_output(A1);
    nrf_gpio_pin_write(A1, 1);
}

void tearDown(void)
{
}

void test1(void){
    nrf_gpio_cfg_input(P04, NRF_GPIO_PIN_NOPULL);
    int level = 0;
    // Change configuration of p0 (clear P4 so will be consider as output)
    pca9539_clear_pin(CONFIGURATION_PORT_0,P4);

    // Set P04 to 0
    pca9539_clear_pin(OUTPUT_PORT_0,P4);
    //Check result
    bool isHigh = pca9539_read_pin(OUTPUT_PORT_0,P4);
    TEST_ASSERT_FALSE(isHigh);
    level = nrf_gpio_pin_read(P04);
    TEST_ASSERT_EQUAL(0, level);

    // Set P04 to 1
    pca9539_set_pin(OUTPUT_PORT_0,P4);
    //Check result
    isHigh = pca9539_read_pin(OUTPUT_PORT_0,P4);
    TEST_ASSERT_TRUE(isHigh);
    level = nrf_gpio_pin_read(P04);
    TEST_ASSERT_EQUAL(1, level);

    // Set P04 to 0
    pca9539_clear_pin(OUTPUT_PORT_0,P4);
    //Check result
    isHigh = pca9539_read_pin(OUTPUT_PORT_0,P4);
    TEST_ASSERT_FALSE(isHigh);
    level = nrf_gpio_pin_read(P04);
    TEST_ASSERT_EQUAL(0, level);
}

void test2(void){
    nrf_gpio_cfg_input(P15, NRF_GPIO_PIN_NOPULL);
    int level = 0;
    // Change configuration of p1 (clear P5 so will be consider as output)
    pca9539_clear_pin(CONFIGURATION_PORT_1,P5);
    // Set P15 to 0
    pca9539_clear_pin(OUTPUT_PORT_1,P5);
    //Check result
    bool isHigh = pca9539_read_pin(OUTPUT_PORT_1,P5);
    TEST_ASSERT_FALSE(isHigh);
    level = nrf_gpio_pin_read(P15);
    TEST_ASSERT_EQUAL(0, level);

    // Change configuration of p1 (set P5 so will be consider as input)
    pca9539_set_pin(CONFIGURATION_PORT_1,P5);
    isHigh = pca9539_read_pin(INPUT_PORT_1,P5);
    //Check result
    TEST_ASSERT_FALSE(isHigh);
    level = nrf_gpio_pin_read(P15);
    TEST_ASSERT_EQUAL(0, level);

    // Change inversion of p1 (set P5 so the input will be invert)
    pca9539_set_pin(POLARITY_INVERSION_PORT_1,P5);
    //Check result
    isHigh = pca9539_read_pin(INPUT_PORT_1,P5);
    TEST_ASSERT_TRUE(isHigh);
    level = nrf_gpio_pin_read(P15);
    TEST_ASSERT_EQUAL(0, level);
}

void test3(void){
    // Set p03 value to 1
    nrf_gpio_cfg_output(P03);
    nrf_gpio_pin_write(P03, 1);

    // Check result
    bool isHigh = pca9539_read_pin(INPUT_PORT_0,P3);
    TEST_ASSERT_TRUE(isHigh);
    int level = nrf_gpio_pin_out_read(P03);
    TEST_ASSERT_EQUAL(1, level);

    // Set p03 value to 0
    nrf_gpio_pin_write(P03, 0);

    // Check result
    isHigh = pca9539_read_pin(INPUT_PORT_0,P3);
    TEST_ASSERT_FALSE(isHigh);
    level = nrf_gpio_pin_out_read(P03);
    TEST_ASSERT_EQUAL(0, level);
}

void test4(void){
    // Check Init int value
    nrf_gpio_cfg_input(INT, NRF_GPIO_PIN_NOPULL);
    int level = nrf_gpio_pin_read(INT);
    TEST_ASSERT_EQUAL(1, level);

    // Read input pin po2 change is value
    nrf_gpio_cfg_output(P02);
    level = nrf_gpio_pin_read(P02);
    nrf_gpio_pin_write(P02, !level);

    // Check that interrupt set to 0
    level = nrf_gpio_pin_read(INT);
    TEST_ASSERT_EQUAL(0, level);

    // Make read from port 0
    pca9539_read_pin(INPUT_PORT_0,P6);

    // Check that interrupt set to 1
    level = nrf_gpio_pin_read(INT);
    TEST_ASSERT_EQUAL(1, level);

}
/*=======Test Functions end=====*/


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    twi_init();
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
          {
                RX_PIN_NUMBER,
                TX_PIN_NUMBER,
                RTS_PIN_NUMBER,
                CTS_PIN_NUMBER,
                APP_UART_FLOW_CONTROL_ENABLED,
                false,
                UART_BAUDRATE_BAUDRATE_Baud115200
          };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
    UnityBegin("main.c");
    RUN_TEST(test1, 20);
    RUN_TEST(test2, 30);
    RUN_TEST(test3, 40);
    RUN_TEST(test4, 50);
    UnityEnd();

    while (true)
    {
    }
}

/** @} */
