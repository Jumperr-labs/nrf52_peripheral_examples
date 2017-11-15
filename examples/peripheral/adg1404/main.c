/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 *
 * @defgroup adg1404_example_main main.c
 * @{
 * @ingroup adg1404_example
 * @brief adg1404 Example Application main file.
 *
 * This file contains the source code for a sample application to adg1404 mux.
 *
 */

#include <stdint.h>
#include <nrf_drv_gpiote.h>
#include "nrf_delay.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define HIGH 1
#define LOW  0

#define INPUT_PIN_NUMBER 4

#define S1 ARDUINO_0_PIN
#define S2 ARDUINO_1_PIN
#define S3 ARDUINO_2_PIN
#define S4 ARDUINO_3_PIN
#define A0 ARDUINO_4_PIN
#define A1 ARDUINO_5_PIN
#define EN ARDUINO_6_PIN
#define D  ARDUINO_7_PIN

int isSuccess;

static void bsp_board_pins_init() {

    // input static pins
    nrf_gpio_cfg_output(S1);
    nrf_gpio_pin_write(S1, HIGH);
    nrf_gpio_cfg_output(S2);
    nrf_gpio_pin_write(S2, LOW);
    nrf_gpio_cfg_output(S3);
    nrf_gpio_pin_write(S3, HIGH);
    nrf_gpio_cfg_output(S4);
    nrf_gpio_pin_write(S4, LOW);

    // enabled pin
    nrf_gpio_cfg_output(EN);
    nrf_gpio_pin_write(EN, HIGH);

    // input dynamic pins
    nrf_gpio_cfg_output(A0);
    nrf_gpio_pin_write(A0, LOW);
    nrf_gpio_cfg_output(A1);
    nrf_gpio_pin_write(A1, LOW);

    nrf_gpio_cfg_input(D, NRF_GPIO_PIN_NOPULL);
}


static void bsp_board_pins_change_input_output_direction() {

    nrf_gpio_cfg_output(EN);
    nrf_gpio_pin_write(EN, LOW);

    // input static pins
    nrf_gpio_cfg_input(S1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(S2, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(S3, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(S4, NRF_GPIO_PIN_NOPULL);

    // input dynamic pins
    nrf_gpio_cfg_output(A0);
    nrf_gpio_pin_write(A0, LOW);
    nrf_gpio_cfg_output(A1);
    nrf_gpio_pin_write(A1, LOW);

    nrf_gpio_cfg_output(D);
    nrf_gpio_pin_write(D, LOW);

    nrf_gpio_pin_write(EN, HIGH);

}


static void log_pin() {

    NRF_LOG_DEBUG("\r\nCheck ADG1404 Start!\r\n");
    NRF_LOG_DEBUG("S1 number [%d] value [%d]\r\n", S1, (int) nrf_gpio_pin_out_read(S1));
    NRF_LOG_DEBUG("S2 number [%d] value [%d]\r\n", S2, (int) nrf_gpio_pin_out_read(S2));
    NRF_LOG_DEBUG("S3 number [%d] value [%d]\r\n", S3, (int) nrf_gpio_pin_out_read(S3));
    NRF_LOG_DEBUG("S4 number [%d] value [%d]\r\n", S4, (int) nrf_gpio_pin_out_read(S4));
    NRF_LOG_DEBUG("EN number [%d] value [%d]\r\n", EN, (int) nrf_gpio_pin_out_read(EN));
    NRF_LOG_DEBUG("A0 number [%d] value [%d]\r\n", A0, (int) nrf_gpio_pin_out_read(A0));
    NRF_LOG_DEBUG("A1 number [%d] value [%d]\r\n", A1, (int) nrf_gpio_pin_out_read(A1));
    NRF_LOG_DEBUG("D number [%d] value [%d]\r\n", D, (int) nrf_gpio_pin_read(D));

    NRF_LOG_FLUSH();
}

static void log_pin_input() {

    NRF_LOG_DEBUG("\r\nCheck ADG1404 Start!\r\n");
    NRF_LOG_DEBUG("S1 number [%d] value [%d]\r\n", S1, (int) nrf_gpio_pin_read(S1));
    NRF_LOG_DEBUG("S2 number [%d] value [%d]\r\n", S2, (int) nrf_gpio_pin_read(S2));
    NRF_LOG_DEBUG("S3 number [%d] value [%d]\r\n", S3, (int) nrf_gpio_pin_read(S3));
    NRF_LOG_DEBUG("S4 number [%d] value [%d]\r\n", S4, (int) nrf_gpio_pin_read(S4));
    NRF_LOG_DEBUG("EN number [%d] value [%d]\r\n", EN, (int) nrf_gpio_pin_out_read(EN));
    NRF_LOG_DEBUG("A0 number [%d] value [%d]\r\n", A0, (int) nrf_gpio_pin_out_read(A0));
    NRF_LOG_DEBUG("A1 number [%d] value [%d]\r\n", A1, (int) nrf_gpio_pin_out_read(A1));
    NRF_LOG_DEBUG("D number [%d] value [%d]\r\n", D, (int) nrf_gpio_pin_out_read(D));

    NRF_LOG_FLUSH();
}

static void compareOutputToSource(uint32_t sourceNumber) {
    if (nrf_gpio_pin_read(D) != nrf_gpio_pin_out_read(sourceNumber)) {
        NRF_LOG_DEBUG("Error: output pin number [%d] value [%d] and source pin number [%d] value [%d] has different values\r\n",
                     D, nrf_gpio_pin_read(D), sourceNumber,
                     nrf_gpio_pin_out_read(sourceNumber));
        isSuccess = 0;

    } else {
        NRF_LOG_DEBUG("Success: output pin number [%d] value [%d] and source pin number [%d] value [%d] are equal\r\n",
                      D, nrf_gpio_pin_read(D), sourceNumber,
                      nrf_gpio_pin_out_read(sourceNumber));
    }
    NRF_LOG_FLUSH();
}

static void checkOutputValue(uint32_t a0_value, uint32_t a1_value) {
    NRF_LOG_DEBUG("Before pin a0 number [%d] value [%d] pin a1 number [%d] value [%d]\r\n", A0, a0_value, A1, a1_value);

    short int sourcePinNumber = a0_value | (a1_value << 1);
    NRF_LOG_DEBUG("After pin a0 number [%d] value [%d] pin a1 number [%d] value [%d]\r\n", A0, a0_value, A1, a1_value);
    NRF_LOG_DEBUG("Source number [%d]\r\n", sourcePinNumber);
    NRF_LOG_FLUSH();

    switch (sourcePinNumber) {
        case 0:
            compareOutputToSource(S1);
            break;
        case 1:
            compareOutputToSource(S2);
            break;
        case 2:
            compareOutputToSource(S3);
            break;
        case 3:
            compareOutputToSource(S4);
            break;
        default:
            NRF_LOG_DEBUG("Error source pin number [%d] Invalid\r\n", sourcePinNumber);
            isSuccess = 0;
            NRF_LOG_FLUSH();
            break;
    }
}

static void compareSourceToOutput(uint32_t sourceNumber) {
    if (nrf_gpio_pin_out_read(D) != nrf_gpio_pin_read(sourceNumber)) {
        NRF_LOG_DEBUG("Error: output pin number [%d] value [%d] and source pin number [%d] value [%d] has different values\r\n",
                      D, nrf_gpio_pin_out_read(D), sourceNumber,
                      nrf_gpio_pin_read(sourceNumber));
        isSuccess = 0;

    } else {
        NRF_LOG_DEBUG("Success: output pin number [%d] value [%d] and source pin number [%d] value [%d] are equal\r\n",
                      D, nrf_gpio_pin_out_read(D), sourceNumber,
                      nrf_gpio_pin_read(sourceNumber));
    }
    NRF_LOG_FLUSH();
}

static void checkSourceValue(uint32_t a0_value, uint32_t a1_value) {
    NRF_LOG_DEBUG("Before pin a0 number [%d] value [%d] pin a1 number [%d] value [%d]\r\n", A0, a0_value, A1, a1_value);

    uint32_t sourcePinNumber = a0_value | (a1_value << 1);
    NRF_LOG_DEBUG("After pin a0 number [%d] value [%d] pin a1 number [%d] value [%d]\r\n", A0, a0_value, A1, a1_value);
    NRF_LOG_DEBUG("Source number [%d]\r\n", sourcePinNumber);
    NRF_LOG_FLUSH();

    switch (sourcePinNumber) {
        case 0:
            compareSourceToOutput(S1);
            break;
        case 1:
            compareSourceToOutput(S2);
            break;
        case 2:
            compareSourceToOutput(S3);
            break;
        case 3:
            compareSourceToOutput(S4);
            break;
        default:
            NRF_LOG_DEBUG("Error source pin number [%d] Invalid\r\n", sourcePinNumber);
            isSuccess = 0;
            NRF_LOG_FLUSH();
            break;
    }
}

static uint32_t getMask(uint32_t a0, uint32_t a1){
    NRF_GPIO_Type * reg        = nrf_gpio_pin_port_decode((uint32_t *) A0);
    uint32_t        pins_state = reg->OUT;
    uint32_t mask_a0 = 1U << A0;
    uint32_t mask_a1 = 1U << A1;
    if (a0){
        pins_state |= mask_a0 ;
    }
    else{
        pins_state &= ~mask_a0;
    }

    if (a1){
        pins_state |= mask_a1 ;
    }
    else{
        pins_state &= ~mask_a1;
    }

    return pins_state;
}
/**
 * @brief Function for application main entry.
 */
int main(void) {

    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    /* Configure pins D output. */
    bsp_board_pins_init();

    uint32_t mask_one = 1;
    isSuccess = 1;

    NRF_LOG_DEBUG("\r\nCheck ADG1404 D output!\r\n");
    NRF_LOG_FLUSH();

    /* Toggle Pins. */
    for (uint32_t i = 0; i < INPUT_PIN_NUMBER; i++) {
        // gets LSB
        uint32_t a0_value = i & mask_one;
        // gets second LSB
        uint32_t a1_value = (i >> 1) & mask_one;

        NRF_LOG_DEBUG("\r\nBEFORE!\r\n");
        NRF_LOG_FLUSH();
        log_pin();

        // update ao and a1 input value
        NRF_GPIO_Type * reg        = nrf_gpio_pin_port_decode((uint32_t *) A0);
        nrf_gpio_port_out_write(reg, getMask(a0_value, a1_value));

        // wait 500 ms and start test again
        nrf_delay_ms(10);

        NRF_LOG_DEBUG("\r\nAFTER!\r\n")
        NRF_LOG_DEBUG("D umber [%d] value [%d]\r\n", D, (int) nrf_gpio_pin_read(D));
        NRF_LOG_FLUSH();
        log_pin();

        checkOutputValue(a0_value, a1_value);
    }

    bsp_board_pins_change_input_output_direction();

    NRF_LOG_DEBUG("\r\nCheck ADG1404 D input!\r\n");
    NRF_LOG_FLUSH();

    /* Toggle Pins. s are output*/
    for (uint32_t i = 0; i < INPUT_PIN_NUMBER; i++) {
        // gets LSB
        uint32_t a0_value = i & mask_one;
        // gets second LSB
        uint32_t a1_value = (i >> 1) & mask_one;

        NRF_LOG_DEBUG("\r\nBEFORE!\r\n");
        NRF_LOG_FLUSH();
        log_pin_input();

        // update ao and a1 input value
        NRF_GPIO_Type * reg        = nrf_gpio_pin_port_decode((uint32_t *) A0);
        nrf_gpio_port_out_write(reg, getMask(a0_value, a1_value));

        // wait 500 ms and start test again
        nrf_delay_ms(10);

        NRF_LOG_DEBUG("\r\nAFTER!\r\n")
        NRF_LOG_FLUSH();
        log_pin_input();

        checkSourceValue(a0_value, a1_value);
    }

    nrf_gpio_pin_write(D, HIGH);

    /* Toggle Pins. s are output*/
    for (uint32_t i = 0; i < INPUT_PIN_NUMBER; i++) {
        // gets LSB
        uint32_t a0_value = i & mask_one;
        // gets second LSB
        uint32_t a1_value = (i >> 1) & mask_one;

        NRF_LOG_DEBUG("\r\nBEFORE!\r\n");
        NRF_LOG_FLUSH();
        log_pin_input();

        // update ao and a1 input value
        NRF_GPIO_Type * reg        = nrf_gpio_pin_port_decode((uint32_t *) A0);
        nrf_gpio_port_out_write(reg, getMask(a0_value, a1_value));

        // wait 500 ms and start test again
        nrf_delay_ms(10);

        NRF_LOG_DEBUG("\r\nAFTER!\r\n")
        NRF_LOG_FLUSH();
        log_pin_input();

        checkSourceValue(a0_value, a1_value);
    }


    while (true) {
        if (isSuccess == 1) {
            NRF_LOG_INFO("Success!\r\n")
            NRF_LOG_FLUSH();
        }
    }
}

/**
 *@}
 **/
