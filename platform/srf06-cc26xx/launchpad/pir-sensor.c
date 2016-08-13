/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/*
 * Driver for HC-SR501 PIR sensor using GPIO pin 21
 * BOARD_IOID_DIO21 = IOID_21 = 0x00000015  
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "launchpad/pir-sensor.h"

#include "ti-lib.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
#define PIR_GPIO_CFG            (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_DOWN  | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
#define DEBOUNCE_DURATION (CLOCK_SECOND >> 5)

struct debounce_timer {
  struct timer debounce;
  clock_time_t start;
  clock_time_t duration;
};

static struct debounce_timer pir_debounce_timer;

/*---------------------------------------------------------------------------*/
static void
motion_detected_handler(uint8_t ioid)
{
    if(ioid == BOARD_IOID_DIO21) {
        if(!timer_expired(&pir_debounce_timer.debounce)) {
        return;
    }

    timer_set(&pir_debounce_timer.debounce, DEBOUNCE_DURATION);

    /*
     * Start PIR motion duration counter on press (rising), notify on release
     * (falling)
     */

    if(ti_lib_gpio_pin_read(1 << BOARD_IOID_DIO21) != 0) {
      pir_debounce_timer.start = clock_time();
      pir_debounce_timer.duration = 0;
    } else {
      pir_debounce_timer.duration = clock_time() - pir_debounce_timer.start;
      sensors_changed(&pir_sensor);
    }
  }
}


/*---------------------------------------------------------------------------*/
static void config_pirs(int type, int c, uint32_t key)
{
	switch(type) {
	case SENSORS_HW_INIT:
	ti_lib_gpio_event_clear(1 << key);

	ti_lib_ioc_port_configure_set(key, IOC_PORT_GPIO, PIR_GPIO_CFG);
	
	ti_lib_gpio_dir_mode_set((1 << key), GPIO_DIR_MODE_IN);
	gpio_interrupt_register_handler(key, motion_detected_handler);
    break;
    
    case SENSORS_ACTIVE:
    if(c) {
      ti_lib_gpio_event_clear(1 << key);
      ti_lib_ioc_port_configure_set(key, IOC_PORT_GPIO, PIR_GPIO_CFG);
      ti_lib_gpio_dir_mode_set((1 << key), GPIO_DIR_MODE_IN);
      ti_lib_ioc_int_enable(key);
    } else {
      ti_lib_ioc_int_disable(key);
    }
    break;
  default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
static int
config_pir(int type, int value)
{
  config_pirs(type, value, BOARD_IOID_DIO21);

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
status_pir(int type, uint32_t key_io_id)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    if(ti_lib_ioc_port_configure_get(key_io_id) & IOC_INT_ENABLE) {
      return 1;
    }
    break;
  default:
    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value_pir(int type)
{
  if(type == PIR_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_pin_read(1 << BOARD_IOID_DIO21) != 0 ?
           PIR_SENSOR_VALUE_ACTIVE : PIR_SENSOR_VALUE_INACTIVE;
  } else if(type == PIR_SENSOR_VALUE_DURATION) {
    return (int)pir_debounce_timer.duration;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/

SENSORS_SENSOR(pir_sensor, PIR_SENSOR, value_pir, config_pir,
               status_pir);
               
/*---------------------------------------------------------------------------*/
/** @} */
