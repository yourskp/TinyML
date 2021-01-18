/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "tensorflow/lite/micro/examples/hello_world/main_functions.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

// This is the default main used on systems that have the standard C entry
// point. Other devices (for example FreeRTOS or ESP32) that have different
// requirements for entry code (like an app_main function) should specialize
// this main.cc file in a target-specific subfolder.
int main(int argc, char* argv[]) {

  cy_rslt_t result;

  /* Initialize the device and board peripherals */
  result = cybsp_init();

  /* Board init failed. Stop program execution */
  if (result != CY_RSLT_SUCCESS)
  {
	  CY_ASSERT(0);
  }

  /* Enable global interrupts */
  __enable_irq();

  /* Initialize retarget-io to use the debug UART port */
  result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
							   CY_RETARGET_IO_BAUDRATE);

  /* retarget-io init failed. Stop program execution */
  if (result != CY_RSLT_SUCCESS)
  {
	  CY_ASSERT(0);
  }

  /* Initialize the User LED */
  result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
						   CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

  /* GPIO init failed. Stop program execution */
  if (result != CY_RSLT_SUCCESS)
  {
	  CY_ASSERT(0);
  }

  /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
  printf("\x1b[2J\x1b[;H");

  printf("****************** "
		 "PSoC 6 MCU: Machine Learning Hello World! Example "
		 "****************** \r\n\n");

  setup();
  while (true) {
    loop();
	Cy_SysLib_Delay(1000);
  }
}
