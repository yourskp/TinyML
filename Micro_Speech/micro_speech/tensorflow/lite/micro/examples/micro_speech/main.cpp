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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "tensorflow/lite/micro/examples/micro_speech/main_functions.h"

// This is the default main used on systems that have the standard C entry
// point. Other devices (for example FreeRTOS) that have different
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

#if 1 /* change retarget_io_init when loopback mode (I2S) is enabled as I2S MCLK and UART Rx share the same IO */
	/* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
								 CY_RETARGET_IO_BAUDRATE);
#else
	/* Initialize retarget-io to use the debug UART port on P12*/
	result = cy_retarget_io_init(P12_1, P12_0,
								 CY_RETARGET_IO_BAUDRATE);
#endif

	/* retarget-io init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	printf("\x1b[2J\x1b[;H");

	printf("****************** "
		 "PSoC 6 MCU: Machine Learning Microprocessor Speech Recognition Example "
		 "****************** \r\n\n");

	setup();
	while (true) {
	loop();
	}
}
