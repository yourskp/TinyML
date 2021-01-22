/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

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

#include "tensorflow/lite/micro/examples/micro_speech/command_responder.h"
#include "stdio.h"

// When a command is detected, write it to the display and log it to the
// serial port.
void RespondToCommand(tflite::ErrorReporter* error_reporter,
                      int32_t current_time, const char* found_command,
                      uint8_t score, bool is_new_command) {
  if (is_new_command) {
    TF_LITE_REPORT_ERROR(error_reporter, "Heard %s (%d) @%dms", found_command,
                         score, current_time);
#ifdef ON_OFF_MODEL
    if (found_command[0] == 'o' && found_command[1] == 'n') {
		printf("ON\r\n");
    } else if (found_command[0] == 'o' && found_command[1] == 'f') {
		printf("OFF\r\n");
    } else if (*found_command == 'u') {
		printf("Unknown\r\n");
    } else {
		printf("Sssh, silence\r\n");
    }
#elif YES_NO_MODEL
	if (*found_command == 'y') {
	  printf("Yes\r\n");
	} else if (*found_command == 'n') {
	  printf("No\r\n");
	} else if (*found_command == 'u') {
	  printf("Unknown\r\n");
	} else {
	  printf("Sssh, silence\r\n");
	}
#endif
  }
}
