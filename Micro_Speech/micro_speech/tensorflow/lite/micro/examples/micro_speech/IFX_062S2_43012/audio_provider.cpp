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

#include "tensorflow/lite/micro/examples/micro_speech/audio_provider.h"
#include "tensorflow/lite/micro/examples/micro_speech/micro_features/micro_model_settings.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdio.h"

/*******************************************************************************
 * Function Name: ARM Cortex M4 tick timer handling routines and registers
 *******************************************************************************
 *******************************************************************************/
volatile unsigned int *DWT_CYCCNT  ;
volatile unsigned int *DWT_CONTROL ;
volatile unsigned int *SCB_DEMCR   ;
void init_tick(){
    DWT_CYCCNT   = (unsigned int *)0xE0001004; /* register address */
    DWT_CONTROL  = (unsigned int *)0xE0001000; /* register address */
    SCB_DEMCR    = (unsigned int *)0xE000EDFC; /* register address */
    *SCB_DEMCR   = *SCB_DEMCR | 0x01000000;
}
void start_tick(void)
{
    *DWT_CONTROL = *DWT_CONTROL | 1 ; /* enable the counter */
}

#if 0
void stop_tick(void)
{
    *DWT_CONTROL = *DWT_CONTROL & ((unsigned int) ~1) ; /* disable the counter */
}
#endif

unsigned int read_tick(){
    return *DWT_CYCCNT; /* read the tick register */
}

namespace {

/*******************************************************************************
* Macros
********************************************************************************/
/* Trigger level configured in the PDM/PCM */
#if AUDIO_LOOPBACK
#define MIC_PCM_FIFO_TRG_LVL        64u
#else
#define MIC_PCM_FIFO_TRG_LVL        128u
#endif

/* Desired sample rate. Typical values: 8/16/22.05/32/44.1/48kHz */
#define SAMPLE_RATE_HZ              16000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock. Typical values depends on the desire sample rate:
- 8/16/48kHz    : 24.576 MHz
- 22.05/44.1kHz : 22.579 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

#define AUDIO_CAPTURE_DEBUG			1
#define AUDIO_LOOPBACK				0 /* BEFORE YOU ENABLE THIS, ROUTE retarget_io in main.cpp to different IO */

#if AUDIO_LOOPBACK
#include "mtb_ak4954a.h"

/* PWM MCLK Pin */
#define MCLK_PIN            P5_0

/* Master Clock (MCLK) Frequency for the audio codec */
#define MCLK_FREQ_HZ        4096000    /* in Hz. Ideally 2.048 MHz */

/* Duty cycle for the MCLK PWM */
#define MCLK_DUTY_CYCLE     50.0f       /* in %  */

#endif

/*******************************************************************************
* Function Prototypes
********************************************************************************/

/*******************************************************************************
* Global Variables
********************************************************************************/
/* HAL Object */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;
cyhal_clock_t system_clock;

/* HAL Config */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg =
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_RIGHT,
    .word_length     = 16,  /* bits */
    .left_gain       = 18,   /* dB */
    .right_gain      = 18,   /* dB */
};

uint32_t packet_index = 0;
bool g_is_audio_initialized = false;
constexpr int kAudioCaptureBufferSize = (kAudioSampleFrequency * 0.5)+ (MIC_PCM_FIFO_TRG_LVL - (((int)(kAudioSampleFrequency * 0.5))%MIC_PCM_FIFO_TRG_LVL));
int16_t g_audio_capture_buffer[kAudioCaptureBufferSize];
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
int32_t g_latest_audio_timestamp = 0;

#if AUDIO_LOOPBACK
cyhal_pwm_t mclk_pwm;
cyhal_i2c_t mi2c;
cyhal_i2s_t i2s;

const cyhal_i2c_cfg_t mi2c_config = {
    .is_slave        = false,
    .address         = 0,
    .frequencyhal_hz = 400000
};

const cyhal_i2s_pins_t i2s_pins = {
    .sck  = P5_1,
    .ws   = P5_2,
    .data = P5_3,
};

const cyhal_i2s_config_t i2s_config = {
    .is_tx_slave    = false,            /* TX is Master */
    .is_rx_slave    = false,            /* RX not used */
    .mclk_hz        = 0,                /* External MCLK not used */
    .channel_length = 32,               /* In bits */
    .word_length    = 16,               /* In bits */
    .sample_rate_hz = SAMPLE_RATE_HZ,   /* In Hz */
};

bool g_is_i2s_initialized = false;
uint32_t out_packet_index = 0;
int16_t transfer_data[32768];
#endif

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*  Initialize the clocks in the system.
*
*******************************************************************************/
void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_get(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_init(&pll_clock);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1])
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_get(&audio_clock, &CYHAL_CLOCK_HF[1]);
    cyhal_clock_init(&audio_clock);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);

#if 0
    /* Initialize the system clock (HFCLK0) */
    cyhal_clock_get(&system_clock, &CYHAL_CLOCK_HF[0]);
    cyhal_clock_init(&system_clock);
    cyhal_clock_set_source(&system_clock, &pll_clock);
#endif

	init_tick();
	start_tick();
}

/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
*  PDM/PCM ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;
#if AUDIO_CAPTURE_DEBUG
	static uint8_t intrCount = 0;
#endif

#if AUDIO_LOOPBACK
	uint8_t i;

	for(i=0; i<MIC_PCM_FIFO_TRG_LVL;i++)
	{
		transfer_data[out_packet_index+i*2] = g_audio_capture_buffer[packet_index+i];
		transfer_data[out_packet_index+i*2+1] = g_audio_capture_buffer[packet_index+i];
	}

	cyhal_i2s_write_async(&i2s, &transfer_data[out_packet_index], 2*MIC_PCM_FIFO_TRG_LVL);
	if(g_is_i2s_initialized == false)
	{
		cyhal_i2s_start_tx(&i2s);
		g_is_i2s_initialized = true;
	}
	out_packet_index = (out_packet_index + 2*MIC_PCM_FIFO_TRG_LVL)% 32768;
#endif

	packet_index = (packet_index + MIC_PCM_FIFO_TRG_LVL)% kAudioCaptureBufferSize;
	g_latest_audio_timestamp = g_latest_audio_timestamp + (MIC_PCM_FIFO_TRG_LVL / (kAudioSampleFrequency / 1000));

	/* Move this to thread context once we have an RTOS up and running -
	 * See https://confluence.cypress.com/display/ILA/Amazon+Voice+over+Wi-Fi+Demo */

    /* Setup to read the next frame */
    cyhal_pdm_pcm_read_async(&pdm_pcm, &g_audio_capture_buffer[packet_index], MIC_PCM_FIFO_TRG_LVL);

#if AUDIO_CAPTURE_DEBUG
	intrCount++;

	if(intrCount >= 125) /* Print every second - Interrupt triggers every 128 sample */
	{
		printf("T:%ld\r\n",read_tick()/(Cy_SysClk_ClkFastGetFrequency()/1000));
		intrCount = 0;
	}
#endif
}

TfLiteStatus InitAudioRecording(tflite::ErrorReporter* error_reporter) {
  clock_init();

#if AUDIO_LOOPBACK
	/* Initialize the Master Clock with a PWM */
	cyhal_pwm_init(&mclk_pwm, MCLK_PIN, NULL);
	cyhal_pwm_set_duty_cycle(&mclk_pwm, MCLK_DUTY_CYCLE, MCLK_FREQ_HZ);
	cyhal_pwm_start(&mclk_pwm);

	    /* Initialize the I2S */
    cyhal_i2s_init(&i2s, &i2s_pins, NULL, NC, &i2s_config, &audio_clock);

	    /* Initialize the I2C Master */
    cyhal_i2c_init(&mi2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    cyhal_i2c_configure(&mi2c, &mi2c_config);

    /* Configure the AK494A codec and enable it */
    mtb_ak4954a_init(&mi2c);
    mtb_ak4954a_activate();
    mtb_ak4954a_adjust_volume(AK4954A_HP_VOLUME_DEFAULT);
#endif

  /* Initialize the PDM/PCM block */
  cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
  cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
  cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);
  cyhal_pdm_pcm_start(&pdm_pcm);

  /* Trigger the first async read call, successive ones will be executed when we get a read complete callback */
  cyhal_pdm_pcm_read_async(&pdm_pcm, &g_audio_capture_buffer[packet_index], MIC_PCM_FIFO_TRG_LVL);

  return kTfLiteOk;
}
}  // namespace

// Main entry point for getting audio data.
TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording(error_reporter);
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }
  // This should only be called when the main thread notices that the latest
  // audio sample data timestamp has changed, so that there's new data in the
  // capture ring buffer. The ring buffer will eventually wrap around and
  // overwrite the data, but the assumption is that the main thread is checking
  // often enough and the buffer is large enough that this call will be made
  // before that happens.
  const int start_offset = start_ms * (kAudioSampleFrequency / 1000);
  const int duration_sample_count =
      duration_ms * (kAudioSampleFrequency / 1000);
  for (int i = 0; i < duration_sample_count; ++i) {
    const int capture_index = (start_offset + i) % kAudioCaptureBufferSize;
    g_audio_output_buffer[i] = g_audio_capture_buffer[capture_index];
  }

  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;
  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }
