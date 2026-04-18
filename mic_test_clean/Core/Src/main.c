/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include <math.h>
#include <string.h>
#include "mel_filterbank.h"
#include "dct_matrix.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define AUDIO_BITS           16U
#define AUDIO_CHANNELS       1U

#define AUDIO_HALF_SAMPLES   256
#define AUDIO_BUF_SAMPLES    (2 * AUDIO_HALF_SAMPLES)

static int16_t audio_buf[AUDIO_BUF_SAMPLES];

#define AUDIO_FS_HZ            16000
#define CAPTURE_SAMPLES        16000
#define TRIGGER_THRESHOLD      2500
#define TRIGGER_BLOCK_COUNT    2
#define CLIP_THRESHOLD         30000

#define FRAME_LEN        400
#define FRAME_HOP        160
#define FRAME_COUNT      98

#define SPEC_BINS   ((FRAME_LEN / 2) + 1)

#define MEL_BANDS   128
#define MFCC_COEFFS 13

#define MODEL_INPUT_SCALE       4.5742292404174805f
#define MODEL_INPUT_ZERO_POINT  (-86)
#define MODEL_INPUT_SIZE        (FRAME_COUNT * MFCC_COEFFS)

#define PRE_ROLL_SAMPLES   3200

#define APP_MODE_AUDIO_ONLY                    0
#define APP_MODE_STATIC_AI_TEST                1
#define APP_MODE_AUDIO_CAPTURE_1S              2
#define APP_MODE_AUDIO_CAPTURE_DEBUG           3
#define APP_MODE_AUDIO_THEN_STATIC_AI          4
#define APP_MODE_AUDIO_PREPARE_MODEL_INPUT     5
#define APP_MODE_AUDIO_PREPARE_FLOAT_INPUT     6
#define APP_MODE_AUDIO_PREPARE_FRAMES          7
#define APP_MODE_AUDIO_PREPARE_HAMMING         8
#define APP_MODE_AUDIO_PREPARE_SPECTRUM1       9
#define APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL   10
#define APP_MODE_AUDIO_PREPARE_MEL_ALL        11
#define APP_MODE_AUDIO_PREPARE_MFCC_ALL       12
#define APP_MODE_AUDIO_PREPARE_Q7_INPUT       13
#define APP_MODE_AUDIO_MIC_TO_AI              14

#define APP_MODE APP_MODE_AUDIO_MIC_TO_AI
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
static void capture_reset(void);
static void capture_stats_compute(void);
static void process_audio_block_capture(const int16_t *src, uint32_t n);
static void capture_debug_extract(void);
static void model_audio_reset(void);
static void model_audio_prepare_from_capture(void);
static void model_float_reset(void);
static void model_float_prepare_from_model_audio(void);
static void frames_reset(void);
static void frames_prepare_from_float(void);
static void hamming_reset(void);
static void hamming_prepare(void);
static void spectrum_reset(void);
static void spectrum_prepare_from_frame0(void);
static void spectrum_all_reset(void);
static void spectrum_all_prepare_from_frames(void);
static void mel_all_reset(void);
static void mel_all_prepare_from_spectra(void);
static void mfcc_reset(void);
static void mfcc_prepare_from_frames(void);
static void model_q7_reset(void);
static void model_q7_prepare_from_mfcc(void);
static void pre_roll_push_block(const int16_t *src, uint32_t n);
static void pre_roll_copy_to_capture(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint32_t cb_count = 0;     // Compte les callbacks DMA (Half+Full)
static volatile uint8_t  half_ready = 0;
static volatile uint8_t  full_ready = 0;

static int16_t capture_buf[CAPTURE_SAMPLES];

volatile uint8_t capture_active = 0;
volatile uint8_t capture_done = 0;
volatile uint32_t capture_index = 0;

volatile uint8_t trigger_count = 0;

volatile int16_t dbg_cap_first = 0;
volatile int16_t dbg_cap_last = 0;
volatile int32_t dbg_cap_sum_abs = 0;
volatile int16_t dbg_cap_max_abs = 0;
volatile uint32_t dbg_cap_clip_count = 0;
volatile uint8_t dbg_cap_stats_ready = 0;

volatile int16_t dbg_s0 = 0;
volatile int16_t dbg_s1 = 0;
volatile int16_t dbg_s2 = 0;
volatile int16_t dbg_s3 = 0;
volatile int16_t dbg_s4 = 0;
volatile int16_t dbg_s5 = 0;
volatile int16_t dbg_s6 = 0;
volatile int16_t dbg_s7 = 0;

volatile int32_t dbg_blk_mean0 = 0;
volatile int32_t dbg_blk_mean1 = 0;
volatile int32_t dbg_blk_mean2 = 0;
volatile int32_t dbg_blk_mean3 = 0;

volatile uint8_t ai_request_after_capture = 0;
volatile uint8_t ai_done_after_capture = 0;

static int16_t model_audio_input[CAPTURE_SAMPLES];

volatile uint8_t model_audio_ready = 0;
volatile int16_t dbg_model_first = 0;
volatile int16_t dbg_model_last = 0;
volatile int32_t dbg_model_sum_abs = 0;
volatile int16_t dbg_model_max_abs = 0;

static float model_audio_float[CAPTURE_SAMPLES];

volatile uint8_t model_float_ready = 0;
volatile float dbg_float_first = 0.0f;
volatile float dbg_float_last = 0.0f;
volatile float dbg_float_mean = 0.0f;
volatile float dbg_float_abs_mean = 0.0f;
volatile float dbg_float_max_abs = 0.0f;

static float frames_buf[FRAME_COUNT][FRAME_LEN];

volatile uint8_t frames_ready = 0;

volatile float dbg_f0_first = 0.0f;
volatile float dbg_f0_last = 0.0f;
volatile float dbg_f1_first = 0.0f;
volatile float dbg_f1_last = 0.0f;
volatile float dbg_f97_first = 0.0f;
volatile float dbg_f97_last = 0.0f;

volatile float dbg_f0_abs_mean = 0.0f;
volatile float dbg_f1_abs_mean = 0.0f;
volatile float dbg_f97_abs_mean = 0.0f;

static float hamming_window[FRAME_LEN];
//static float frames_win_buf[FRAME_COUNT][FRAME_LEN];

volatile uint8_t hamming_ready = 0;

volatile float dbg_hamm0 = 0.0f;
volatile float dbg_hamm1 = 0.0f;
volatile float dbg_hamm2 = 0.0f;
volatile float dbg_hamm_last = 0.0f;

volatile float dbg_fw0_first = 0.0f;
volatile float dbg_fw0_mid = 0.0f;
volatile float dbg_fw0_last = 0.0f;

volatile float dbg_fw1_first = 0.0f;
volatile float dbg_fw1_mid = 0.0f;
volatile float dbg_fw1_last = 0.0f;

volatile float dbg_fw97_first = 0.0f;
volatile float dbg_fw97_mid = 0.0f;
volatile float dbg_fw97_last = 0.0f;

static float spectrum_buf[SPEC_BINS];

volatile uint8_t spectrum_ready = 0;

volatile float dbg_spec0 = 0.0f;
volatile float dbg_spec1 = 0.0f;
volatile float dbg_spec2 = 0.0f;
volatile float dbg_spec10 = 0.0f;
volatile float dbg_spec50 = 0.0f;
volatile float dbg_spec100 = 0.0f;
volatile float dbg_spec200 = 0.0f;

volatile float dbg_spec_sum = 0.0f;
volatile float dbg_spec_max = 0.0f;
volatile uint32_t dbg_spec_argmax = 0;

//static float spectrum_all_buf[FRAME_COUNT][SPEC_BINS];

volatile uint8_t spectrum_all_ready = 0;

volatile float dbg_sp0_bin0 = 0.0f;
volatile float dbg_sp0_bin1 = 0.0f;
volatile float dbg_sp0_bin2 = 0.0f;
volatile float dbg_sp0_bin10 = 0.0f;

volatile float dbg_sp1_bin0 = 0.0f;
volatile float dbg_sp1_bin1 = 0.0f;
volatile float dbg_sp1_bin2 = 0.0f;
volatile float dbg_sp1_bin10 = 0.0f;

volatile float dbg_sp97_bin0 = 0.0f;
volatile float dbg_sp97_bin1 = 0.0f;
volatile float dbg_sp97_bin2 = 0.0f;
volatile float dbg_sp97_bin10 = 0.0f;

volatile float dbg_sp0_max = 0.0f;
volatile uint32_t dbg_sp0_argmax = 0;

volatile float dbg_sp1_max = 0.0f;
volatile uint32_t dbg_sp1_argmax = 0;

volatile float dbg_sp97_max = 0.0f;
volatile uint32_t dbg_sp97_argmax = 0;

//static float mel_all_buf[FRAME_COUNT][MEL_BANDS];

volatile uint8_t mel_all_ready = 0;

volatile float dbg_mel0_0 = 0.0f;
volatile float dbg_mel0_1 = 0.0f;
volatile float dbg_mel0_2 = 0.0f;
volatile float dbg_mel0_10 = 0.0f;

volatile float dbg_mel1_0 = 0.0f;
volatile float dbg_mel1_1 = 0.0f;
volatile float dbg_mel1_2 = 0.0f;
volatile float dbg_mel1_10 = 0.0f;

volatile float dbg_mel97_0 = 0.0f;
volatile float dbg_mel97_1 = 0.0f;
volatile float dbg_mel97_2 = 0.0f;
volatile float dbg_mel97_10 = 0.0f;

volatile float dbg_mel0_max = 0.0f;
volatile uint32_t dbg_mel0_argmax = 0;

volatile float dbg_mel1_max = 0.0f;
volatile uint32_t dbg_mel1_argmax = 0;

volatile float dbg_mel97_max = 0.0f;
volatile uint32_t dbg_mel97_argmax = 0;

static float mfcc_buf[FRAME_COUNT][MFCC_COEFFS];

volatile uint8_t mfcc_ready = 0;

volatile float dbg_mfcc0_0 = 0.0f;
volatile float dbg_mfcc0_1 = 0.0f;
volatile float dbg_mfcc0_2 = 0.0f;
volatile float dbg_mfcc0_12 = 0.0f;

volatile float dbg_mfcc1_0 = 0.0f;
volatile float dbg_mfcc1_1 = 0.0f;
volatile float dbg_mfcc1_2 = 0.0f;
volatile float dbg_mfcc1_12 = 0.0f;

volatile float dbg_mfcc97_0 = 0.0f;
volatile float dbg_mfcc97_1 = 0.0f;
volatile float dbg_mfcc97_2 = 0.0f;
volatile float dbg_mfcc97_12 = 0.0f;

int8_t model_input_q7_from_mic[MODEL_INPUT_SIZE];

volatile uint8_t model_q7_ready = 0;

volatile int8_t dbg_q7_0 = 0;
volatile int8_t dbg_q7_1 = 0;
volatile int8_t dbg_q7_2 = 0;
volatile int8_t dbg_q7_10 = 0;
volatile int8_t dbg_q7_100 = 0;
volatile int8_t dbg_q7_500 = 0;
volatile int8_t dbg_q7_1000 = 0;
volatile int8_t dbg_q7_last = 0;

volatile int8_t dbg_q7_min = 0;
volatile int8_t dbg_q7_max = 0;

volatile uint8_t ai_use_mic_q7 = 0;

static int16_t pre_roll_buf[PRE_ROLL_SAMPLES];
volatile uint32_t pre_roll_write_idx = 0;

static uint16_t audio_peak_u16(const int16_t *x, uint32_t n)
{
  uint32_t i;
  uint16_t peak = 0;
  for (i = 0; i < n; i++)
  {
    int32_t v = x[i];
    if (v < 0) v = -v;
    if ((uint16_t)v > peak) peak = (uint16_t)v;
  }
  return peak;
}

static void capture_reset(void)
{
  capture_active = 0;
  capture_done = 0;
  capture_index = 0;
  trigger_count = 0;

  dbg_cap_first = 0;
  dbg_cap_last = 0;
  dbg_cap_sum_abs = 0;
  dbg_cap_max_abs = 0;
  dbg_cap_clip_count = 0;
  dbg_cap_stats_ready = 0;

  pre_roll_write_idx = 0;
  memset(pre_roll_buf, 0, sizeof(pre_roll_buf));

  memset(capture_buf, 0, sizeof(capture_buf));
}

static void capture_stats_compute(void)
{
  int32_t sum_abs = 0;
  int16_t max_abs = 0;
  uint32_t clip_count = 0;

  dbg_cap_first = capture_buf[0];
  dbg_cap_last  = capture_buf[CAPTURE_SAMPLES - 1];

  for (uint32_t i = 0; i < CAPTURE_SAMPLES; i++)
  {
    int32_t v = capture_buf[i];
    int32_t a = (v < 0) ? -v : v;

    sum_abs += a;

    if (a > max_abs)
      max_abs = (int16_t)a;

    if (a >= CLIP_THRESHOLD)
      clip_count++;
  }

  dbg_cap_sum_abs = sum_abs;
  dbg_cap_max_abs = max_abs;
  dbg_cap_clip_count = clip_count;
  dbg_cap_stats_ready = 1;
}

static void process_audio_block_capture(const int16_t *src, uint32_t n)
{

  pre_roll_push_block(src, n);
  uint16_t peak = audio_peak_u16(src, n);

  if (!capture_active && !capture_done)
  {
    if (peak > TRIGGER_THRESHOLD)
    {
      if (trigger_count < 255)
        trigger_count++;
    }
    else
    {
      trigger_count = 0;
    }

    if (trigger_count >= TRIGGER_BLOCK_COUNT)
    {
      capture_active = 1;
      capture_done = 0;
      pre_roll_copy_to_capture();
      //HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    }
  }

  if (capture_active && !capture_done)
  {
    for (uint32_t i = 0; i < n; i++)
    {
      if (capture_index < CAPTURE_SAMPLES)
      {
        capture_buf[capture_index++] = src[i];
      }
      else
      {
        capture_done = 1;
        capture_active = 0;
        break;
      }
    }

    if (capture_index >= CAPTURE_SAMPLES)
    {
      capture_done = 1;
      capture_active = 0;
    }
  }
}

static void capture_debug_extract(void)
{
  int32_t sum0 = 0;
  int32_t sum1 = 0;
  int32_t sum2 = 0;
  int32_t sum3 = 0;

  dbg_s0 = capture_buf[0];
  dbg_s1 = capture_buf[1000];
  dbg_s2 = capture_buf[2000];
  dbg_s3 = capture_buf[4000];
  dbg_s4 = capture_buf[8000];
  dbg_s5 = capture_buf[12000];
  dbg_s6 = capture_buf[14000];
  dbg_s7 = capture_buf[15999];

  for (uint32_t i = 0; i < 4000; i++)      sum0 += capture_buf[i];
  for (uint32_t i = 4000; i < 8000; i++)   sum1 += capture_buf[i];
  for (uint32_t i = 8000; i < 12000; i++)  sum2 += capture_buf[i];
  for (uint32_t i = 12000; i < 16000; i++) sum3 += capture_buf[i];

  dbg_blk_mean0 = sum0 / 4000;
  dbg_blk_mean1 = sum1 / 4000;
  dbg_blk_mean2 = sum2 / 4000;
  dbg_blk_mean3 = sum3 / 4000;
}

static void model_audio_reset(void)
{
  model_audio_ready = 0;
  dbg_model_first = 0;
  dbg_model_last = 0;
  dbg_model_sum_abs = 0;
  dbg_model_max_abs = 0;

  memset(model_audio_input, 0, sizeof(model_audio_input));
}

static void model_audio_prepare_from_capture(void)
{
  int32_t sum_abs = 0;
  int16_t max_abs = 0;

  memcpy(model_audio_input, capture_buf, sizeof(model_audio_input));

  dbg_model_first = model_audio_input[0];
  dbg_model_last  = model_audio_input[CAPTURE_SAMPLES - 1];

  for (uint32_t i = 0; i < CAPTURE_SAMPLES; i++)
  {
    int32_t v = model_audio_input[i];
    int32_t a = (v < 0) ? -v : v;

    sum_abs += a;

    if (a > max_abs)
      max_abs = (int16_t)a;
  }

  dbg_model_sum_abs = sum_abs;
  dbg_model_max_abs = max_abs;
  model_audio_ready = 1;
}

static void model_float_reset(void)
{
  model_float_ready = 0;
  dbg_float_first = 0.0f;
  dbg_float_last = 0.0f;
  dbg_float_mean = 0.0f;
  dbg_float_abs_mean = 0.0f;
  dbg_float_max_abs = 0.0f;

  memset(model_audio_float, 0, sizeof(model_audio_float));
}

static void model_float_prepare_from_model_audio(void)
{
  double sum = 0.0;
  double sum_abs = 0.0;
  float max_abs = 0.0f;

  for (uint32_t i = 0; i < CAPTURE_SAMPLES; i++)
  {
    float x = (float)model_audio_input[i];
    model_audio_float[i] = x;

    sum += x;
    sum_abs += (x < 0.0f) ? -x : x;

    float ax = (x < 0.0f) ? -x : x;
    if (ax > max_abs)
      max_abs = ax;
  }

  dbg_float_first = model_audio_float[0];
  dbg_float_last  = model_audio_float[CAPTURE_SAMPLES - 1];
  dbg_float_mean = (float)(sum / (double)CAPTURE_SAMPLES);
  dbg_float_abs_mean = (float)(sum_abs / (double)CAPTURE_SAMPLES);
  dbg_float_max_abs = max_abs;

  model_float_ready = 1;
}

static void frames_reset(void)
{
  frames_ready = 0;

  dbg_f0_first = 0.0f;
  dbg_f0_last = 0.0f;
  dbg_f1_first = 0.0f;
  dbg_f1_last = 0.0f;
  dbg_f97_first = 0.0f;
  dbg_f97_last = 0.0f;

  dbg_f0_abs_mean = 0.0f;
  dbg_f1_abs_mean = 0.0f;
  dbg_f97_abs_mean = 0.0f;

  memset(frames_buf, 0, sizeof(frames_buf));
}

static float frame_abs_mean(const float *x, uint32_t n)
{
  double s = 0.0;

  for (uint32_t i = 0; i < n; i++)
  {
    float a = (x[i] < 0.0f) ? -x[i] : x[i];
    s += a;
  }

  return (float)(s / (double)n);
}

static void frames_prepare_from_float(void)
{
  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    uint32_t start = f * FRAME_HOP;

    for (uint32_t i = 0; i < FRAME_LEN; i++)
    {
      frames_buf[f][i] = model_audio_float[start + i];
    }
  }

  dbg_f0_first = frames_buf[0][0];
  dbg_f0_last  = frames_buf[0][FRAME_LEN - 1];

  dbg_f1_first = frames_buf[1][0];
  dbg_f1_last  = frames_buf[1][FRAME_LEN - 1];

  dbg_f97_first = frames_buf[97][0];
  dbg_f97_last  = frames_buf[97][FRAME_LEN - 1];

  dbg_f0_abs_mean  = frame_abs_mean(frames_buf[0], FRAME_LEN);
  dbg_f1_abs_mean  = frame_abs_mean(frames_buf[1], FRAME_LEN);
  dbg_f97_abs_mean = frame_abs_mean(frames_buf[97], FRAME_LEN);

  frames_ready = 1;
}

static void hamming_reset(void)
{
  hamming_ready = 0;

  dbg_hamm0 = 0.0f;
  dbg_hamm1 = 0.0f;
  dbg_hamm2 = 0.0f;
  dbg_hamm_last = 0.0f;

  dbg_fw0_first = 0.0f;
  dbg_fw0_mid = 0.0f;
  dbg_fw0_last = 0.0f;

  dbg_fw1_first = 0.0f;
  dbg_fw1_mid = 0.0f;
  dbg_fw1_last = 0.0f;

  dbg_fw97_first = 0.0f;
  dbg_fw97_mid = 0.0f;
  dbg_fw97_last = 0.0f;

  memset(hamming_window, 0, sizeof(hamming_window));
  //memset(frames_win_buf, 0, sizeof(frames_win_buf));
}

static void hamming_prepare(void)
{
  for (uint32_t i = 0; i < FRAME_LEN; i++)
  {
    hamming_window[i] = 0.54f - 0.46f * cosf((2.0f * 3.14159265358979323846f * (float)i) / (float)(FRAME_LEN - 1));
  }

  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    for (uint32_t i = 0; i < FRAME_LEN; i++)
    {
      //frames_win_buf[f][i] = frames_buf[f][i] * hamming_window[i];
    	frames_buf[f][i] = frames_buf[f][i] * hamming_window[i];
    }
  }

  dbg_hamm0 = hamming_window[0];
  dbg_hamm1 = hamming_window[1];
  dbg_hamm2 = hamming_window[2];
  dbg_hamm_last = hamming_window[FRAME_LEN - 1];

  /*
  dbg_fw0_first = frames_win_buf[0][0];
  dbg_fw0_mid   = frames_win_buf[0][FRAME_LEN / 2];
  dbg_fw0_last  = frames_win_buf[0][FRAME_LEN - 1];

  dbg_fw1_first = frames_win_buf[1][0];
  dbg_fw1_mid   = frames_win_buf[1][FRAME_LEN / 2];
  dbg_fw1_last  = frames_win_buf[1][FRAME_LEN - 1];

  dbg_fw97_first = frames_win_buf[97][0];
  dbg_fw97_mid   = frames_win_buf[97][FRAME_LEN / 2];
  dbg_fw97_last  = frames_win_buf[97][FRAME_LEN - 1];
  */

  dbg_fw0_first = frames_buf[0][0];
  dbg_fw0_mid   = frames_buf[0][FRAME_LEN / 2];
  dbg_fw0_last  = frames_buf[0][FRAME_LEN - 1];

  dbg_fw1_first = frames_buf[1][0];
  dbg_fw1_mid   = frames_buf[1][FRAME_LEN / 2];
  dbg_fw1_last  = frames_buf[1][FRAME_LEN - 1];

  dbg_fw97_first = frames_buf[97][0];
  dbg_fw97_mid   = frames_buf[97][FRAME_LEN / 2];
  dbg_fw97_last  = frames_buf[97][FRAME_LEN - 1];

  hamming_ready = 1;
}

static void spectrum_reset(void)
{
  spectrum_ready = 0;

  dbg_spec0 = 0.0f;
  dbg_spec1 = 0.0f;
  dbg_spec2 = 0.0f;
  dbg_spec10 = 0.0f;
  dbg_spec50 = 0.0f;
  dbg_spec100 = 0.0f;
  dbg_spec200 = 0.0f;

  dbg_spec_sum = 0.0f;
  dbg_spec_max = 0.0f;
  dbg_spec_argmax = 0;

  memset(spectrum_buf, 0, sizeof(spectrum_buf));
}

static void spectrum_prepare_from_frame0(void)
{
  float spec_sum = 0.0f;
  float spec_max = 0.0f;
  uint32_t spec_argmax = 0;

  for (uint32_t k = 0; k < SPEC_BINS; k++)
  {
    double re = 0.0;
    double im = 0.0;

    for (uint32_t n = 0; n < FRAME_LEN; n++)
    {
      double x = (double)frames_buf[0][n];
      double ang = 2.0 * 3.14159265358979323846 * (double)k * (double)n / (double)FRAME_LEN;

      re += x * cos(ang);
      im -= x * sin(ang);
    }

    float p = (float)(re * re + im * im);
    spectrum_buf[k] = p;

    spec_sum += p;

    if ((k == 0) || (p > spec_max))
    {
      spec_max = p;
      spec_argmax = k;
    }
  }

  dbg_spec0 = spectrum_buf[0];
  dbg_spec1 = spectrum_buf[1];
  dbg_spec2 = spectrum_buf[2];
  dbg_spec10 = spectrum_buf[10];
  dbg_spec50 = spectrum_buf[50];
  dbg_spec100 = spectrum_buf[100];
  dbg_spec200 = spectrum_buf[200];

  dbg_spec_sum = spec_sum;
  dbg_spec_max = spec_max;
  dbg_spec_argmax = spec_argmax;

  spectrum_ready = 1;
}

static void spectrum_all_reset(void)
{
  spectrum_all_ready = 0;

  dbg_sp0_bin0 = 0.0f;
  dbg_sp0_bin1 = 0.0f;
  dbg_sp0_bin2 = 0.0f;
  dbg_sp0_bin10 = 0.0f;

  dbg_sp1_bin0 = 0.0f;
  dbg_sp1_bin1 = 0.0f;
  dbg_sp1_bin2 = 0.0f;
  dbg_sp1_bin10 = 0.0f;

  dbg_sp97_bin0 = 0.0f;
  dbg_sp97_bin1 = 0.0f;
  dbg_sp97_bin2 = 0.0f;
  dbg_sp97_bin10 = 0.0f;

  dbg_sp0_max = 0.0f;
  dbg_sp0_argmax = 0;
  dbg_sp1_max = 0.0f;
  dbg_sp1_argmax = 0;
  dbg_sp97_max = 0.0f;
  dbg_sp97_argmax = 0;

  //memset(spectrum_all_buf, 0, sizeof(spectrum_all_buf));
}

static void spectrum_all_prepare_from_frames(void)
{
  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    float spec_max = 0.0f;
    uint32_t spec_argmax = 0;

    for (uint32_t k = 0; k < SPEC_BINS; k++)
    {
      double re = 0.0;
      double im = 0.0;

      for (uint32_t n = 0; n < FRAME_LEN; n++)
      {
        double x = (double)frames_buf[f][n];
        double ang = 2.0 * 3.14159265358979323846 * (double)k * (double)n / (double)FRAME_LEN;

        re += x * cos(ang);
        im -= x * sin(ang);
      }

      float p = (float)(re * re + im * im);

      if ((k == 0) || (p > spec_max))
      {
        spec_max = p;
        spec_argmax = k;
      }

      if (f == 0)
      {
        if (k == 0)  dbg_sp0_bin0 = p;
        if (k == 1)  dbg_sp0_bin1 = p;
        if (k == 2)  dbg_sp0_bin2 = p;
        if (k == 10) dbg_sp0_bin10 = p;
      }
      else if (f == 1)
      {
        if (k == 0)  dbg_sp1_bin0 = p;
        if (k == 1)  dbg_sp1_bin1 = p;
        if (k == 2)  dbg_sp1_bin2 = p;
        if (k == 10) dbg_sp1_bin10 = p;
      }
      else if (f == 97)
      {
        if (k == 0)  dbg_sp97_bin0 = p;
        if (k == 1)  dbg_sp97_bin1 = p;
        if (k == 2)  dbg_sp97_bin2 = p;
        if (k == 10) dbg_sp97_bin10 = p;
      }
    }

    if (f == 0)
    {
      dbg_sp0_max = spec_max;
      dbg_sp0_argmax = spec_argmax;
    }
    else if (f == 1)
    {
      dbg_sp1_max = spec_max;
      dbg_sp1_argmax = spec_argmax;
    }
    else if (f == 97)
    {
      dbg_sp97_max = spec_max;
      dbg_sp97_argmax = spec_argmax;
    }
  }

  spectrum_all_ready = 1;
}

static void mel_all_reset(void)
{
  mel_all_ready = 0;

  dbg_mel0_0 = 0.0f;
  dbg_mel0_1 = 0.0f;
  dbg_mel0_2 = 0.0f;
  dbg_mel0_10 = 0.0f;

  dbg_mel1_0 = 0.0f;
  dbg_mel1_1 = 0.0f;
  dbg_mel1_2 = 0.0f;
  dbg_mel1_10 = 0.0f;

  dbg_mel97_0 = 0.0f;
  dbg_mel97_1 = 0.0f;
  dbg_mel97_2 = 0.0f;
  dbg_mel97_10 = 0.0f;

  dbg_mel0_max = 0.0f;
  dbg_mel0_argmax = 0;
  dbg_mel1_max = 0.0f;
  dbg_mel1_argmax = 0;
  dbg_mel97_max = 0.0f;
  dbg_mel97_argmax = 0;

  //memset(mel_all_buf, 0, sizeof(mel_all_buf));
}

static void mel_all_prepare_from_spectra(void)
{
  float spec_tmp[SPEC_BINS];

  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    float mel_max = 0.0f;
    uint32_t mel_argmax = 0;

    /* 1) Calcul du spectre de puissance UNE SEULE FOIS pour la frame f */
    for (uint32_t k = 0; k < SPEC_BINS; k++)
    {
      float re = 0.0f;
      float im = 0.0f;

      for (uint32_t n = 0; n < FRAME_LEN; n++)
      {
        float x = frames_buf[f][n];
        float ang = 2.0f * 3.14159265358979323846f * (float)k * (float)n / (float)FRAME_LEN;

        re += x * cosf(ang);
        im -= x * sinf(ang);
      }

      spec_tmp[k] = re * re + im * im;
    }

    /* 2) Application des filtres Mel sur ce spectre déjà calculé */
    for (uint32_t m = 0; m < MEL_BANDS; m++)
    {
      float sum = 0.0f;

      for (uint32_t k = 0; k < SPEC_BINS; k++)
      {
        sum += mel_filterbank[m][k] * spec_tmp[k];
      }

      if ((m == 0) || (sum > mel_max))
      {
        mel_max = sum;
        mel_argmax = m;
      }

      if (f == 0)
      {
        if (m == 0)  dbg_mel0_0 = sum;
        if (m == 1)  dbg_mel0_1 = sum;
        if (m == 2)  dbg_mel0_2 = sum;
        if (m == 10) dbg_mel0_10 = sum;
      }
      else if (f == 1)
      {
        if (m == 0)  dbg_mel1_0 = sum;
        if (m == 1)  dbg_mel1_1 = sum;
        if (m == 2)  dbg_mel1_2 = sum;
        if (m == 10) dbg_mel1_10 = sum;
      }
      else if (f == 97)
      {
        if (m == 0)  dbg_mel97_0 = sum;
        if (m == 1)  dbg_mel97_1 = sum;
        if (m == 2)  dbg_mel97_2 = sum;
        if (m == 10) dbg_mel97_10 = sum;
      }
    }

    if (f == 0)
    {
      dbg_mel0_max = mel_max;
      dbg_mel0_argmax = mel_argmax;
    }
    else if (f == 1)
    {
      dbg_mel1_max = mel_max;
      dbg_mel1_argmax = mel_argmax;
    }
    else if (f == 97)
    {
      dbg_mel97_max = mel_max;
      dbg_mel97_argmax = mel_argmax;
    }
  }

  mel_all_ready = 1;
}

static void mfcc_reset(void)
{
  mfcc_ready = 0;

  dbg_mfcc0_0 = 0.0f;
  dbg_mfcc0_1 = 0.0f;
  dbg_mfcc0_2 = 0.0f;
  dbg_mfcc0_12 = 0.0f;

  dbg_mfcc1_0 = 0.0f;
  dbg_mfcc1_1 = 0.0f;
  dbg_mfcc1_2 = 0.0f;
  dbg_mfcc1_12 = 0.0f;

  dbg_mfcc97_0 = 0.0f;
  dbg_mfcc97_1 = 0.0f;
  dbg_mfcc97_2 = 0.0f;
  dbg_mfcc97_12 = 0.0f;

  memset(mfcc_buf, 0, sizeof(mfcc_buf));
}

static void mfcc_prepare_from_frames(void)
{
  float spec_tmp[SPEC_BINS];
  float mel_tmp[MEL_BANDS];

  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    /* 1) spectre de puissance */
    for (uint32_t k = 0; k < SPEC_BINS; k++)
    {
      float re = 0.0f;
      float im = 0.0f;

      for (uint32_t n = 0; n < FRAME_LEN; n++)
      {
        float x = frames_buf[f][n];
        float ang = 2.0f * 3.14159265358979323846f * (float)k * (float)n / (float)FRAME_LEN;

        re += x * cosf(ang);
        im -= x * sinf(ang);
      }

      spec_tmp[k] = re * re + im * im;
    }

    /* 2) énergies Mel */
    for (uint32_t m = 0; m < MEL_BANDS; m++)
    {
      float sum = 0.0f;

      for (uint32_t k = 0; k < SPEC_BINS; k++)
      {
        sum += mel_filterbank[m][k] * spec_tmp[k];
      }

      /* éviter log(0) */
      if (sum < 1.0e-12f)
        sum = 1.0e-12f;

      mel_tmp[m] = logf(sum);
    }

    /* 3) DCT -> 13 MFCC */
    for (uint32_t c = 0; c < MFCC_COEFFS; c++)
    {
      float s = 0.0f;

      for (uint32_t m = 0; m < MEL_BANDS; m++)
      {
        s += dct_matrix[c][m] * mel_tmp[m];
      }

      mfcc_buf[f][c] = s;
    }

    /* debug */
    if (f == 0)
    {
      dbg_mfcc0_0  = mfcc_buf[0][0];
      dbg_mfcc0_1  = mfcc_buf[0][1];
      dbg_mfcc0_2  = mfcc_buf[0][2];
      dbg_mfcc0_12 = mfcc_buf[0][12];
    }
    else if (f == 1)
    {
      dbg_mfcc1_0  = mfcc_buf[1][0];
      dbg_mfcc1_1  = mfcc_buf[1][1];
      dbg_mfcc1_2  = mfcc_buf[1][2];
      dbg_mfcc1_12 = mfcc_buf[1][12];
    }
    else if (f == 97)
    {
      dbg_mfcc97_0  = mfcc_buf[97][0];
      dbg_mfcc97_1  = mfcc_buf[97][1];
      dbg_mfcc97_2  = mfcc_buf[97][2];
      dbg_mfcc97_12 = mfcc_buf[97][12];
    }
  }

  mfcc_ready = 1;
}

static int8_t clamp_int8(int32_t x)
{
  if (x < -128) return -128;
  if (x > 127)  return 127;
  return (int8_t)x;
}

static void model_q7_reset(void)
{
  model_q7_ready = 0;

  dbg_q7_0 = 0;
  dbg_q7_1 = 0;
  dbg_q7_2 = 0;
  dbg_q7_10 = 0;
  dbg_q7_100 = 0;
  dbg_q7_500 = 0;
  dbg_q7_1000 = 0;
  dbg_q7_last = 0;

  dbg_q7_min = 0;
  dbg_q7_max = 0;

  memset(model_input_q7_from_mic, 0, sizeof(model_input_q7_from_mic));
}

static void model_q7_prepare_from_mfcc(void)
{
  int8_t qmin = 127;
  int8_t qmax = -128;

  for (uint32_t f = 0; f < FRAME_COUNT; f++)
  {
    for (uint32_t c = 0; c < MFCC_COEFFS; c++)
    {
      uint32_t idx = f * MFCC_COEFFS + c;

      float x = mfcc_buf[f][c];
      float qf = x / MODEL_INPUT_SCALE + (float)MODEL_INPUT_ZERO_POINT;
      int32_t qi = (int32_t)lrintf(qf);
      int8_t q = clamp_int8(qi);

      model_input_q7_from_mic[idx] = q;

      if (q < qmin) qmin = q;
      if (q > qmax) qmax = q;
    }
  }

  dbg_q7_0 = model_input_q7_from_mic[0];
  dbg_q7_1 = model_input_q7_from_mic[1];
  dbg_q7_2 = model_input_q7_from_mic[2];
  dbg_q7_10 = model_input_q7_from_mic[10];
  dbg_q7_100 = model_input_q7_from_mic[100];
  dbg_q7_500 = model_input_q7_from_mic[500];
  dbg_q7_1000 = model_input_q7_from_mic[1000];
  dbg_q7_last = model_input_q7_from_mic[MODEL_INPUT_SIZE - 1];

  dbg_q7_min = qmin;
  dbg_q7_max = qmax;

  model_q7_ready = 1;
}

static void pre_roll_push_block(const int16_t *src, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++)
  {
    pre_roll_buf[pre_roll_write_idx] = src[i];
    pre_roll_write_idx++;
    if (pre_roll_write_idx >= PRE_ROLL_SAMPLES)
      pre_roll_write_idx = 0;
  }
}

static void pre_roll_copy_to_capture(void)
{
  uint32_t idx = pre_roll_write_idx;

  for (uint32_t i = 0; i < PRE_ROLL_SAMPLES; i++)
  {
    capture_buf[i] = pre_roll_buf[idx];
    idx++;
    if (idx >= PRE_ROLL_SAMPLES)
      idx = 0;
  }

  capture_index = PRE_ROLL_SAMPLES;
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  half_ready = 1;
  cb_count++;
}

void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  full_ready = 1;
  cb_count++;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  //SCB_EnableDCache();

  MX_X_CUBE_AI_Init();


	#if (APP_MODE == APP_MODE_AUDIO_ONLY) || \
		(APP_MODE == APP_MODE_AUDIO_CAPTURE_1S) || \
		(APP_MODE == APP_MODE_AUDIO_CAPTURE_DEBUG) || \
		(APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MODEL_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FLOAT_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FRAMES) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_HAMMING) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM1) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MEL_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MFCC_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_Q7_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)

    capture_reset();
    model_audio_reset();
    model_float_reset();
    frames_reset();
    hamming_reset();
    spectrum_reset();
    spectrum_all_reset();
    mel_all_reset();
    mfcc_reset();
    model_q7_reset();
    ai_request_after_capture = 0;
    ai_done_after_capture = 0;
    ai_use_mic_q7 = 0;

    if (BSP_AUDIO_IN_InitEx(INPUT_DEVICE_DIGITAL_MICROPHONE_2,
                            AUDIO_FREQUENCY_16K,
                            DEFAULT_AUDIO_IN_BIT_RESOLUTION,
                            DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
    {
      while (1)
      {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        HAL_Delay(80);
      }
    }

    if (BSP_AUDIO_IN_Record((uint16_t *)audio_buf, AUDIO_BUF_SAMPLES) != AUDIO_OK)
    {
      while (1)
      {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        HAL_Delay(50);
      }
    }

  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	#if (APP_MODE == APP_MODE_STATIC_AI_TEST)

	  MX_X_CUBE_AI_Process();

	  while (1)
	  {
		  HAL_Delay(1000);
	  }

	#else

	  if (half_ready)
	  {
		  half_ready = 0;

	#if (APP_MODE == APP_MODE_AUDIO_ONLY)
		  uint16_t peak = audio_peak_u16(&audio_buf[0], AUDIO_HALF_SAMPLES);
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
				  (peak > 2000U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	#elif (APP_MODE == APP_MODE_AUDIO_CAPTURE_1S) || \
		(APP_MODE == APP_MODE_AUDIO_CAPTURE_DEBUG) || \
		(APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MODEL_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FLOAT_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FRAMES) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_HAMMING) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM1) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MEL_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MFCC_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_Q7_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)
		  process_audio_block_capture(&audio_buf[0], AUDIO_HALF_SAMPLES);
	#endif
	  }

	  if (full_ready)
	  {
		  full_ready = 0;

	#if (APP_MODE == APP_MODE_AUDIO_ONLY)
		  uint16_t peak = audio_peak_u16(&audio_buf[AUDIO_HALF_SAMPLES], AUDIO_HALF_SAMPLES);
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
				  (peak > 2000U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	#elif (APP_MODE == APP_MODE_AUDIO_CAPTURE_1S) || \
		(APP_MODE == APP_MODE_AUDIO_CAPTURE_DEBUG) || \
		(APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MODEL_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FLOAT_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FRAMES) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_HAMMING) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM1) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MEL_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MFCC_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_Q7_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)
		  process_audio_block_capture(&audio_buf[AUDIO_HALF_SAMPLES], AUDIO_HALF_SAMPLES);
	#endif
	  }

	#if (APP_MODE == APP_MODE_AUDIO_CAPTURE_1S) || \
		(APP_MODE == APP_MODE_AUDIO_CAPTURE_DEBUG) || \
		(APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MODEL_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FLOAT_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_FRAMES) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_HAMMING) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM1) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MEL_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_MFCC_ALL) || \
		(APP_MODE == APP_MODE_AUDIO_PREPARE_Q7_INPUT) || \
		(APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)
	  if (capture_done && !dbg_cap_stats_ready)
	  {
		  capture_stats_compute();
		  capture_debug_extract();
		  model_audio_prepare_from_capture();
		  model_float_prepare_from_model_audio();
		  frames_prepare_from_float();
		  hamming_prepare();

	#if (APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM1)
		  spectrum_prepare_from_frame0();
	#elif (APP_MODE == APP_MODE_AUDIO_PREPARE_SPECTRUM_ALL)
		  spectrum_all_prepare_from_frames();
	#elif (APP_MODE == APP_MODE_AUDIO_PREPARE_MEL_ALL)
		  mel_all_prepare_from_spectra();
	#elif (APP_MODE == APP_MODE_AUDIO_PREPARE_MFCC_ALL)
		  mfcc_prepare_from_frames();
	#elif (APP_MODE == APP_MODE_AUDIO_PREPARE_Q7_INPUT)
		  mfcc_prepare_from_frames();
		  model_q7_prepare_from_mfcc();
	#elif (APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)
		  mfcc_prepare_from_frames();
		  model_q7_prepare_from_mfcc();
		  ai_use_mic_q7 = 1;
		  ai_request_after_capture = 1;
	#endif

	#if (APP_MODE != APP_MODE_AUDIO_MIC_TO_AI)
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	#endif

	#if (APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI)
		  ai_request_after_capture = 1;
	#endif
	  }
	#endif

	#if (APP_MODE == APP_MODE_AUDIO_THEN_STATIC_AI)
	  if (ai_request_after_capture && !ai_done_after_capture)
	  {
		  ai_request_after_capture = 0;
		  ai_done_after_capture = 1;
		  MX_X_CUBE_AI_Process();
	  }
	#endif

	#if (APP_MODE == APP_MODE_AUDIO_MIC_TO_AI)
	  if (ai_request_after_capture && model_q7_ready && !ai_done_after_capture)
	  {
		  ai_request_after_capture = 0;
		  ai_done_after_capture = 1;
		  MX_X_CUBE_AI_Process();
	  }
	#endif

	#endif
  }
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
