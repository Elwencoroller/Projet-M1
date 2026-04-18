
/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body
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

 /*
  * Description
  *   v1.0 - Minimum template to show how to use the Embedded Client API
  *          model. Only one input and one output is supported. All
  *          memory resources are allocated statically (AI_NETWORK_XX, defines
  *          are used).
  *          Re-target of the printf function is out-of-scope.
  *   v2.0 - add multiple IO and/or multiple heap support
  *
  *   For more information, see the embeded documentation:
  *
  *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
  *
  *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
  *   typical : C:\Users\[user_name]\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\7.1.0
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined ( __ICCARM__ )
#elif defined ( __CC_ARM ) || ( __GNUC__ )
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"
#include "network.h"
#include "network_data.h"

/* USER CODE BEGIN includes */
#include "yes_input_q7.h"
#include "no_input_q7.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END includes */

/* IO buffers ----------------------------------------------------------------*/

#if !defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_in_1[AI_NETWORK_IN_1_SIZE_BYTES];
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
data_in_1
};
#else
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
NULL
};
#endif

#if !defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_out_1[AI_NETWORK_OUT_1_SIZE_BYTES];
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
data_out_1
};
#else
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
NULL
};
#endif

/* Activations buffers -------------------------------------------------------*/

AI_ALIGNED(32)
static uint8_t pool0[AI_NETWORK_DATA_ACTIVATION_1_SIZE];

ai_handle data_activations0[] = {pool0};

/* AI objects ----------------------------------------------------------------*/

static ai_handle network = AI_HANDLE_NULL;

static ai_buffer* ai_input;
static ai_buffer* ai_output;

volatile uint8_t dbg_pred = 255;
volatile ai_i8 dbg_out0 = 0;
volatile ai_i8 dbg_out1 = 0;
volatile ai_i8 dbg_out2 = 0;
volatile uint8_t dbg_post_done = 0;

volatile uint8_t dbg_yes_pred = 255;
volatile ai_i8 dbg_yes_out0 = 0;
volatile ai_i8 dbg_yes_out1 = 0;
volatile ai_i8 dbg_yes_out2 = 0;

volatile uint8_t dbg_no_pred = 255;
volatile ai_i8 dbg_no_out0 = 0;
volatile ai_i8 dbg_no_out1 = 0;
volatile ai_i8 dbg_no_out2 = 0;

static uint8_t app_ai_done = 0;
static uint8_t test_step = 0;   /* 0 = YES, 1 = NO, 2 = terminé */

extern int8_t model_input_q7_from_mic[AI_NETWORK_IN_1_SIZE_BYTES];
extern volatile uint8_t model_q7_ready;
extern volatile uint8_t ai_use_mic_q7;

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  if (fct)
    printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
        err.type, err.code);
  else
    printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);

  do {} while (1);
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle *act_addr)
{
  ai_error err;

  /* Create and initialize an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    ai_log_err(err, "ai_network_create_and_init");
    return -1;
  }

  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

#if defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-inputs" option is used, memory buffer can be
   *  used from the activations buffer. This is not mandatory.
   */
  for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
	data_ins[idx] = ai_input[idx].data;
  }
#else
  for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
	  ai_input[idx].data = data_ins[idx];
  }
#endif

#if defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
  /*  In the case where "--allocate-outputs" option is used, memory buffer can be
   *  used from the activations buffer. This is no mandatory.
   */
  for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
	data_outs[idx] = ai_output[idx].data;
  }
#else
  for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
	ai_output[idx].data = data_outs[idx];
  }
#endif

  return 0;
}

static int ai_run(void)
{
  ai_i32 batch;

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    ai_log_err(ai_network_get_error(network),
        "ai_network_run");
    return -1;
  }

  return 0;
}

/* USER CODE BEGIN 2 */
int acquire_and_process_data(ai_i8* data[])
{
  if (app_ai_done)
    return -1;

  if (ai_use_mic_q7 && model_q7_ready)
  {
    memcpy(data[0], model_input_q7_from_mic, AI_NETWORK_IN_1_SIZE_BYTES);
    return 0;
  }

  memcpy(data[0], yes_input_q7, AI_NETWORK_IN_1_SIZE_BYTES);
  return 0;
}

int post_process(ai_i8* data[])
{
  ai_i8 *out = data[0];
  uint8_t pred = 0;
  ai_i8 max_val = out[0];

  for (uint8_t i = 1; i < AI_NETWORK_OUT_1_SIZE; i++)
  {
    if (out[i] > max_val)
    {
      max_val = out[i];
      pred = i;
    }
  }

  dbg_pred = pred;
  dbg_out0 = out[0];
  dbg_out1 = out[1];
  dbg_out2 = out[2];
  dbg_post_done = 1;

  if (ai_use_mic_q7)
  {
    /* comportement final attendu */
    if (pred == 0)
    {
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);   /* YES */
    }
    else if (pred == 1)
    {
      HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); /* NO */
    }
    else
    {
      /* UNKNOWN : ne rien changer */
    }

    app_ai_done = 1;
    return -1;
  }

  /* fallback test statique YES */
  if (pred == 0)
  {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  }
  else
  {
    while (1)
    {
      HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
      HAL_Delay(120);
    }
  }

  app_ai_done = 1;
  return -1;
}
/* USER CODE END 2 */

/* Entry points --------------------------------------------------------------*/

void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  printf("\r\nTEMPLATE - initialization\r\n");

  ai_boostrap(data_activations0);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
	  /* USER CODE BEGIN 6 */
	  int res = 0;

	  if (!network || app_ai_done)
	    return;

	  res = acquire_and_process_data(data_ins);
	  if (res == 0)
	    res = ai_run();
	  if (res == 0)
	    res = post_process(data_outs);

	  if ((res != 0) && (!app_ai_done))
	  {
	    ai_error err = {AI_ERROR_INVALID_STATE, AI_ERROR_CODE_NETWORK};
	    ai_log_err(err, "Process has FAILED");
	  }
	  /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
