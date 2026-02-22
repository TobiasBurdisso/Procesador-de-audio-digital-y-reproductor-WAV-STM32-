/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "ssd1306.h"
#include "fft.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_WAV_FILES 4
#define MAX_NAME_LEN 48
#define CHUNK_SIZE 256
#define PWM_BUF_SIZE 512
#define WAV_HEADER_SIZE 702
#define BTN_OK GPIO_PIN_15
#define FFT_PTS 64
#define N_COLUMNAS 32
#define MAX_BAR_HEIGHT 50



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for TASK_UI */
osThreadId_t TASK_UIHandle;
const osThreadAttr_t TASK_UI_attributes = {
  .name = "TASK_UI",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for TASK_LecturaSD */
osThreadId_t TASK_LecturaSDHandle;
const osThreadAttr_t TASK_LecturaSD_attributes = {
  .name = "TASK_LecturaSD",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for TASK_EcuFilter */
osThreadId_t TASK_EcuFilterHandle;
const osThreadAttr_t TASK_EcuFilter_attributes = {
  .name = "TASK_EcuFilter",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for TASK_FFT */
osThreadId_t TASK_FFTHandle;
const osThreadAttr_t TASK_FFT_attributes = {
  .name = "TASK_FFT",
  .stack_size = 450 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};
/* Definitions for Queue_SongIndex */
osMessageQueueId_t Queue_SongIndexHandle;
const osMessageQueueAttr_t Queue_SongIndex_attributes = {
  .name = "Queue_SongIndex"
};
/* Definitions for Queue_EcuFilter */
osMessageQueueId_t Queue_EcuFilterHandle;
const osMessageQueueAttr_t Queue_EcuFilter_attributes = {
  .name = "Queue_EcuFilter"
};
/* Definitions for Queue_FFT */
osMessageQueueId_t Queue_FFTHandle;
const osMessageQueueAttr_t Queue_FFT_attributes = {
  .name = "Queue_FFT"
};
/* Definitions for mutex1 */
osMutexId_t mutex1Handle;
const osMutexAttr_t mutex1_attributes = {
  .name = "mutex1"
};
/* Definitions for sem1 */
osSemaphoreId_t sem1Handle;
const osSemaphoreAttr_t sem1_attributes = {
  .name = "sem1"
};
/* Definitions for sem2 */
osSemaphoreId_t sem2Handle;
const osSemaphoreAttr_t sem2_attributes = {
  .name = "sem2"
};
/* USER CODE BEGIN PV */

typedef enum{
	UI_MODE,
	FFT_MODE
} SystemMode_t;

typedef struct{
	float b0, b1, b2, a0, a1;
	float x1, x2, y1, y2;
}Biquad_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void xTASK_UI(void *argument);
void xTASK_LecturaSD(void *argument);
void xTASK_EcuFilter(void *argument);
void xTASK_FFT(void *argument);

/* USER CODE BEGIN PFP */
void Init_Display(void);
void ScanWavFiles(void);
void OLED_DisplayString(char *str, uint8_t x, uint8_t y);
void iniciar_reproduccion_pwm(void);
void detener_reproduccion_pwm(void);
float filtro_iir(float x, Biquad_t *f);
float fast_sqrtf(float number);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char wavFiles[MAX_WAV_FILES][MAX_NAME_LEN];  //Matriz para guardar el nombre de las canciones
uint8_t wavCount = 0;
uint8_t flag = 0;
uint8_t currentIndex = 0;
uint16_t buffer_pwm[PWM_BUF_SIZE];
SystemMode_t systemMode = UI_MODE;

Biquad_t hp200 = { .b0= 0.89485, .b1= -1.7897155, .b2=0.89485,
                 .a0= -1.77863, .a1=0.80080,
                 .x1=0,.x2=0,.y1=0,.y2=0 };

Biquad_t lp3600 = { .b0= 0.800591, .b1= 1.601182, .b2=0.800591,
                  .a0= 1.561015, .a1= 0.641348,
                  .x1=0,.x2=0,.y1=0,.y2=0 };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  Init_Display(); //Inicializo display
  ScanWavFiles(); //Leo los nombres de los archivos WAV

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutex1 */
  mutex1Handle = osMutexNew(&mutex1_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem1 */
  sem1Handle = osSemaphoreNew(1, 0, &sem1_attributes);

  /* creation of sem2 */
  sem2Handle = osSemaphoreNew(1, 1, &sem2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_SongIndex */
  Queue_SongIndexHandle = osMessageQueueNew (1, sizeof(uint8_t), &Queue_SongIndex_attributes);

  /* creation of Queue_EcuFilter */
  Queue_EcuFilterHandle = osMessageQueueNew (1, 256, &Queue_EcuFilter_attributes);

  /* creation of Queue_FFT */
  Queue_FFTHandle = osMessageQueueNew (1, 256, &Queue_FFT_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TASK_UI */
  TASK_UIHandle = osThreadNew(xTASK_UI, NULL, &TASK_UI_attributes);

  /* creation of TASK_LecturaSD */
  TASK_LecturaSDHandle = osThreadNew(xTASK_LecturaSD, NULL, &TASK_LecturaSD_attributes);

  /* creation of TASK_EcuFilter */
  TASK_EcuFilterHandle = osThreadNew(xTASK_EcuFilter, NULL, &TASK_EcuFilter_attributes);

  /* creation of TASK_FFT */
  TASK_FFTHandle = osThreadNew(xTASK_FFT, NULL, &TASK_FFT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  vTaskSetApplicationTaskTag( TASK_LecturaSDHandle, (void*) TAG_TASK_LECTURASD);
  vTaskSetApplicationTaskTag( TASK_EcuFilterHandle, (void*) TAG_TASK_ECUFILTER);
  vTaskSetApplicationTaskTag( TASK_FFTHandle, (void*) TAG_TASK_FFT);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 640-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* Función para inicializar pantalla OLED */
void Init_Display(void) {

	if(SSD1306_Init()){
		SSD1306_Clear();
		SSD1306_GotoXY(7, 32);
		SSD1306_Puts("REPRODUCTOR WAV", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
}

void ScanWavFiles(void)
{
    DIR dir;
    FILINFO fno;
    FATFS fs;    // objeto del sistema de archivos
    FRESULT res;

    res = f_mount(&fs, "0:/", 1);
    if (res != FR_OK) {
    	OLED_DisplayString("ERROR: Montar SD", 7, 64);
    	return;

    }

    res = f_opendir(&dir, "0:/");
    if (res != FR_OK){
    	OLED_DisplayString("ERROR: Archivos NULL", 7, 64);
    	return;
    }

    while (wavCount < MAX_WAV_FILES) {
        res = f_readdir(&dir, &fno);	//Leer directorio
        if (res != FR_OK || (fno.fname[0] == 0)){
        	break;  // fin o error
        }

        if(!(fno.fattrib & AM_DIR)){  //Chequear si es un directoria o un archivo
        	char *ext = strrchr(fno.fname, '.');
            if(ext && strcasecmp(ext, ".wav") == 0){
            	strncpy(wavFiles[wavCount], fno.fname, MAX_NAME_LEN-1);
                wavFiles[wavCount][MAX_NAME_LEN-1] = '\0';
                wavCount++;
            }
        }
    }

    f_closedir(&dir);


}


void OLED_DisplayString(char *str, uint8_t x, uint8_t y)
{

    osMutexAcquire(mutex1Handle, portMAX_DELAY);

    SSD1306_Clear();
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();

    osMutexRelease(mutex1Handle);
}

float filtro_iir(float x, Biquad_t *f){

	float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 - f->a0*f->y1 - f->a1*f->y2;

	f->x2 = f->x1;  f->x1 = x;
	f->y2 = f->y1;  f->y1 = y;

	return y;

}

void iniciar_reproduccion_pwm(void)
{
    // 1. Iniciar PWM por TIM3 (canal 1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // 4. Iniciar TIM2 (8kHz) que lanza el DMA
    HAL_TIM_Base_Start_IT(&htim2);
}

void detener_reproduccion_pwm(void)
{
    // 1. Detener el timer TIM2 (que disparaba el DMA)
    HAL_TIM_Base_Stop(&htim2);

    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);



}

float fast_sqrtf(float number) {

    if (number <= 0.0){
    	return 0.0;
    }

    float guess = number * 0.5;

    // 3 iteraciones de Newton-Raphson (suficiente para visualización)
    for (uint8_t i = 0; i < 10; ++i){
    	guess = 0.5 * (guess + number / guess);
    }

    return guess;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == BTN_OK){
		osSemaphoreRelease(sem1Handle);
	}


}


void callback_in(int tag) { /* Definición en main.c*/
	switch (tag) {
		case TAG_TASK_LECTURASD: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); break;
		case TAG_TASK_ECUFILTER: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); break;
		case TAG_TASK_FFT: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); break;

	}
}

void callback_out(int tag) { /* Definición en main.c*/
	switch (tag) {
		case TAG_TASK_LECTURASD: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); break;
		case TAG_TASK_ECUFILTER: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); break;
		case TAG_TASK_FFT: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); break;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	//TaskHandle_t xIdleHandle = xTaskGetIdleTaskHandle();
	//vTaskSetApplicationTaskTag(xIdleHandle, (void*)TAG_TASK_IDLE);
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(sem1Handle, osWaitForever) == osOK){

		  osDelay(20);

		  while (HAL_GPIO_ReadPin(GPIOA, BTN_OK) == GPIO_PIN_RESET){
			  osDelay(5);
		  }

		  if(systemMode == UI_MODE){
			  systemMode = FFT_MODE;
			  iniciar_reproduccion_pwm();
			  osMessageQueuePut(Queue_SongIndexHandle, &currentIndex, 0, 0);
		  }
		  else{
			  systemMode = UI_MODE;
			  detener_reproduccion_pwm();
		  }

	  }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_xTASK_UI */
/**
* @brief Function implementing the TASK_UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xTASK_UI */
void xTASK_UI(void *argument)
{
  /* USER CODE BEGIN xTASK_UI */
	uint32_t xRaw;
	//uint8_t y_start[N_COLUMNAS];
  /* Infinite loop */
  for(;;)
  {

	  if(systemMode == UI_MODE){

		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Espera fin de conversión
		  xRaw = HAL_ADC_GetValue(&hadc1);

		  if (xRaw < 95 && currentIndex > 0) {
			  currentIndex--;
			  OLED_DisplayString(wavFiles[currentIndex], 0, 0);
		  }
		  else if (xRaw > 4000 && currentIndex < wavCount - 1) {
			  currentIndex++;
			  OLED_DisplayString(wavFiles[currentIndex], 0, 0);
		  }

	  }

	  osDelay(100);

  }



  /* USER CODE END xTASK_UI */
}

/* USER CODE BEGIN Header_xTASK_LecturaSD */
/**
* @brief Function implementing the TASK_LecturaSD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xTASK_LecturaSD */
void xTASK_LecturaSD(void *argument)
{
  /* USER CODE BEGIN xTASK_LecturaSD */
	FIL wavFile;
	uint8_t idx;
	UINT bytesRead;
	FRESULT res;
	uint8_t buffer[CHUNK_SIZE];
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(Queue_SongIndexHandle, &idx, 0, osWaitForever); //Espero seleccion desde UI

	  res = f_open(&wavFile, wavFiles[idx], FA_READ);  //Abro el WAV seleccionado
	  if (res != FR_OK) {
		  OLED_DisplayString("ERROR: ABRIR ARCHIVO", 7, 32);
		  continue;
	  }

	  f_lseek(&wavFile, WAV_HEADER_SIZE); //Salteo la cabecera (44 bytes generalmente) para comenzar a leer el archivo

	  while (systemMode == FFT_MODE) {
		  //Bucle de lectura de muestras
		  res = f_read(&wavFile, buffer, CHUNK_SIZE, &bytesRead);
		  if (res != FR_OK || bytesRead == 0){
			  systemMode = UI_MODE;
			  break;   // fin de archivo o error de lectura
		  }

		  osMessageQueuePut(Queue_EcuFilterHandle, buffer, 0, osWaitForever);  //Envio a EcuFilter
	  }

	  f_close(&wavFile);

  }
  /* USER CODE END xTASK_LecturaSD */
}

/* USER CODE BEGIN Header_xTASK_EcuFilter */
/**
* @brief Function implementing the TASK_EcuFilter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xTASK_EcuFilter */
void xTASK_EcuFilter(void *argument)
{
  /* USER CODE BEGIN xTASK_EcuFilter */
	uint8_t samples[CHUNK_SIZE];
	uint8_t fft_idx = 0;
	uint16_t duty;
	float y_lo, y_hi, xn, buffer_fft[FFT_PTS];

  /* Infinite loop */

	for(;;)
	{


		osMessageQueueGet(Queue_EcuFilterHandle, samples, 0, osWaitForever); //Recibo de LecturaSD

		osSemaphoreAcquire(sem2Handle, osWaitForever);

		for(uint16_t i=0; i < CHUNK_SIZE; i++){


			xn = ((float)samples[i] - 128.0) / 128.0 ;
			y_lo = filtro_iir(xn, &hp200);
			y_hi = filtro_iir(y_lo, &lp3600);

			// 4. Almacenar en buffer PWM
			duty = (uint16_t) ((y_hi * 0.5 + 0.5)* __HAL_TIM_GET_AUTORELOAD(&htim3));
			buffer_pwm[i] = duty;

			if((i % 4) == 0){
				buffer_fft[fft_idx++] = y_hi;
				if(fft_idx >= FFT_PTS){
					osMessageQueuePut(Queue_FFTHandle, buffer_fft, 0, 0); //No bloqueante, envio a FFT
					fft_idx = 0;
				}
			}




		}


  }
  /* USER CODE END xTASK_EcuFilter */
}

/* USER CODE BEGIN Header_xTASK_FFT */
/**
* @brief Function implementing the TASK_FFT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_xTASK_FFT */
void xTASK_FFT(void *argument)
{
  /* USER CODE BEGIN xTASK_FFT */
	uint8_t y_start[N_COLUMNAS];
	uint8_t h;
	static uint8_t  peak[N_COLUMNAS] = {0};
	float mag,buffer[FFT_PTS];
	struct cmpx Y[FFT_PTS];

  /* Infinite loop */
  for(;;)
  {


	  osMessageQueueGet(Queue_FFTHandle, buffer, 0, osWaitForever);  //Recibo de EcuFilter

	  //Preparamos array complejo y ejecutamos FFT
	  for (uint8_t i = 0; i < FFT_PTS; i++) {
		  Y[i].real = buffer[i];
		  Y[i].imag = 0;
	  }
	  FFT(Y, FFT_PTS);

	  for(uint8_t i = 0; i < N_COLUMNAS ; i++){

		  mag = fast_sqrtf((Y[i].real*Y[i].real) + (Y[i].imag*Y[i].imag));

		  h = mag * MAX_BAR_HEIGHT;

		  if(h > MAX_BAR_HEIGHT){ h = MAX_BAR_HEIGHT; }

		  if(h > peak[i]){        //Attack
			  peak[i] = h;
		  }
		  else if(h <= peak[i]){  //Si peak es menor que h hago que decaiga la columna de a 2
			  peak[i] = peak[i] - 2;
			  if(peak[i] < 0){
				  peak[i] = 0;
			  }
		  }
		  else{
			  peak[i] = 0;
		  }

		  y_start[i] = 64 - peak[i];
	  }


  }
  /* USER CODE END xTASK_FFT */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim->Instance == TIM2){
		static uint16_t idx = 0;
		uint16_t duty = buffer_pwm[idx++];

		if (idx >= CHUNK_SIZE){
			idx = 0;
			osSemaphoreRelease(sem2Handle);
		}
		// Actualiza el CCR1 para el siguiente ciclo PWM
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
