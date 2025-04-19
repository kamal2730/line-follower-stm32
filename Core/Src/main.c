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
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

// ADC buffer for sensor readings
uint32_t adc_buffer[8];
uint32_t minValues[8];
uint32_t maxValues[8];

// Line following parameters
int sensorWeight[8] = {70, 60, 50, 40, 30, 20, 10, 0};
int thresh[8] = {900,900,900,900,900,900,900,900};  // Threshold for black line detection
int position;

// PID variables
double Kp = 13.5;
double Ki = 0.01;
double Kd = 4;
double error = 0;
double P = 0, I = 0, D = 0;
double lastInput = 0;
uint32_t lastTime = 0;
int setpoint = 35;

// Motor control
double base_speed = 200;
double correction = 0;
int turn = 0;

//UART COMMUNICATION
uint8_t rx_buffer[16], main_buffer[16];

//EEPROM
#define FLASH_USER_START_ADDR  0x08020000
#define FLASH_SECTOR_TO_ERASE FLASH_SECTOR_5
#define EEPROM_MAGIC 0xDEADBEEF


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
void setMotorSpeed(uint8_t motor, int32_t speed);
int line_data(void);
void computePID(double error, int32_t input);
void checkAndHandleTurn();
void calibrate();
void setPIDParameter(char *input);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void printSensorState();
void saveToFlash();


/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Motor control function
void setMotorSpeed(uint8_t motor, int32_t speed) {
    uint16_t pwm = abs(speed);
    if (pwm > 200) pwm = 200;  // Limit max speed

    if (motor == 0) {  // Left motor
        if (speed > 0) {
            TIM1->CCR1 = pwm;
            TIM1->CCR2 = 0;
        } else {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = pwm;
        }
    }
    else if (motor == 1) {  // Right motor
        if (speed > 0) {
            TIM2->CCR1 = pwm;
            TIM3->CCR1 = 0;
        } else {
            TIM2->CCR1 = 0;
            TIM3->CCR1 = pwm;
        }
    }
}

// Line position calculation
int line_data(void) {
    int sum = 0;
    int weighted_sum = 0;
    int onLine = 0;

    for (int i = 0; i < 8; i++) {
        if (adc_buffer[i] > thresh[i]) {
            weighted_sum += sensorWeight[i];
            sum += 1;
            onLine = 1;
        }
    }

    if (!onLine) {
        return 255;  // Line lost condition
    }

    return weighted_sum / sum;
}

// PID computation
void computePID(double error, int32_t input) {
    double timeChange = (double)(HAL_GetTick() - lastTime);

    // Calculate PID terms
    P = Kp * error;
    I += Ki * error * timeChange;

    // Limit integral term
    if (I > 25) I = 25;
    if (I < -25) I = -25;

    D = Kd * (input - lastInput) / timeChange;

    // Calculate correction
    correction = P + I + D;

    // Update variables for next iteration
    lastInput = input;
    lastTime = HAL_GetTick();
    setMotorSpeed(0, base_speed - correction);
    setMotorSpeed(1, base_speed + correction);
}

void checkAndHandleTurn() {
    position = line_data();

    // Detect junction or sharp turn
    turn = (position > 50 && position <= 70) ? -1 : (position < 20) ? 1 : turn;

    // If line is lost and a turn is needed
    if (turn) {
        while (position == 255) {
            if (turn == 1) { // Turn right
                setMotorSpeed(0, 150);
                setMotorSpeed(1, -150);
            } else if (turn == -1) { // Turn left
                setMotorSpeed(0, -150);
                setMotorSpeed(1, 150);
            }

            HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 8);
            position = line_data(); // Re-check for line
        }

        HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 8);
        position = line_data(); // Final update after recovery
    }

    // Update turn direction for next time
    turn = (position > 52 && position <= 70) ? -1 : (position < 20) ? 1 : turn;

    // Skip rest of loop if line is still lost
    if (position == 255) {
        return;
    }
}

volatile uint8_t adc_ready = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc_ready = 1;
    }
}

// Non-blocking callibrate()
void callibrate() {
    static int j = 0;
    static uint8_t init_done = 0;

    if (!init_done) {
        for (int i = 0; i < 8; i++) {
            minValues[i] = adc_buffer[i];
            maxValues[i] = adc_buffer[i];
        }
        init_done = 1;
        j = 0;
        adc_ready = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8); // Start once
    }

    if (adc_ready && j < 10000) {
        adc_ready = 0; // Reset flag
        for (int i = 0; i < 8; i++) {
            if (adc_buffer[i] < minValues[i]) minValues[i] = adc_buffer[i];
            if (adc_buffer[i] > maxValues[i]) maxValues[i] = adc_buffer[i];
        }

        j++;

        // Start next conversion
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8);

        if (j >= 10000) {
            for (int i = 0; i < 8; i++) {
                thresh[i] = (minValues[i] + maxValues[i]) / 2;
            }

            char done[] = "Calibration done\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)done, strlen(done), HAL_MAX_DELAY);

            // Reset state
            init_done = 0;
            j = 0;
        }
    }
}



void setPIDParameter(char *input) {
    char *ptr = input;

    while (*ptr != '\0') {
        char type = *ptr;
        ptr++;

        char valueStr[12] = {0};
        int i = 0;

        while (*ptr != 'P' && *ptr != 'p' &&
               *ptr != 'I' && *ptr != 'i' &&
               *ptr != 'D' && *ptr != 'd' &&
               *ptr != '\0') {
            valueStr[i++] = *ptr++;
        }

        double value = atof(valueStr);

        switch (type) {
            case 'P': case 'p':
                Kp = value;
                break;
            case 'I': case 'i':
                Ki = value;
                break;
            case 'D': case 'd':
                Kd = value;
                break;
        }
    }

    char confirm[] = "PID updated\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)confirm, strlen(confirm), HAL_MAX_DELAY);
//    saveToFlash();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        memset(main_buffer, '\0', 16);
        memcpy(main_buffer, rx_buffer, Size);
        memset(rx_buffer, '\0', 16);
        main_buffer[Size] = '\0';

        int len = strlen((char*)main_buffer);
        while (len > 0 && (main_buffer[len-1] == '\r' || main_buffer[len-1] == '\n')) {
            main_buffer[--len] = '\0';
        }

        // Handle 'Q' query
        if (strcmp((char*)main_buffer, "Q") == 0) {
            char status[128];

            // Send PID values
            snprintf(status, sizeof(status),
                "Kp=%d.%02d Ki=%d.%03d Kd=%d.%02d\nThresh: ",
                (int)Kp, (int)(Kp * 100) % 100,
                (int)Ki, (int)(Ki * 1000) % 1000,
                (int)Kd, (int)(Kd * 100) % 100);
            HAL_UART_Transmit(&huart6, (uint8_t*)status, strlen(status), HAL_MAX_DELAY);

            // Send all 8 threshold values in one line
            for (int i = 0; i < 8; i++) {
                char tbuf[8];
                sprintf(tbuf, "%d ", thresh[i]);
                HAL_UART_Transmit(&huart6, (uint8_t*)tbuf, strlen(tbuf), HAL_MAX_DELAY);
            }

            HAL_UART_Transmit(&huart6, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
        }

        // Handle 'C' or 'c' for calibration
        else if ((main_buffer[0] == 'C' || main_buffer[0] == 'c') && len ==1) {
            callibrate();
            char msg[] = "Calibration done\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        // Fallback: PID parameter input
        else {
            setPIDParameter((char*) main_buffer);
        }

        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, 16);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}

void printSensorState(void) {
    // Print sensor readings visually
    for (int i = 0; i < 8; i++) {
        if (adc_buffer[i] > thresh[i])
            HAL_UART_Transmit(&huart6, (uint8_t*)" | ", 3, HAL_MAX_DELAY);
        else
            HAL_UART_Transmit(&huart6, (uint8_t*)" 0 ", 3, HAL_MAX_DELAY);
    }

    // Print current line position
    char msg[32];
    sprintf(msg, " -> Pos: %d\n", position);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void saveToFlash() {

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;
    uint32_t address = FLASH_USER_START_ADDR;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_TO_ERASE;
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    // Write magic number
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, EEPROM_MAGIC);
    address += 4;

    // Write PID
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *(double*)&Kp); address += 8;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *(double*)&Ki); address += 8;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *(double*)&Kd); address += 8;

    // Write threshold array
    for (int i = 0; i < 8; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint32_t)thresh[i]);
        address += 4;
    }

    HAL_FLASH_Lock();
}

void loadFromFlash() {
    uint32_t address = FLASH_USER_START_ADDR;

    uint32_t magic = *(uint32_t*)address;
    address += 4;

    if (magic != EEPROM_MAGIC) {
        // Flash is empty or invalid — don't load
        return;
    }

    // Load PID values
    Kp = *(double*)address; address += 8;
    Ki = *(double*)address; address += 8;
    Kd = *(double*)address; address += 8;

    // Load threshold array
    for (int i = 0; i < 8; i++) {
        thresh[i] = *(uint32_t*)address;
        address += 4;
    }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();


  /* USER CODE BEGIN 2 */
  // Start ADC with DMA
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8);

  // Initialize PWM outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Initialize timing variable
  lastTime = HAL_GetTick();

  //UART
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

  //EEPROM
//  loadFromFlash();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // Start new ADC conversion
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 8);


      // Get line position
      int position = line_data();

      turn = (position > 50 && position <= 70) ? -1 : (position < 20) ? 1 : turn;

      // If line is lost and a turn is needed
      if (turn) {
          while (position == 255) {
              if (turn == 1) { // Turn right
                  setMotorSpeed(0, 150);
                  setMotorSpeed(1, -150);
              } else if (turn == -1) { // Turn left
                  setMotorSpeed(0, -150);
                  setMotorSpeed(1, 150);
              }

              HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 8);
              position = line_data(); // Re-check for line
          }

          HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, 8);
          position = line_data(); // Final update after recovery
      }

      error=setpoint-position;

      computePID(error, position);

      // Handle line following

//      printSensorState();

//      checkAndHandleTurn();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

