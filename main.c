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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h" // ?? fabs() ??
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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BASE_SPEED 15  // ????PWM
#define TURN_SPEED 12  // ????PWM
#define MPU_ADDR 0xD0  // MPU6050 I2C??

// ==================== ????????? ====================
float Gyro_OffsetZ = 0;
float Yaw_Angle = 0.0f;
float target_yaw = 0.0f;
float last_yaw_error = 0.0f;
uint32_t last_tick = 0;

uint8_t line_sensor_count = 0;
uint8_t global_task_id = 1; 

uint32_t beep_start_tick = 0;
uint8_t beep_active = 0;

// ==================== ?????? ====================

void Trigger_Beep(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    beep_start_tick = HAL_GetTick();
    beep_active = 1;
}

void Handle_Beep(void) {
    if (beep_active && (HAL_GetTick() - beep_start_tick >= 1000)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        beep_active = 0;
    }
}

void Set_Motor(int left_pwm, int right_pwm) {
    if(left_pwm > 100) left_pwm = 100; if(left_pwm < -100) left_pwm = -100;
    if(right_pwm > 100) right_pwm = 100; if(right_pwm < -100) right_pwm = -100;

    if(left_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        left_pwm = -left_pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, left_pwm);

    if(right_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        right_pwm = -right_pwm;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_pwm);
}

// ==================== ????????? ====================

uint8_t Get_Gray_Raw(void) {
    uint8_t gray = 0;
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)) gray |= (1<<7); 
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)) gray |= (1<<6);
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)) gray |= (1<<5);
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)) gray |= (1<<4); 
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)) gray |= (1<<3); 
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)) gray |= (1<<2);
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9))  gray |= (1<<1);
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8))  gray |= (1<<0); 
    return gray;
}

uint8_t Is_Line_Detected(void) {
    uint8_t gray = Get_Gray_Raw();
    if((gray & (1<<4)) || (gray & (1<<3))) return 1; 
    return 0;
}

// ???????:??????
float Get_Line_Error(void) {
    uint8_t gray = Get_Gray_Raw();
    float error = 0;
    line_sensor_count = 0; 
    
    if(gray & (1<<7)) { error -= 4.0f; line_sensor_count++; } 
    if(gray & (1<<6)) { error -= 3.0f; line_sensor_count++; }
    if(gray & (1<<5)) { error -= 2.0f; line_sensor_count++; }
    if(gray & (1<<4)) { error -= 1.0f; line_sensor_count++; } 
    
    if(gray & (1<<3)) { error += 1.0f; line_sensor_count++; } 
    if(gray & (1<<2)) { error += 2.0f; line_sensor_count++; }
    if(gray & (1<<1)) { error += 3.0f; line_sensor_count++; }
    if(gray & (1<<0)) { error += 4.0f; line_sensor_count++; } 
    
    if(line_sensor_count == 0) return 999.0f; 
    
    return error / line_sensor_count;
}

// ?????? (???????,??????????)
void Track_Line(float err, int base_spd) {
    float Kp_line = 2.0f; // ??P????
    float Kd_line = 0.5f; 
    static float last_track_err = 0;
    
    int adjust = (int)(err * Kp_line + (err - last_track_err) * Kd_line);
    last_track_err = err;
    
    // ????????????,????
    if(adjust > base_spd) adjust = base_spd;
    if(adjust < -base_spd) adjust = -base_spd;

    int left_pwm = base_spd + adjust;
    int right_pwm = base_spd - adjust;

    // ??????(0)????????,???????
    if (left_pwm < 0) left_pwm = 0;
    if (right_pwm < 0) right_pwm = 0;
    
    Set_Motor(left_pwm, right_pwm);
}

void MPU6050_Init_Wakeup(void) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x75, 1, &check, 1, 100);
    if (check == 0x68) {
        data = 0x00; HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x6B, 1, &data, 1, 100);
        data = 0x10; HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x1B, 1, &data, 1, 100);
        data = 0x08; HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, 0x1C, 1, &data, 1, 100);
    }
}

void MPU6050_Calibrate(void) {
    int32_t sumZ = 0;
    uint8_t buf[6];
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); 
    for(int i = 0; i < 500; i++) {
        HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x43, 1, buf, 6, 10);
        int16_t gz = (buf[4] << 8) | buf[5];
        sumZ += gz;
        HAL_Delay(2);
    }
    Gyro_OffsetZ = (float)sumZ / 500.0f;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); 
}

void Update_Mahony_And_Yaw(float dt) {
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x3B, 1, buf, 14, 10);
    int16_t gz = (buf[12] << 8) | buf[13];
    float gz_rad = (gz - Gyro_OffsetZ) * 0.0005326f;
    Yaw_Angle += gz_rad * dt * 57.29578f; 
    if(Yaw_Angle > 180.0f) Yaw_Angle -= 360.0f;
    if(Yaw_Angle < -180.0f) Yaw_Angle += 360.0f;
}

// ==================== ??? ???? ====================

typedef enum {
    STATE_WAIT_BTN = 0, STATE_CALIBRATE, STATE_FIND_LINE, STATE_WAIT_SETTLE, 
    STATE_TURN_180, STATE_BEEP_START, STATE_BLIND_STRAIGHT, STATE_FINISHED
} Task1_State;

Task1_State t1_state = STATE_WAIT_BTN;
uint32_t wait_start_tick = 0; 

void Task1_Run(void) {
    float turn_error, adjust;
    float Kp = 1.2f;

    switch(t1_state) {
        case STATE_WAIT_BTN:
            Set_Motor(0, 0);
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { 
                HAL_Delay(20); 
                if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                    t1_state = STATE_CALIBRATE;
                }
            }
            break;

        case STATE_CALIBRATE:
            MPU6050_Calibrate();
            Yaw_Angle = 0.0f; 
            t1_state = STATE_FIND_LINE;
            break;

        case STATE_FIND_LINE:
            Set_Motor(TURN_SPEED, -TURN_SPEED); 
            if(Is_Line_Detected()) { 
                Set_Motor(0, 0);                 
                wait_start_tick = HAL_GetTick(); 
                t1_state = STATE_WAIT_SETTLE;  
            }
            break;

        case STATE_WAIT_SETTLE:
            Set_Motor(0, 0); 
            if(HAL_GetTick() - wait_start_tick >= 500) {
                target_yaw = Yaw_Angle + 182.0f; 
                if(target_yaw > 180.0f) target_yaw -= 360.0f;
                t1_state = STATE_TURN_180; 
            }
            break;

        case STATE_TURN_180:
            turn_error = target_yaw - Yaw_Angle;
            if(turn_error > 180.0f) turn_error -= 360.0f;
            if(turn_error < -180.0f) turn_error += 360.0f;

            if(fabs(turn_error) < 2.0f) { 
                Set_Motor(0, 0);
                t1_state = STATE_BEEP_START;
            } else {
                int pwm = (int)(turn_error * Kp);
                if(pwm > TURN_SPEED) pwm = TURN_SPEED;
                if(pwm < -TURN_SPEED) pwm = -TURN_SPEED;
                if(pwm > 0 && pwm < 12) pwm = 12; 
                if(pwm < 0 && pwm > -12) pwm = -12;
                Set_Motor(-pwm, pwm);
            }
            break;

        case STATE_BEEP_START:
            Trigger_Beep(); 
            Set_Motor(BASE_SPEED, BASE_SPEED); 
            target_yaw = Yaw_Angle;            
            t1_state = STATE_BLIND_STRAIGHT; 
            break;

        case STATE_BLIND_STRAIGHT:
            turn_error = target_yaw - Yaw_Angle;
            if(turn_error > 180.0f) turn_error -= 360.0f;
            if(turn_error < -180.0f) turn_error += 360.0f;

            adjust = turn_error * 1.5f + (turn_error - last_yaw_error) * 0.2f; 
            last_yaw_error = turn_error;
            Set_Motor(BASE_SPEED - (int)adjust, BASE_SPEED + (int)adjust);

            if(Get_Gray_Raw() != 0) { 
                Set_Motor(0, 0);
                Trigger_Beep(); 
                t1_state = STATE_FINISHED;
            }
            break;

        case STATE_FINISHED:
            Set_Motor(0, 0);
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { 
                HAL_Delay(20); 
                if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
                    global_task_id = 2;       
                    MPU6050_Calibrate();      
                    Yaw_Angle = 0.0f;
                }
            }
            break;
    }
}

// ==================== ??? ???? ====================

typedef enum {
    T2_SPIN_FIND_A = 0, 
    T2_WAIT_SETTLE,     
    T2_TURN_YAW, 
    T2_BLIND_FORWARD, 
    T2_ALIGN_LINE,       // ??????:????????
    T2_TRACK_LINE, 
    T2_FINISHED
} Task2_State;

Task2_State t2_state = T2_SPIN_FIND_A;
uint8_t t2_step = 0;

void Task2_Run(void) {
    float turn_error, adjust;
    float Kp_turn = 1.2f; 
    float err = Get_Line_Error();
    uint8_t gray_raw = Get_Gray_Raw();

    static float last_valid_err = 0.0f;
    static uint32_t ignore_corner_tick = 0; 

    if(err != 999.0f) {
        last_valid_err = err;
    }

    switch(t2_state) {
        case T2_SPIN_FIND_A:
            Set_Motor(TURN_SPEED, -TURN_SPEED); 
            if(Is_Line_Detected()) { 
                Set_Motor(0, 0);                 
                wait_start_tick = HAL_GetTick(); 
                t2_state = T2_WAIT_SETTLE;       
            }
            break;

        case T2_WAIT_SETTLE:
            Set_Motor(0, 0); 
            if(HAL_GetTick() - wait_start_tick >= 500) {
                Trigger_Beep(); 
                target_yaw = Yaw_Angle + 182.0f; 
                if(target_yaw > 180.0f) target_yaw -= 360.0f;
                t2_state = T2_TURN_YAW;
            }
            break;

        case T2_TURN_YAW:
            turn_error = target_yaw - Yaw_Angle;
            if(turn_error > 180.0f) turn_error -= 360.0f;
            if(turn_error < -180.0f) turn_error += 360.0f;

            if(fabs(turn_error) < 2.0f) { 
                Set_Motor(0, 0);
                if (t2_step == 0) {
                    t2_step = 1;
                    target_yaw = Yaw_Angle;
                    t2_state = T2_BLIND_FORWARD; 
                } else {
                    t2_step++; 
                    ignore_corner_tick = HAL_GetTick(); 
                    t2_state = T2_TRACK_LINE;    
                }
            } else {
                int pwm = (int)(turn_error * Kp_turn);
                if(pwm > TURN_SPEED) pwm = TURN_SPEED;
                if(pwm < -TURN_SPEED) pwm = -TURN_SPEED;
                if(pwm > 0 && pwm < 12) pwm = 12;
                if(pwm < 0 && pwm > -12) pwm = -12;
                Set_Motor(-pwm, pwm);
            }
            break;

        case T2_BLIND_FORWARD:
            turn_error = target_yaw - Yaw_Angle;
            if(turn_error > 180.0f) turn_error -= 360.0f;
            if(turn_error < -180.0f) turn_error += 360.0f;

            adjust = turn_error * 1.5f + (turn_error - last_yaw_error) * 0.2f; 
            last_yaw_error = turn_error;
            Set_Motor(BASE_SPEED - (int)adjust, BASE_SPEED + (int)adjust);

            if(Get_Gray_Raw() != 0) { 
                Set_Motor(0, 0); // ?????
                Trigger_Beep();  
                t2_state = T2_ALIGN_LINE; // ???,????????
            }
            break;

        case T2_ALIGN_LINE:
            // ????:????????????,??????????????
            if (err != 999.0f) {
                // ???P???????
                int align_adjust = (int)(err * 3.5f); 
                int l_pwm = 12 + align_adjust;
                int r_pwm = 12 - align_adjust;
                if(l_pwm < 0) l_pwm = 0;
                if(r_pwm < 0) r_pwm = 0;
                Set_Motor(l_pwm, r_pwm);

                // ??????:???<=1.0 (????????) 
                if (fabs(err) <= 1.0f) {
                    t2_step++; 
                    ignore_corner_tick = HAL_GetTick(); // ??????
                    t2_state = T2_TRACK_LINE; // ?????,??!
                }
            } else {
                // ????????,????????
                if (last_valid_err > 0) Set_Motor(12, 0); 
                else Set_Motor(0, 12); 
            }
            break;

        case T2_TRACK_LINE:
            if (err == 999.0f) { 
                if (t2_step == 4) { // ??G????
                    Set_Motor(0, 0);
                    Trigger_Beep(); 
                    t2_step++;
                    target_yaw = Yaw_Angle;
                    t2_state = T2_BLIND_FORWARD;
                } else if (t2_step == 8) { // ????
                    Set_Motor(0, 0);
                    Trigger_Beep(); 
                    t2_state = T2_FINISHED;
                } else {
                    if (last_valid_err > 0) Set_Motor(12, 3); 
                    else Set_Motor(3, 12); 
                }
            } 
            // ?????????,??????
            else if ((HAL_GetTick() - ignore_corner_tick > 1000) && line_sensor_count >= 3 && (gray_raw & 0x03)) { 
                if (t2_step == 2 || t2_step == 3 || t2_step == 6 || t2_step == 7) {
                    Set_Motor(0, 0);
                    target_yaw = Yaw_Angle - 90.0f; 
                    if(target_yaw < -180.0f) target_yaw += 360.0f;
                    t2_state = T2_TURN_YAW;
                } else {
                    Track_Line(err, BASE_SPEED);
                }
            } 
            else {
                Track_Line(err, BASE_SPEED);
            }
            break;

        case T2_FINISHED:
            Set_Motor(0, 0);
            break;
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // 1. ??????PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  
  // 2. ??????? (??)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  // 3. ??MPU6050
  MPU6050_Init_Wakeup();
  
  // 4. ??????????
  last_tick = HAL_GetTick();

  /* USER CODE END 2 */

 /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint32_t current_tick = HAL_GetTick();
    
    // ?? 5ms (200Hz) ????????
    if(current_tick - last_tick >= 5) {
        float dt = (current_tick - last_tick) / 1000.0f; 
        last_tick = current_tick;
        
        // 1. ??????
        Update_Mahony_And_Yaw(dt);
        
        // 2. ????????
        if (global_task_id == 1) {
            Task1_Run();
        } else if (global_task_id == 2) {
            Task2_Run();
        }
        
        // 3. ??????? (??1?????)
        Handle_Beep();
    }
    
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  * where the assert_param error has occurred.
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