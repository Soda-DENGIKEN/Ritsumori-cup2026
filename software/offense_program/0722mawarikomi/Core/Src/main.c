/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : メインプログラム（ラインセンサー診断用カウンター出力版）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "motor.h"
#include "sensor.h"
#include "attack.h"
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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t sw_running = 0;
static uint8_t sw_prev    = 0;

// 💡 リンカエラー対策：変数たちの「本体（実体）」をここで定義します！
volatile uint8_t  attack_goal_color = 0; // 0:青, 1:黄

// 💡 ここに追加！個数を保持するグローバル変数の実体
volatile uint8_t  line_detected_count = 0;

// IR・ラインセンサーのUARTバッファの実体
uint8_t rx_buf[256];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

uint8_t line_rx_buf[256];
volatile uint16_t line_rx_head = 0;
volatile uint16_t line_rx_tail = 0;
uint8_t line_rx_byte = 0;

// 以下は main.c で使う他のファイルの変数の参照
extern volatile uint8_t  line_calib_state;
extern volatile uint32_t line_packet_count;
extern volatile float    ball_angle;
extern volatile float    ball_strength;
extern volatile uint8_t  line_on_line;
extern volatile float    line_angle;
extern volatile uint16_t line_sensor_bits;
extern volatile uint8_t  goal_blue_detected;
extern volatile float    goal_blue_angle;
extern volatile uint8_t  goal_yellow_detected;
extern volatile float    goal_yellow_angle;
extern volatile float    yaw_offset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void BNO055_Init(void);
void Sensor_InitYawOffset(void);
void Sensor_Update(void);
float Sensor_GetOmega(float goal_angle, uint8_t goal_detected);
void Attack_Update(float omega);
void Drive_Omni(float angle, float speed, float omega);
float BNO055_GetYaw(void);
void Sensor_ResetYawOnly(void);
void Sensor_ResumeLineRx(void);
float Direct_Get_BNO055_Yaw(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    BNO055_Init();
    Sensor_InitYawOffset();

    // ジャイロ ＆ ラインセンサー同期型キャリブレーション
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
    {
        HAL_UART_Transmit(&huart4, (uint8_t*)">> STARTING MULTI-CALIBRATION <<\r\n", 35, 100);

        uint32_t last_log = 0;
        uint32_t sync_timeout = 0;

        // フェーズ1：黒面サンプリング
        HAL_UART_Transmit(&huart4, (uint8_t*)"[PHASE 1] Sampling BLACK... Keep on Black Floor.\r\n", 50, 100);
        HAL_UART_Transmit(&huart2, (uint8_t*)"CALIB AUTO BLACK\n", 17, 10);

        sync_timeout = HAL_GetTick();
        while (line_calib_state != 1)
        {
            Sensor_Update();
            if (HAL_GetTick() - sync_timeout > 1000) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"CALIB AUTO BLACK\n", 17, 10);
                Sensor_ResumeLineRx();
                sync_timeout = HAL_GetTick();
            }
            HAL_Delay(5);
        }

        last_log = 0;
        while (line_calib_state == 1 || line_calib_state == 0)
        {
            Drive_Omni(0.0f, 0.0f, 0.0f);
            Sensor_ResetYawOnly();
            Sensor_Update();

            uint32_t current_time = HAL_GetTick();
            if (current_time - last_log > 500) {
                last_log = current_time;
                char lmsg[128];
                snprintf(lmsg, sizeof(lmsg), "... Line Sensor Sampling BLACK (State: %d) ...\r\n", line_calib_state);
                HAL_UART_Transmit(&huart4, (uint8_t*)lmsg, strlen(lmsg), 100);
            }
            HAL_Delay(5);
        }

        // フェーズ2：移動猶予（ロボットを白線へ）
        HAL_UART_Transmit(&huart4, (uint8_t*)"[PHASE 2] BLACK Done! -> MOVE TO WHITE LINE NOW.\r\n", 50, 100);
        HAL_UART_Transmit(&huart4, (uint8_t*)"👉 Press Button AGAIN when robot is on White Line! 👈\r\n", 57, 100);

        HAL_Delay(300);
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
            Sensor_Update();
            HAL_Delay(10);
        }

        last_log = 0;
        uint8_t white_triggered = 0;

        while (line_calib_state == 2)
        {
            Drive_Omni(0.0f, 0.0f, 0.0f);
            Sensor_Update();

            uint32_t current_time = HAL_GetTick();

            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET && !white_triggered)
            {
                HAL_Delay(50);
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
                {
                    HAL_UART_Transmit(&huart2, (uint8_t*)"CALIB AUTO WHITE\n", 17, 10);
                    HAL_UART_Transmit(&huart4, (uint8_t*)"[MAIN] Sent CALIB AUTO WHITE! Waiting for Link...\r\n", 51, 100);
                    white_triggered = 1;

                    sync_timeout = HAL_GetTick();
                    while (line_calib_state != 3)
                    {
                        Sensor_Update();
                        if (HAL_GetTick() - sync_timeout > 1000) {
                            HAL_UART_Transmit(&huart2, (uint8_t*)"CALIB AUTO WHITE\n", 17, 10);
                            Sensor_ResumeLineRx();
                            sync_timeout = HAL_GetTick();
                        }
                        HAL_Delay(5);
                    }
                }
            }

            if (current_time - last_log > 500 && !white_triggered) {
                last_log = current_time;
                char lmsg[128];
                snprintf(lmsg, sizeof(lmsg), "... Waiting WHITE Trigger (State: %d) ...\r\n", line_calib_state);
                HAL_UART_Transmit(&huart4, (uint8_t*)lmsg, strlen(lmsg), 100);
            }
            HAL_Delay(5);
        }

        // フェーズ3：白線サンプリング
        HAL_UART_Transmit(&huart4, (uint8_t*)"[PHASE 3] Sampling WHITE... Keep Robot Still!\r\n", 48, 100);

        last_log = 0;
        while (line_calib_state == 3)
        {
            Drive_Omni(0.0f, 0.0f, 0.0f);
            Sensor_Update();

            uint32_t current_time = HAL_GetTick();
            if (current_time - last_log > 500) {
                last_log = current_time;
                char lmsg[128];
                snprintf(lmsg, sizeof(lmsg), "... Line Sensor Sampling WHITE (State: %d) ...\r\n", line_calib_state);
                HAL_UART_Transmit(&huart4, (uint8_t*)lmsg, strlen(lmsg), 100);
            }
            HAL_Delay(5);
        }

        HAL_UART_Transmit(&huart2, (uint8_t*)"CALIB SAVE\n", 11, 10);
        HAL_Delay(200);

        HAL_UART_Transmit(&huart4, (uint8_t*)">> All Calibrations Successfully Saved! <<\r\n", 44, 100);

        HAL_Delay(500);
        sw_prev = 0;
        sw_running = 0;
    }

    // メインループ突入直前にUARTをリフレッシュ
    Sensor_ResumeLineRx();

    uint32_t last_debug_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
      {
            // 1. スイッチのチャタリング防止とモード切替
            uint8_t sw_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
            if (sw_state == GPIO_PIN_SET && sw_prev == GPIO_PIN_RESET)
            {
                HAL_Delay(50); // スイッチ押下時のみの一時的なデリートなのでOK
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
                {
                    sw_running = !sw_running;

                    char sw_msg[64];
                    snprintf(sw_msg, sizeof(sw_msg), ">>> MODE CHANGED: %s <<<\r\n", sw_running ? "RUN" : "STOP");
                    HAL_UART_Transmit(&huart4, (uint8_t*)sw_msg, strlen(sw_msg), 100);
                }
            }
            sw_prev = sw_state;

            // 2. センサー値の更新（ここを最速で回す！）
            Sensor_Update();

            // 3. 姿勢制御 omega の計算
            float omega;
            if (attack_goal_color == 0) // 0 = ATTACK_BLUE
            {
                omega = Sensor_GetOmega(goal_blue_angle, goal_blue_detected);
            }
            else
            {
                omega = Sensor_GetOmega(goal_yellow_angle, goal_yellow_detected);
            }

            // 4. モーターへの出力命令
            if (sw_running == 1)
            {
                Attack_Update(omega);
            }
            else
            {
                Drive_Omni(0.0f, 0.0f, 0.0f);
            }

            // ==========================================================
            // 💡 変更点：ロボットが停止している（STOP）時だけデバッグ出力を出す！
            // ==========================================================
            if (sw_running == 0)
            {
                uint32_t current_tick = HAL_GetTick();
                if (current_tick - last_debug_tick >= 100) // 停止時は100ms周期で十分
                {
                    last_debug_tick = current_tick;
                    char tx_out[256];
                    int len = 0;

                    // 💡重い I2C 直接取得はやめ、通常更新された ball_angle や BNO055_GetYaw() を使い回す
                    float gyro_yaw = BNO055_GetYaw();

                    char line_ang_str[8];
                    if (line_on_line) {
                        snprintf(line_ang_str, sizeof(line_ang_str), "%6.1f", line_angle);
                    } else {
                        snprintf(line_ang_str, sizeof(line_ang_str), "  NONE");
                    }

                    char line_bits_str[17];
                    for (int i = 0; i < 16; i++) {
                        line_bits_str[i] = (line_sensor_bits & (1u << i)) ? '1' : '0';
                    }
                    line_bits_str[16] = '\0';

                    len += snprintf(tx_out + len, sizeof(tx_out) - len,
                                    "[STOP ] IR_Ang:%6.1f | IR_Str:%5.1f | Yaw:%6.1f | Line_Ang:%s | Line:[%s]\r\n",
                                    ball_angle, ball_strength, gyro_yaw, line_ang_str, line_bits_str);

                    HAL_UART_Transmit(&huart4, (uint8_t *)tx_out, len, 100);
                }
            }

            // 💡 変更点：HAL_Delay(5); を完全に削除！
            // これによりマイコンが何もせずに待たされる時間がゼロになり、超高速でループします。

      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 17;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 17;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 💡 main.cのI2C設定(hi2c2)を使って直接ジャイロから角度を奪い取る緊急関数
float Direct_Get_BNO055_Yaw(void) {
    uint8_t data[2];
    int16_t raw_yaw = 0;

    // I2C2ポートを直接叩いてYawレジスタ(0x1A)から2バイト読む
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, (0x28 << 1), 0x1A, I2C_MEMADD_SIZE_8BIT, data, 2, 10);
    if (status == HAL_OK) {
        raw_yaw = (int16_t)((data[1] << 8) | data[0]);
        float yaw_deg = (float)raw_yaw / 16.0f;
        if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
        return yaw_deg;
    }
    return -999.0f; // 通信エラー時は -999 を返す
}
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
#ifdef USE_FULL_ASSERT
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
