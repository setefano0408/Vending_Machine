/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (4 IR Sensors + Motor Logic + Debounce)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include <stdlib.h>

char *text = "CHOOSE PRODUCT";
char *text_cancel = "CANCEL PURCHASE?";
char *text_no_money = "NOT ENOUGH MONEY";
char *text_refund = "MONEY RETURNED";
char *text_insert_coin = "VUI LONG NAP TIEN";

int prices[4] = {5, 10, 15, 10};

// --- CAC BIEN TOAN CUC ---
volatile int total_money = 0;
volatile int press = 0;
volatile int change = 0;

// Bien dieu khien Motor
volatile int motor_active = 0;
volatile int selected_motor = 0;

// Cac "Co" (Flags) bao hieu tu ngat
volatile uint8_t flag_pay_pressed = 0;
volatile uint8_t flag_choose_pressed = 0;
volatile uint8_t flag_confirm_pressed = 0;

// Bien thoi gian
uint32_t last_activity_tick = 0;
uint32_t last_lcd_refresh = 0;

// Chong rung rieng biet
volatile uint32_t time_pay_press = 0;
volatile uint32_t time_choose_press = 0;
volatile uint32_t time_confirm_press = 0;

char lcd_line1[20];
char lcd_line2[20];

// --- 1. HAM XU LY NGAT (Chi bat co) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();

    // Nut NAP TIEN (PA3)
    if (GPIO_Pin == PAYMENT_INPUT_Pin) {
        if ((current_time - time_pay_press) > 250) {
            flag_pay_pressed = 1;
            time_pay_press = current_time;
        }
    }
    // Nut CHOOSE (PB0)
    else if (GPIO_Pin == CHOOSE_INPUT_Pin) {
        if ((current_time - time_choose_press) > 250) {
            flag_choose_pressed = 1;
            time_choose_press = current_time;
        }
    }
    // Nut CONFIRM (PB1) - Sieu nhay (50ms)
    else if (GPIO_Pin == CONFIRM_INPUT_Pin) {
        if ((current_time - time_confirm_press) > 50) {
            flag_confirm_pressed = 1;
            time_confirm_press = current_time;
        }
    }
}
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();

  // Bat PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  lcd_init();
  last_activity_tick = HAL_GetTick();
  last_lcd_refresh = HAL_GetTick();
  int update_lcd = 1;

  while (1)
  {
    // ====================================================
    // A. XU LY LOGIC TU CAC CO (FLAGS)
    // ====================================================

    // 1. Xu ly NAP TIEN
    if (flag_pay_pressed == 1) {
        flag_pay_pressed = 0;
        last_activity_tick = HAL_GetTick();

        if (press == 0 && motor_active == 0) {
            if (total_money < 100) {
                total_money += 5;
                update_lcd = 1;
            }
        }
    }

    // 2. Xu ly CHOOSE
    if (flag_choose_pressed == 1) {
        flag_choose_pressed = 0;
        last_activity_tick = HAL_GetTick();

        if (motor_active == 0) {
            if (total_money == 0) {
                 lcd_clear();
                 lcd_set_cursor(0,0); lcd_write_string(text_insert_coin);
                 HAL_Delay(1000);
                 update_lcd = 1;
            } else {
                press++;
                if (press > 5) press = 1;
                update_lcd = 1;
            }
        }
    }

    // 3. Xu ly CONFIRM
    if (flag_confirm_pressed == 1) {
        flag_confirm_pressed = 0;
        last_activity_tick = HAL_GetTick();

        if (press == 0 || motor_active == 1) {
             // Khong lam gi
        }
        else if (press == 5) { // Cancel
            change = total_money;
            total_money = 0;
            press = 0;
            lcd_clear();
            lcd_set_cursor(0,0); lcd_write_string(text_refund);
            char temp[10]; sprintf(temp, "%d", change);
            lcd_set_cursor(1,0); lcd_write_string(temp);
            HAL_Delay(2000);
            update_lcd = 1;
        }
        else if (press >= 1 && press <= 4) { // Mua SP
            int price = prices[press-1];
            int stock = 0; // Mac dinh la het hang

            // [CAP NHAT IR] Kiem tra hang theo tung san pham (IR: 0=Con, 1=Het)
            if (press == 1) {
                 if (HAL_GPIO_ReadPin(IR_SENSOR_1_GPIO_Port, IR_SENSOR_1_Pin) == GPIO_PIN_RESET) stock = 1;
            }
            else if (press == 2) {
                 if (HAL_GPIO_ReadPin(IR_SENSOR_2_GPIO_Port, IR_SENSOR_2_Pin) == GPIO_PIN_RESET) stock = 1;
            }
            else if (press == 3) {
                 if (HAL_GPIO_ReadPin(IR_SENSOR_3_GPIO_Port, IR_SENSOR_3_Pin) == GPIO_PIN_RESET) stock = 1;
            }
            else if (press == 4) {
                 if (HAL_GPIO_ReadPin(IR_SENSOR_4_GPIO_Port, IR_SENSOR_4_Pin) == GPIO_PIN_RESET) stock = 1;
            }

            if (stock == 0) {
                lcd_clear();
                lcd_set_cursor(0,0); lcd_write_string("HET HANG");
                HAL_Delay(1500);
                update_lcd = 1;
            } else if (total_money < price) {
                lcd_clear();
                lcd_set_cursor(0,0); lcd_write_string(text_no_money);
                HAL_Delay(1500);
                update_lcd = 1;
            } else {
                // === MUA THANH CONG ===
                change = total_money - price;
                total_money = 0;

                selected_motor = press;
                motor_active = 1;
                update_lcd = 0;
            }
        }
    }

    // ====================================================
    // B. LOGIC MOTOR (BLOCKING MODE)
    // ====================================================
    if (motor_active == 1) {
        lcd_clear();
        lcd_set_cursor(0,0); lcd_write_string("DANG TRA HANG...");

        // Bat Motor
        if (selected_motor == 1) TIM1->CCR1 = 65535;
        else if (selected_motor == 2) TIM1->CCR2 = 65535;
        else if (selected_motor == 3) TIM1->CCR3 = 65535;
        else if (selected_motor == 4) TIM1->CCR4 = 65535;

        HAL_Delay(5000); // Chay 5 giay tuyet doi

        // Tat Motor
        TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

        // Khoi phuc LCD
        lcd_init();
        lcd_clear();
        lcd_set_cursor(0,0); lcd_write_string("MUA THANH CONG");
        sprintf(lcd_line2, "Change: %d", change);
        lcd_set_cursor(1,0); lcd_write_string(lcd_line2);
        HAL_Delay(3000);

        // Reset
        press = 0;
        motor_active = 0;
        selected_motor = 0;
        change = 0;
        update_lcd = 1;

        // Reset flags de tranh xung dot sau khi delay dai
        last_activity_tick = HAL_GetTick();
        flag_pay_pressed = 0; flag_choose_pressed = 0; flag_confirm_pressed = 0;
    }

    // --- C. LOGIC TIMEOUT (10s) ---
    if (total_money > 0 && motor_active == 0 && (HAL_GetTick() - last_activity_tick > 10000)) {
        change = total_money;
        total_money = 0;
        press = 0;
        lcd_clear();
        lcd_set_cursor(0,0); lcd_write_string("TIMEOUT!");
        sprintf(lcd_line2, "Tra lai: %d", change);
        lcd_set_cursor(1,0); lcd_write_string(lcd_line2);
        HAL_Delay(3000);
        update_lcd = 1;
        last_activity_tick = HAL_GetTick();
    }

    // --- D. HIEN THI LCD DINH KY (500ms) ---
    if ((update_lcd || (HAL_GetTick() - last_lcd_refresh > 500)) && motor_active == 0) {
        lcd_clear();
        if (press == 0) {
            sprintf(lcd_line1, "Total: %d", total_money);
            lcd_set_cursor(0,0); lcd_write_string(lcd_line1);
            lcd_set_cursor(1,0); lcd_write_string(text);
        } else if (press == 5) {
            sprintf(lcd_line1, "Total: %d", total_money);
            lcd_set_cursor(0,0); lcd_write_string(lcd_line1);
            lcd_set_cursor(1,0); lcd_write_string(text_cancel);
        } else {
            sprintf(lcd_line1, "Total: %d", total_money);
            sprintf(lcd_line2, "%c: Price: %d", 'A' + (press-1), prices[press-1]);
            lcd_set_cursor(0,0); lcd_write_string(lcd_line1);
            lcd_set_cursor(1,0); lcd_write_string(lcd_line2);
        }
        update_lcd = 0;
        last_lcd_refresh = HAL_GetTick();
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PAYMENT_INPUT_Pin */
  GPIO_InitStruct.Pin = PAYMENT_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PAYMENT_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CHOOSE_INPUT_Pin CONFIRM_INPUT_Pin */
  GPIO_InitStruct.Pin = CHOOSE_INPUT_Pin|CONFIRM_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_SENSOR_1_Pin IR_SENSOR_2_Pin IR_SENSOR_3_Pin IR_SENSOR_4_Pin */
  // [CAP NHAT] Cau hinh cho ca 4 cam bien
  GPIO_InitStruct.Pin = IR_SENSOR_1_Pin|IR_SENSOR_2_Pin|IR_SENSOR_3_Pin|IR_SENSOR_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
