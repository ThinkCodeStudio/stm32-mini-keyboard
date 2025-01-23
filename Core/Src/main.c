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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_hid.h>
#include "usb_device.h"
#include "keyboard_define.h"
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
#define KEYBOARD_ROW 6
#define KEYBOARD_COLUMN 14
const int key_reg_max = 6;
const int delay = 4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
struct gpio {
    GPIO_TypeDef *port;
    uint16_t pin;
};

struct keyboard_msg_mag {
    struct keyboard_msg msg;
    uint8_t key_down_count;
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct keyboard_key keys_obj[KEYBOARD_ROW][KEYBOARD_COLUMN] = {
    {
        KEY_INIT(KEY_ESC),
        KEY_INIT(KEY_1),
        KEY_INIT(KEY_2),
        KEY_INIT(KEY_3),
        KEY_INIT(KEY_4),
        KEY_INIT(KEY_5),
        KEY_INIT(KEY_6),
        KEY_INIT(KEY_7),
        KEY_INIT(KEY_8),
        KEY_INIT(KEY_9),
        KEY_INIT(KEY_0),
        KEY_INIT(KEY_S_SUB),
        KEY_INIT(KEY_S_EQU),
        KEY_INIT(KEY_DELETE),
    },
    {
        KEY_INIT(KEY_TAB),
        KEY_INIT(KEY_Q),
        KEY_INIT(KEY_W),
        KEY_INIT(KEY_E),
        KEY_INIT(KEY_R),
        KEY_INIT(KEY_T),
        KEY_INIT(KEY_Y),
        KEY_INIT(KEY_U),
        KEY_INIT(KEY_I),
        KEY_INIT(KEY_O),
        KEY_INIT(KEY_P),
        KEY_INIT(KEY_S_L_SQU),
        KEY_INIT(KEY_S_R_SQU),
        KEY_INIT(KEY_DELETE_F),
    },
    {
        KEY_INIT(KEY_S_TIL),
        KEY_INIT(KEY_A),
        KEY_INIT(KEY_S),
        KEY_INIT(KEY_D),
        KEY_INIT(KEY_F),
        KEY_INIT(KEY_G),
        KEY_INIT(KEY_H),
        KEY_INIT(KEY_J),
        KEY_INIT(KEY_K),
        KEY_INIT(KEY_L),
        KEY_INIT(KEY_S_SEM),
        KEY_INIT(KEY_S_QUO),
        KEY_INIT(KEY_S_R_SLA),
        KEY_INIT(KEY_ENTER),
    },
    {
        KEY_INIT(KEY_CAPS),
        KEY_INIT(KEY_Z),
        KEY_INIT(KEY_X),
        KEY_INIT(KEY_C),
        KEY_INIT(KEY_V),
        KEY_INIT(KEY_B),
        KEY_INIT(KEY_N),
        KEY_INIT(KEY_M),
        KEY_INIT(KEY_S_COM),
        KEY_INIT(KEY_S_FUL),
        KEY_INIT(KEY_S_L_SLA),
        KEY_INIT(KEY_PAGE_UP),
        KEY_INIT(KEY_UP),
        KEY_INIT(KEY_PAGE_DOWN),
    },
    {
        KEY_INIT(KEY_FN),
        KEY_INIT(KEY_L_WIN),
        KEY_INIT(KEY_L_ALT),
        KEY_INIT(KEY_L_CTRL),
        KEY_INIT(KEY_L_SHIFT),
        KEY_INIT(KEY_HOME),
        KEY_INIT(KEY_SPACE),
        KEY_INIT(KEY_E),
        KEY_INIT(KEY_END),
        KEY_INIT(KEY_R_SHIFT),
        KEY_INIT(KEY_R_CTRL),
        KEY_INIT(KEY_LEFT),
        KEY_INIT(KEY_DOWN),
        KEY_INIT(KEY_RIGHT),
    },
    {
        KEY_INIT(KEY_E),
        KEY_INIT(KEY_F1),
        KEY_INIT(KEY_F2),
        KEY_INIT(KEY_F3),
        KEY_INIT(KEY_F4),
        KEY_INIT(KEY_F5),
        KEY_INIT(KEY_F6),
        KEY_INIT(KEY_F7),
        KEY_INIT(KEY_F8),
        KEY_INIT(KEY_F9),
        KEY_INIT(KEY_F10),
        KEY_INIT(KEY_F11),
        KEY_INIT(KEY_F12),
        KEY_INIT(KEY_E),
    },

};

struct gpio keyboard_row[KEYBOARD_ROW] = {
    {R0_GPIO_Port, R0_Pin},
    {R1_GPIO_Port, R1_Pin},
    {R2_GPIO_Port, R2_Pin},
    {R3_GPIO_Port, R3_Pin},
    {R4_GPIO_Port, R4_Pin},
    {R5_GPIO_Port, R5_Pin},
};

struct gpio keyboard_column[KEYBOARD_COLUMN] = {
    {C0_GPIO_Port, C0_Pin},
    {C1_GPIO_Port, C1_Pin},
    {C2_GPIO_Port, C2_Pin},
    {C3_GPIO_Port, C3_Pin},
    {C4_GPIO_Port, C4_Pin},
    {C5_GPIO_Port, C5_Pin},
    {C6_GPIO_Port, C6_Pin},
    {C7_GPIO_Port, C7_Pin},
    {C8_GPIO_Port, C8_Pin},
    {C9_GPIO_Port, C9_Pin},
    {CA_GPIO_Port, CA_Pin},
    {CB_GPIO_Port, CB_Pin},
    {CC_GPIO_Port, CC_Pin},
    {CD_GPIO_Port, CD_Pin},
};

struct keyboard_msg_mag mag;

void Key_Begin() {
    memset(&mag, 0, sizeof(mag));
}

void Key_End() {
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &mag.msg, sizeof(mag.msg));
}

void Key_Handler(const int row, const int column, const int state) {
    struct keyboard_key *obj = &keys_obj[row][column];
    if (state == 0) {
        obj->delay = 0;
        obj->state = 0;
    } else if (obj->delay >= delay) {
        obj->state = 1;
        switch (obj->code) {
            case KEY_L_CTRL:
                mag.msg.ctrl.L_CTRL = 1;
                break;
            case KEY_L_SHIFT:
                mag.msg.ctrl.L_SHIFT = 1;
                break;
            case KEY_L_ALT:
                mag.msg.ctrl.L_ALT = 1;
                break;
            case KEY_L_WIN:
                mag.msg.ctrl.L_WIN = 1;
                break;
            case KEY_R_CTRL:
                mag.msg.ctrl.R_CTRL = 1;
                break;
            case KEY_R_SHIFT:
                mag.msg.ctrl.R_SHIFT = 1;
                break;
            case KEY_R_ALT:
                mag.msg.ctrl.R_ALT = 1;
                break;
            case KEY_R_WIN:
                mag.msg.ctrl.R_WIN = 1;
                break;
            default:
                if (mag.key_down_count < key_reg_max) {
                    mag.msg.key[mag.key_down_count] = obj->code;
                    mag.key_down_count++;
                }
                break;
        }
    } else {
        obj->delay++;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    for (;;) {
        Key_Begin();
        for (uint8_t row = 0; row < KEYBOARD_ROW; row++) {
            const struct gpio *k_row = &keyboard_row[row];
            HAL_GPIO_WritePin(k_row->port, k_row->pin, GPIO_PIN_SET);
            for (uint8_t column = 0; column < KEYBOARD_COLUMN; column++) {
                const struct gpio *k_column = &keyboard_column[column];
                Key_Handler(row, column, HAL_GPIO_ReadPin(k_column->port, k_column->pin));
            }
            HAL_GPIO_WritePin(k_row->port, k_row->pin, GPIO_PIN_RESET);
        }
        Key_End();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        HAL_Delay(1);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin|L0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L2_Pin|L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin R3_Pin
                           R4_Pin R5_Pin L0_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|R3_Pin
                          |R4_Pin|R5_Pin|L0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C0_Pin C1_Pin C9_Pin */
  GPIO_InitStruct.Pin = C0_Pin|C1_Pin|C9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C3_Pin C4_Pin C5_Pin
                           C6_Pin C7_Pin C8_Pin CA_Pin
                           CB_Pin CC_Pin CD_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C3_Pin|C4_Pin|C5_Pin
                          |C6_Pin|C7_Pin|C8_Pin|CA_Pin
                          |CB_Pin|CC_Pin|CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L2_Pin L1_Pin */
  GPIO_InitStruct.Pin = L2_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    while (1) {
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
