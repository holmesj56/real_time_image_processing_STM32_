/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUFFER_SIZE 8 // reduced from 12288 to 8 for debugging

uint8_t rcvd_data[BUFFER_SIZE]; // input buffer
// output buffer
// for debugging of thresholding, quantization, and gray level transformations
uint8_t trmt_data[BUFFER_SIZE] = {122, 155, 80, 254, 7, 40, 240, 50};
// for debugging of histogram equalization
//uint8_t trmt_data[BUFFER_SIZE] = {1, 1, 3, 5, 6, 7, 7, 7};

// segmentation/thresholding constants
#define THRESHOLD 80
#define THRESHOLD1 80
#define THRESHOLD2 240

// gray level transformation constants
#define A 120
#define B 50

// histogram equalization constant and variables
#define NUM_GRAY_LEVEL 8 // reduced from 256 to 8 for histogram equalization debugging

uint32_t hist_cnt[NUM_GRAY_LEVEL] = {0};  // store counts
uint8_t mapped_levels[NUM_GRAY_LEVEL];


// segmentation/thresholding functions
void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold);

void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);
void band_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold1, uint8_t threshold2);
void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold);

// gray level quantization functions
void gray_level_quantization_c(uint8_t *x, uint32_t size, uint8_t shift_factor);
void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor);

// gray level transformation functions
void gray_level_transformation1_c(uint8_t *x, uint32_t size);
void gray_level_transformation2_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_c(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

void gray_level_transformation1_hybrid(uint8_t *x, uint32_t size);
void gray_level_transformation2_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);
void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0);

// histogram equalization functions
void calculate_histogram_c(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_c(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_c(uint8_t *x, uint8_t *mapping_table, uint32_t size);

void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size);
void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels);
void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // global_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD);
  //semi_thresholding_hybrid(trmt_data, BUFFER_SIZE, THRESHOLD);
 //gray_level_quantization_hybrid(trmt_data, BUFFER_SIZE, 1);
  //calculate_histogram_hybrid(trmt_data, hist_cnt, BUFFER_SIZE);
  //map_levels_hybrid(hist_cnt, mapped_levels, BUFFER_SIZE, NUM_GRAY_LEVEL);
  //transform_image_hybrid(trmt_data,mapped_levels, BUFFER_SIZE);
  //gray_level_transformation1_hybrid(trmt_data, BUFFER_SIZE);
  gray_level_transformation3_hybrid(trmt_data, BUFFER_SIZE, A, B);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void global_thresholding_c(uint8_t *x, uint32_t size, uint8_t threshold)
{

}

__attribute__ ((naked)) void global_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile (
	// x -> r0, size -> r1, threshold -> r2
			"PUSH {r4, r5, r6, lr}\n\t" // save return address

			// loop over the array
			"MOV r3, #0\n\t" // loop counter r3
			"MOV r4, #255\n\t" // constant used
			"MOV r5, #0\n\t" // constant used"
			"loop_gth: CMP r3, r1\n\t" // terminate the loop when r3 >= r1
			"BGE exit_gth\n\t"
			"LDRB r6, [r0]\n\t" // load array element to r6
			"CMP r6, r2\n\t" // compare with threshold r2
			"BGE else_gth\n\t"
			"STRB r5, [r0], #1\n\t" // store r5 to its memory location, post increment r0 by 1
			"B endif_gth\n\t"
			"else_gth: STRB r4, [r0], #1\n\t" // store r4 to its memory location, post increment r0 by 1
			"endif_gth: ADD r3, r3, #1\n\t" // increment loop counter
			"B loop_gth\n\t"
			"exit_gth: POP {r4, r5, r6, pc}\n\t" // return from function

    );

}

__attribute__ ((naked)) void semi_thresholding_hybrid(uint8_t *x, uint32_t size, uint8_t threshold)
{
	__asm volatile (
	        "PUSH {r4, r5, r6, r7, lr}\n\t"

	        "MOV r5, #255\n\t"       // r5 = 255 (value to set if above threshold)
	        "MOV r6, #0\n\t"         // r6 = 0 (value to set if below or equal to threshold)
	        "MOV r4, #0\n\t"         // r4 = 0 (loop index)
	        "loop_gth_:\n\t"
	        "CMP r4, r1\n\t"         // Compare loop index (r4) with size (r1)
	        "BGE exit_gth_\n\t"      // If index >= size, exit loop

	        "LDRB r7, [r0, r4]\n\t"  // Load the current byte from x into r7
	        "CMP r7, r2\n\t"         // Compare the byte with the threshold (r2)
	        "BGE thre\n\t"           // If the byte is greater than or equal to the threshold, branch to 'thre'

	        "STRB r6, [r0, r4]\n\t"  // Store 0 (r6) in x[index] if below threshold
	        "B cnt_gth\n\t"          // Unconditionally branch to increment index (this line can actually be removed)

	        "thre:\n\t"
	        "STRB r5, [r0, r4]\n\t"  // Store 255 (r5) in x[index] if above or equal to threshold

	        "cnt_gth:\n\t"
	        "ADD r4, r4, #1\n\t"     // Increment the loop index (r4)
	        "B loop_gth_\n\t"        // Unconditionally branch to the beginning of the loop

	        "exit_gth_:\n\t"
	        "POP {r4, r5, r6, r7, pc}\n\t"

	    );
}
__attribute__ ((naked)) void gray_level_transformation3_hybrid(uint8_t *x, uint32_t size, uint8_t a0, uint8_t b0)
{
	__asm volatile(
	    "PUSH {r4-r8, lr}\n\t"
	    "MOV r4, #0\n"


	    "loop1:\n\t"
	    "CMP r4, r1\n"
	    "BGE exit1\n"

	    "LDRB r5, [r0, r4]\n"

	    //x<=b0
	    "CMP r5, r3\n"
	    "BLE lte_1\n"                       // x<=b0

	    "MOV r6, #255\n"                    //255 into r6
	    "SUB r6, r6, r3\n"

	    "CMP r5, r6\n"
	    "BLE bte_1\n"                       //if x> b0 && x<= 255-b0

	    //x> 255-b0
	    "B gte2_1\n"                        //x>255-b0

	    // x<=b0
	    "lte_1:\n\t"
	    "B str_1\n"

	    // x>b0  x<=255-b0
	    "bte_1:\n\t"
	    "B str_1\n"

	    // x>255-b0
	    "gte2_1:\n\t"



	    "str_1:\n\t"
	    "STRB r5, [r0, r4]\n"

	    "ADD r4, r4, #1\n"
	    "B loop1\n"


	    "exit1:\n\t"
	    "POP {r4-r8, lr}\n"
	    "BX lr\n"
	);
}

__attribute__ ((naked)) void gray_level_quantization_hybrid(uint8_t *x, uint32_t size, uint8_t shift_factor)
{
	__asm volatile (
		// x -> r0, size -> r1, threshold -> r2
			"PUSH {r4, r5, lr}\n\t"          // Save registers and LR (return address)

			        "MOV r4, #0\n\t"                 // Initialize loop counter to 0
			        "loop_glq:\n\t"
			        "CMP r4, r1\n\t"                 // Compare counter with size
			        "BGE exit_glq\n\t"               // If counter >= size, exit loop

			        "LDRB r5, [r0 ,r4]\n\t"          // Load byte from array into r5
			        "LSR r5, r5, #1\n\t"             // Logical Shift Right by 1 (divide by 2)
			        "STRB r5, [r0, r4]\n\t"          // Store the result back into the array

			        "ADD r4, r4, #1\n\t"             // Increment loop counter
			        "B loop_glq\n\t"                 // Jump back to start of loop

			        "exit_glq:\n\t"
			        "POP {r4, r5, lr}\n\t"           // Restore saved registers and LR
			        "BX lr\n\t"

	    );
}
__attribute__ ((naked)) void gray_level_transformation1_hybrid (uint8_t *x, uint32_t size)
{
	__asm volatile(
			// x -> r0, size -> r1, threshold -> r2
						"PUSH {r4, r5, lr}\n\t"
				        "MOV r4, #0\n\t"                 // loop counter  0
				        "loop_glt_l:\n\t"
				        "CMP r4, r1\n\t"                 // i<size
				        "BGE exit_glt_l\n\t"               // if counter >= size, exit
				        "LDRB r5, [r0 ,r4]\n\t"
				        "RSB r5, r5, #255 \n\t"
				        "STRB r5, [r0, r4]\n\t"          // back to array

				        "ADD r4, r4, #1\n\t"             // increment loop
				        "B loop_glt_l\n\t"                 //  start

				        "exit_glt_l:\n\t"
				        "POP {r4, r5, lr}\n\t"
				        "BX lr\n\t"

		    );
}

__attribute__ ((naked)) void calculate_histogram_hybrid(uint8_t *x, uint32_t *hist, uint32_t size)
{
    __asm volatile(
        "PUSH {r4-r7, lr}\n\t"           // save registers r4-r7 and lr

        // Initialize histogram array to 0
        "MOV r3, #0\n\t"                 // r3 for index in the histogram array
        "MOV r4, #256\n"                 // r4 is the constant 256
        "loop:\n\t"                      // Start of the loop for initialization
        "CMP r3, r4\n\t"                 // Compare the index r3 with 256
        "BGE init_done\n\t"              // If r3 >= 256, initialization is done
        "MOV r5, #0\n\t"                 // Move 0 into r5 to use for initializing the histogram
        "STR r5, [r1, r3, LSL #2]\n\t"   // Store 0 in the histogram array at index r3, offset by r3*4
        "ADD r3, r3, #1\n\t"             // Increment index r3
        "B loop\n\t"                     // Branch to the start of the loop

        "init_done:\n\t"
        // Calculate histogram from the image data
        "MOV r3, #0\n\t"                 // Reset r3 to use as index for the image data
        "hist_loop:\n\t"
        "CMP r3, r2\n\t"                 // Compare r3 with size
        "BGE hist_done\n\t"              // If r3 >= size, we're done with the histogram
        "LDRB r5, [r0, r3]\n\t"          // Load byte (pixel value) from image data into r5
        "LDR r6, [r1, r5, LSL #2]\n\t"   // Load current histogram count into r6
        "ADD r6, r6, #1\n\t"             // Increment histogram count
        "STR r6, [r1, r5, LSL #2]\n\t"   // Store back incremented count into histogram
        "ADD r3, r3, #1\n\t"             // Increment index r3
        "B hist_loop\n\t"                // Branch to the start of the loop

        "hist_done:\n\t"
        "POP {r4-r7, lr}\n\t"            // Restore registers r4-r7 and lr
        "BX lr\n\t"
        :::"r3", "r4", "r5", "r6"
    );
}

/*
"map_levels" is a function that generates a mapping table (mapped_levels) to
equalize the histogram.
*/


__attribute__ ((naked)) void map_levels_hybrid(uint32_t *hist, uint8_t *mapping_table, uint32_t size, uint16_t levels)
{
	// make sure you "MUL" first, then "UDIV", otherwise you will get zero
	// you need to accumulate the histogram counts, multiply by (levels-1), then divide by size
	__asm volatile(
	        "PUSH {r4-r11, lr}\n\t"             // save registers .

	        // r0 = hist, r1 = mapping_table, r2 = size, r3 = levels

	        "MOV r4, #0\n\t"                    //  i = 0
	        "MOV r5, #0\n\t"                    //  sum = 0
	        "map:\n\t"
	        "CMP r4, r3\n\t"                    // i<  levels
	        "BGE mapd\n\t"              // If i >= levels, done

	        //  hist[i] into r6
	        "LDR r6, [r0, r4, LSL #2]\n\t"      // r6 = hist[i]

	        //  hist counts
	        "ADD r5, r5, r6\n\t"                // r5 += hist[i]


	        "MUL r6, r5, r3\n\t"                // first multiply as instructed
	        "UDIV r6, r6, r2\n\t"               //then divide
	        "STRB r6, [r1, r4]\n\t"             // result to table

	        "ADD r4, r4, #1\n\t"                // loop count
	        "B map\n\t"

	        "mapd:\n\t"
	        "POP {r4-r11, lr}\n\t"
	        "BX lr\n\t"
	    );
}



/*
"transform_image" is a function that transforms the brightness levels of an image array
according to the mapping_table using histogram equalization
*/


__attribute__ ((naked)) void transform_image_hybrid(uint8_t *x, uint8_t *mapping_table, uint32_t size)
{
	 __asm volatile(
	        "PUSH {r4-r7, lr}\n\t"

	        "MOV r3, #0\n\t"            // i=0
	        "loop5:\n\t"
	        "CMP r3, r2\n\t"            //  counter i
	        "BGE loop_end\n\t"  // i >= size

	        "LDRB r4, [r0, r3]\n\t"
	        "LDRB r4, [r1, r4]\n\t"
	        "STRB r4, [r0, r3]\n\t"     // store value to x

	        "ADD r3, r3, #1\n\t"        //  i++
	        "B loop5\n\t"      // continue


	        "loop_end:\n\t"
	        "POP {r4-r7, lr}\n\t"
	        "BX lr\n\t");

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
