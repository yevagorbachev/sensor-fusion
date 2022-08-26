/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <malloc.h>

#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag_i.h"
#include "sensors/mag_e.h"
#include "badsched.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	ORANGE = LD3_Pin,
	GREEN = LD4_Pin,
	RED = LD5_Pin,
	BLUE = LD6_Pin
} led_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HCLK_kHz 96000
#define TIM10_PRESCALER 96
#define TIM10_DIVIDER_MS HCLK_kHz / TIM10_PRESCALER
#define TIM11_PRESCALER 9600
#define TIM11_DIVIDER_MS HCLK_kHz / TIM11_PRESCALER

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

stmdev_ctx_t accel_ctx;
stmdev_ctx_t gyro_ctx;
stmdev_ctx_t mag_i_ctx;
stmdev_ctx_t mag_e_ctx;

accel_t acceleration;
angular_rate_t angular_rate;
mag_field_t mag_i_field;
mag_field_t mag_e_field;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// wrapping HAL function for brevity
void toggle_led(led_t led)
{
	HAL_GPIO_TogglePin(GPIOD, led);
}
void set_led(led_t led)
{
	HAL_GPIO_WritePin(GPIOD, led, GPIO_PIN_SET);
}
void reset_led(led_t led)
{
	HAL_GPIO_WritePin(GPIOD, led, GPIO_PIN_RESET);
}

// overrides syscall _write so we can use SWV
int _write(int file, char* ptr, int len)
{
	for (int DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

void my_init(void)
{
	printf("Started both timers\n");

	/* Notes
	 * ODR - output data rate, usually Hz
	 * FS - full scale, set in each sensor implementation's header becuase the conversion
	 * 	we use in <quantity>_get is dependent on this
	 * OM - operating mode, for the mags
	 * PM - power mode, low/normal/high res
	 * ODR - output data rate; FS - full scale, OM - operating mode, PM - power mode
	 * Full scale is set in each sensor implementation's header file because the
	 * conversion constant depends on the full scale
	 */
	/* Gyroscope setup
	 * Set: ODR, FS
	 */
	printf("Initializing gyro\n");
	init_gyro_ctx(&gyro_ctx, &hspi1);

	uint8_t gyro_controls[] = {0, 0, 0, 0, 0};
	i3g4250d_write_reg(&gyro_ctx, I3G4250D_CTRL_REG1, gyro_controls, sizeof(gyro_controls)); // clear control registers

	i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_100Hz);
	i3g4250d_full_scale_set(&gyro_ctx, GYRO_SCALE);

	i3g4250d_read_reg(&gyro_ctx, I3G4250D_CTRL_REG1, gyro_controls, sizeof(gyro_controls));
	printf("Gyro control registers: 0x");
	print_hex(gyro_controls, 5);
	/* End gyroscope setup */

	/* Accelerometer setup
	 * Set: ODR, FS, Power mode (Low power, normal, high resolution)
	 */
	printf("Initializing accelerometer\n");
	init_accel_ctx(&accel_ctx, &hi2c1);

	uint8_t accel_controls[] = {0x07U, 0, 0, 0, 0, 0};
	lsm303agr_write_reg(&accel_ctx, LSM303AGR_CTRL_REG1_A,
			accel_controls, sizeof(accel_controls)); // clear control registers

	lsm303agr_xl_data_rate_set(&accel_ctx, LSM303AGR_XL_ODR_100Hz);
	lsm303agr_xl_operating_mode_set(&accel_ctx, LSM303AGR_HR_12bit);
	lsm303agr_xl_full_scale_set(&accel_ctx, ACCEL_SCALE);
	lsm303agr_xl_block_data_update_set(&accel_ctx, 0); // might be unnecessary?

	lsm303agr_read_reg(&accel_ctx, LSM303AGR_CTRL_REG1_A, accel_controls, sizeof(accel_controls));
	printf("Accelerometer control registers: 0x");
	print_hex(accel_controls, sizeof(accel_controls));
	/* End accelerometer setup */

	/* Internal magnetometer setup
	 * Set: ODR, PM, OM
	 */
	printf("Initalizing internal magnetometer\n");
	init_mag_i_ctx(&mag_i_ctx, &hi2c1);

	uint8_t mag_i_controls[] = {0x03U, 0, 0};
	lsm303agr_write_reg(&mag_i_ctx, LSM303AGR_CFG_REG_A_M,
			mag_i_controls, sizeof(mag_i_controls));

	lsm303agr_mag_data_rate_set(&mag_i_ctx, LSM303AGR_MG_ODR_100Hz);
	lsm303agr_mag_power_mode_set(&mag_i_ctx, LSM303AGR_HIGH_RESOLUTION);
	lsm303agr_mag_operating_mode_set(&mag_i_ctx, LSM303AGR_CONTINUOUS_MODE);

	lsm303agr_read_reg(&mag_i_ctx, LSM303AGR_CFG_REG_A_M,
			mag_i_controls, sizeof(mag_i_controls));
	printf("Internal magnetometer control registers: 0x");
	print_hex(mag_i_controls, sizeof(mag_i_controls));
	/* End internal magnetometer setup */

	/* External magnetometer setup
	 * Set: FS, [PM, ODR], OM
	 */
	printf("Initalizing external magnetometer\n");
	init_mag_e_ctx(&mag_e_ctx, &hi2c1);

	uint8_t mag_e_controls[] = {0x10U, 0, 0x03U, 0, 0};
	lis3mdl_write_reg(&mag_e_ctx, LIS3MDL_CTRL_REG1,
			mag_e_controls, sizeof(mag_e_controls));

	lis3mdl_full_scale_set(&mag_e_ctx, MAG_E_SCALE);
	lis3mdl_data_rate_set(&mag_e_ctx, LIS3MDL_UHP_155Hz);
	lis3mdl_operating_mode_set(&mag_e_ctx, LIS3MDL_CONTINUOUS_MODE);


	lis3mdl_read_reg(&mag_e_ctx, LIS3MDL_CTRL_REG1,
			mag_e_controls, sizeof(mag_e_controls));
	printf("External magnetometer control registers: 0x");
	print_hex(mag_e_controls, sizeof(mag_e_controls));
	/* End external magnetometer setup */
}

int32_t flash_leds(void* unused, uint32_t elapsed)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
	{
		toggle_led(BLUE);
		set_led(GREEN);
	}
	else
	{
		reset_led(BLUE);
		toggle_led(GREEN);
	}
	return 0;
}

int32_t run_accel(void* data, uint32_t elapsed)
{
	int32_t ret;
	ret = get_accel(&accel_ctx, (accel_t*) data);
	return ret;
}

int32_t run_gyro(void* data, uint32_t elapsed)
{
	int32_t ret;
	ret = get_angular_rate(&gyro_ctx, (angular_rate_t*) data);
	return ret;
}

int32_t run_mag_i(void* data, uint32_t elapsed)
{
	int32_t ret;
	ret = get_mag_i(&mag_i_ctx, (mag_field_t*) data);
	return ret;
}

int32_t run_mag_e(void* data, uint32_t elapsed)
{
	int32_t ret;
	ret = get_mag_e(&mag_e_ctx, (mag_field_t*) data);
	return ret;
}

int32_t print_quantities(void* unused, uint32_t elapsed)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
	{
		printf("Linear acceleration: X %3.2f, Y %3.2f, Z %3.2f [%s]\n",
				acceleration.x, acceleration.y, acceleration.z, acceleration.unit);
		printf("Angular rate: X %3.2f, Y %3.2f, Z %3.2f [%s]\n",
				angular_rate.x, angular_rate.y, angular_rate.z, angular_rate.unit);
		printf("Internal mag reading: X %3.2f, Y %3.2f, Z %3.2f [%s]\n",
				mag_i_field.x, mag_i_field.y, mag_i_field.z, mag_i_field.unit);
		printf("External mag reading: X %3.2f, Y %3.2f, Z %3.2f [%s]\n",
				mag_e_field.x, mag_e_field.y, mag_e_field.z, mag_e_field.unit);
	}
	return 0;
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

	// output symbols
	acceleration.unit = "m/s^2";
	angular_rate.unit = "deg/s";
	mag_i_field.unit = "gauss";
	mag_e_field.unit = "gauss";

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	my_init();
	bad_task_t routines[] = {
		{.task = run_accel, .data = &acceleration, .timer = &htim10, .period = 20000U, .last = 0U},
		{.task = run_gyro, .data = &angular_rate, .timer = &htim10, .period = 20000U, .last = 0U},
		{.task = run_mag_i, .data = &mag_i_field, .timer = &htim10, .period = 20000U, .last = 0U},
		{.task = run_mag_e, .data = &mag_e_field, .timer = &htim10, .period = 10000U, .last = 0U},
		{.task = flash_leds, .data = NULL, .timer = &htim11, .period = 5000U, .last = 0U},
		{.task = print_quantities, .data = NULL, .timer = &htim11, .period = 5000U, .last = 0U},
	};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_Base_Start(&htim11);
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		printf("Compiler thinks routines is %d long\n")
		run_tasks(routines, sizeof(routines) / sizeof(bad_task_t));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
		toggle_led(RED);
		HAL_Delay(50);
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
