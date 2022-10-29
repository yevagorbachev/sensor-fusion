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

/** Pin guide
 * LIS3MDL GRAY 3V3
 * LIS3MDL BLACK GND
 * LIS3MDL VIOLET PB9
 * LIS3MDL WHITE PB6
 * UART BLACK GND
 * UART RED 5V
 * UART GREEN PA10
 * UART WHITE PA9
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

typedef enum
{
	INIT,
	IDLE,
	BTN,
	GYROCAL,
	MAGCAL,
	CONTROL,
} program_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	UPDATE_ROLLING_MEAN(mean, new, n) mean = (mean * n + new) / (n + 1)
#define HCLK_kHz 96000U
#define APB1_kHz 48000U
#define TIM5_RATE_kHz (APB1_kHz / TIM5_PRESCALER) // counts per ms
#define TIM10_RATE_kHz (HCLK_kHz / TIM10_PRESCALER) // counts per ms
#define TIM11_RATE_kHz (HCLK_kHz / TIM11_PRESCALER) // counts per ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
program_mode_t program_mode = INIT;
uint16_t button_time = 0;
uint32_t gyrocal_time;
uint32_t magcal_time;

// control structures
stmdev_ctx_t accel_ctx;
stmdev_ctx_t gyro_ctx;
stmdev_ctx_t mag_i_ctx;
stmdev_ctx_t mag_ctx;

#include "state.h"

// led variables
// so we can have a task use a pointer to one of these 4 LEDs in its data argument
// instead of making 4 tasks to use each LED
led_t orange_led_value = ORANGE;
led_t green_led_value = GREEN;
led_t blue_led_value = BLUE;
led_t red_led_value = RED;

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
void reset_all_led()
{
	HAL_GPIO_WritePin(GPIOD, RED, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, BLUE, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GREEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, ORANGE, GPIO_PIN_RESET);
}

// overrides syscall _write so we can use SWV

int _write(int file, char* ptr, int len)
{
	for (int i = 0; i < len; i++)
	{
		ITM_SendChar(ptr[i]);
	}
	return len;
}

/* Gyroscope setup
 * Set: ODR, FS, DRDY interrupt
 */
void init_gyro(void)
{
#ifdef INC_DEBUG_H_
	printf("Initializing gyro\n");
#endif

	init_gyro_ctx(&gyro_ctx, &hspi1);

	uint8_t gyro_controls[] = {0x07U, 0, 0, 0, 0}; // control register defaults
	i3g4250d_write_reg(&gyro_ctx, I3G4250D_CTRL_REG1, gyro_controls, sizeof(gyro_controls));

	i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_400Hz);
	i3g4250d_full_scale_set(&gyro_ctx, I3G4250D_FS);
	i3g4250d_int2_route_t gyro_int2_cfg = {.i2_empty = 0, .i2_orun = 0, .i2_wtm = 0, .i2_drdy = 0};
	i3g4250d_pin_int2_route_set(&gyro_ctx, gyro_int2_cfg);

	// print to confirm values
	i3g4250d_read_reg(&gyro_ctx, I3G4250D_CTRL_REG1, gyro_controls, sizeof(gyro_controls));

#ifdef INC_DEBUG_H_
	printf("Gyro control registers: 0x");
	print_hex(gyro_controls, 5);
#endif
}

/* Accelerometer setup
 * Set: ODR, FS, Power mode (Low power, normal, high resolution)
 */
void init_accel(void)
{
#ifdef INC_DEBUG_H_
	printf("Initializing accelerometer\n");
	init_accel_ctx(&accel_ctx, &hi2c1);
#endif

	// default values fromm datasheet
	uint8_t accel_controls[] = {0x07U, 0, 0, 0, 0, 0};
	lsm303agr_write_reg(&accel_ctx, LSM303AGR_CTRL_REG1_A,
			accel_controls, sizeof(accel_controls));

	lsm303agr_xl_data_rate_set(&accel_ctx, LSM303AGR_XL_ODR_100Hz);
	lsm303agr_xl_operating_mode_set(&accel_ctx, LSM303AGR_HR_12bit);
	lsm303agr_xl_full_scale_set(&accel_ctx, ACCEL_SCALE);

	lsm303agr_read_reg(&accel_ctx, LSM303AGR_CTRL_REG1_A, accel_controls, sizeof(accel_controls));

#ifdef INC_DEBUG_H_
	printf("Accelerometer control registers: 0x");
	print_hex(accel_controls, sizeof(accel_controls));
#endif
}

void init_devices(void)
{
	printf("Testing API\n");

	__disable_irq(); // the gyro DRDY messes with stuff if we don't do this

	init_gyro();
//	init_accel();
	init_mag_ctx(&mag_ctx, &hi2c1);
	init_mag(&mag_ctx);

	__enable_irq();
}

int32_t toggle_led_task(void* led, uint32_t elapsed)
{
	toggle_led(*((led_t*) led));
	return 0;
}

int32_t mag_print(void* data, uint32_t elapsed)
{
	// exit after 30s
	if ((__HAL_TIM_GET_COUNTER(&htim5) - magcal_time) > 30U * 1000U * TIM5_RATE_kHz)
	{
		program_mode = IDLE;
		reset_led(BLUE);
#ifdef INC_DEBUG_H_
		printf("Finished magnetometer calibration\n");
#endif
	}
	int32_t ret = get_mag(&mag_ctx, data);
	mag_field_t* field = (mag_field_t*) data;
	printf("%f,%f,%f\n", field->x, field->y, field->z);
	return ret;
}

int32_t add_gyro_measurement(void* unused, uint32_t elapsed)
{
	// exit after 3s
	if ((__HAL_TIM_GET_COUNTER(&htim5) - gyrocal_time) > 3U * 1000U * TIM5_RATE_kHz)
	{
		program_mode = IDLE;
		reset_led(GREEN);
#ifdef INC_DEBUG_H_
		printf("Gyroscope offset: %3.2f, %3.2f, %3.2f, [dps]\n",
				angular_rate_bias.x, angular_rate_bias.y, angular_rate_bias.z);
#endif
		return 0;
	}

	int32_t ret = get_angular_rate(&gyro_ctx, &angular_rate_raw);
	if (ret == 0)
	{
#ifdef INC_DEBUG_H_
		printf("Raw anglar rate: %3.2f, %3.2f, %3.2f, [dps]\n",
				angular_rate_raw.x, angular_rate_raw.y, angular_rate_raw.z);
#endif
		UPDATE_ROLLING_MEAN(angular_rate_bias.x, angular_rate_raw.x, angular_rate_bias_n);
		UPDATE_ROLLING_MEAN(angular_rate_bias.y, angular_rate_raw.y, angular_rate_bias_n);
		UPDATE_ROLLING_MEAN(angular_rate_bias.z, angular_rate_raw.z, angular_rate_bias_n);
		angular_rate_bias_n++;
	}
	return ret;
}

int32_t measure_angular_rate(void* unused, uint32_t elapsed)
{
	int32_t ret = get_angular_rate(&gyro_ctx, &angular_rate_raw);
	if (ret == 0)
	{
		angular_rate = apply_gyro_cal(angular_rate_raw, angular_rate_bias);
		angular_rate_time = __HAL_TIM_GET_COUNTER(&htim5);
	}
	return ret;
}

int32_t print_mag_field(void* data, uint32_t elapsed)
{
	return 0;
}

int32_t measure_mag_field(void* unused, uint32_t elapsed)
{
	int32_t ret = get_mag(&mag_ctx, &mag_field_raw);
	if (ret == 0)
	{
		mag_field = apply_mag_cal(mag_field_raw, mag_hard_bias, mag_soft_bias);
		mag_field_time = __HAL_TIM_GET_COUNTER(&htim5);
	}
	return ret;
}

int32_t put_smooth_mag_field(void* data, uint32_t elapsed)
{
	int32_t ret = get_mag(&mag_ctx, &mag_field_raw);
	if (ret)
	{
		return ret;
	}

	mag_field = apply_mag_cal(mag_field_raw, mag_hard_bias, mag_soft_bias);
	put_mag(&mag_ringbuf, mag_field);
	return 0;
}

int32_t print_smooth_mag_field(void* data, uint32_t elapsed)
{
	mag_field_t field = get_smooth_mag(&mag_ringbuf);
	// printf("Raw: %f,%f,%f\t", mag_field_raw.x, mag_field_raw.y, mag_field_raw.z);
	printf("Smooth: %f,%f,%f\t", field.x, field.y, field.z);
	printf("Heading: %3.2f deg\n", get_heading(field));
	return 0;
}

int32_t print_tim5(void* unused, uint32_t elapsed)
{
	uint32_t time = __HAL_TIM_GET_COUNTER(&htim5);
	printf("Elapsed time: %lu ms\n", time / TIM5_RATE_kHz);
	return 0;
}

// just flash while idle
bad_task_t idle_routines[] = {
	{.task = toggle_led_task, .data = &orange_led_value, .timer = &htim11, .period = 2500U, .last = 0U},
	{.task = print_tim5, .data = NULL, .timer = &htim5, .period = 10000U, .last = 0U},
};

// delay for one second, store and print the rolling average over two seconds of measurement
bad_task_t gyrocal_routines[] = {
	{.task = add_gyro_measurement, .data = NULL, .timer = &htim11, .period = 25U, .last = 0U},
	{.task = toggle_led_task, .data = &green_led_value, .timer = &htim11, .period = 1000U, .last = 0U},
};

// delay for one second, print out 10 seconds of measurements
bad_task_t magcal_routines[] = {
	{.task = mag_print, .data = &mag_field_raw, .timer = &htim11, .period = 70U, .last = 0U},
	{.task = toggle_led_task, .data = &blue_led_value, .timer = &htim11, .period = 1000U, .last = 0U},
};

bad_task_t btn_routines[] = {
	{.task = toggle_led_task, .data = &red_led_value, .timer = &htim11, .period = 2500U, .last = 0U},
};

bad_task_t control_routines[] = {
//	{.task = measure_mag_field, .data = NULL, .timer = &htim11, .period = 70U, .last = 0U},
//	{.task = measure_angular_rate, .data = NULL, .timer = &htim11, .period = 25U, .last = 0U},
	{.task = put_smooth_mag_field, .data = NULL, .timer = &htim5, .period = 70U, .last = 0U},
//	{.task = print_mag_field, .data = NULL, .timer = &htim5, .period = 1000U, .last = 0U},
	{.task = print_smooth_mag_field, .data = NULL, .timer = &htim5, .period = 500U, .last = 0U},
	{.task = toggle_led_task, .data = &orange_led_value, .timer = &htim11, .period = 500U, .last = 0U},
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  program_mode = INIT;
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  init_devices();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// start timers
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_Base_Start(&htim11);


	program_mode = IDLE;
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch(program_mode)
		{
			case(IDLE):
				RUN_TASKS(idle_routines);
				break;
			case(GYROCAL):
				RUN_TASKS(gyrocal_routines);
				break;
			case(MAGCAL):
				RUN_TASKS(magcal_routines);
				break;
			case(CONTROL):
				RUN_TASKS(control_routines);
				break;
			case(BTN):
				RUN_TASKS(btn_routines);
				break;
			default:
				RUN_TASKS(idle_routines);
//				printf("Invalid program mode\n");
				break;
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

void handle_button(void)
{
	GPIO_PinState button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	if (button_state == GPIO_PIN_SET && program_mode == IDLE)
	{
		program_mode = BTN;
		button_time = __HAL_TIM_GET_COUNTER(&htim5);
	}
	else if (button_state == GPIO_PIN_RESET && program_mode == BTN)
	{
		uint16_t elapsed = __HAL_TIM_GET_COUNTER(&htim5) - button_time;
		int ct_per_sec = 1000 * TIM5_RATE_kHz;
		printf("%3.2fs elapsed, ", ((float_t) elapsed) / ct_per_sec);
		if (elapsed < ct_per_sec)
		{
			printf("Mode change to GYROCAL\n");
			program_mode = GYROCAL;
			gyrocal_time = __HAL_TIM_GET_COUNTER(&htim5);
		}
		else if (elapsed < 3 * ct_per_sec)
		{
			printf("Mode change to MAGCAL\n");
			program_mode = MAGCAL;
			magcal_time = __HAL_TIM_GET_COUNTER(&htim5);
		}
		else
		{
			printf("Mode change to CONTROL\n");
			program_mode = CONTROL;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	printf("EXTI %u\n", GPIO_Pin);
	switch(GPIO_Pin)
	{
		case(1):
			handle_button();
			break;
		default:
			printf("Pin %u ISR not defined\n", GPIO_Pin);
			break;
	}
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
