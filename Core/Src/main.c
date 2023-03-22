/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
/* USER CODE BEGIN PV */

/* static inline void spin(volatile uint32_t count) { */
/*   while (count--) asm("nop"); */
/* } */

/* #define BIT(x) (1UL << (x)) */
/* #define PIN(bank, num) ((((bank) - 'A') << 8) | (num)) */
/* #define PINNO(pin) (pin & 255) */
/* #define PINBANK(pin) (pin >> 8) */

/* #define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank))) */


/* enum { APB1_PRE = 4 /\* AHB clock / 4 *\/, APB2_PRE = 4 /\* AHB clock / 2 *\/ }; */
/* enum { PLL_HSI = 16, PLL_M = 16, PLL_N = 336, PLL_P = 4 };  // Run at 180 Mhz */
/* #define SYS_FREQUENCY ((PLL_HSI * PLL_N / PLL_M / PLL_P) * 1000000) */
/* #define APB2_FREQUENCY (SYS_FREQUENCY / (BIT(APB2_PRE - 3))) */
/* #define APB1_FREQUENCY (SYS_FREQUENCY / (BIT(APB1_PRE - 3))) */


/* struct gpio { */
/*   volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2]; */
/* }; */
/* enum { GPIO_MODE_IN, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANLOG }; */

/* static inline void gpio_set_mode(uint16_t pin, uint8_t mode) { */
/*   struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank */
/*   int n = PINNO(pin);                      // Pin number */
/*   RCC->AHB1ENR |= BIT(PINBANK(pin));       // Enable GPIO clock */
/*   gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting */
/*   gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode */
/* } */

/* static inline void gpio_set_af(uint16_t pin, uint8_t af_num) { */
/*   struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank */
/*   int n = PINNO(pin);                      // Pin number */
/*   gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); */
/*   gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4); */
/* } */

/* static inline void gpio_write(uint16_t pin, uint8_t val) { */
/*   struct gpio *gpio = GPIO(PINBANK(pin)); */
/*   gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16); */
/* } */

/* struct uart { */
/*   volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR; */
/* }; */

/* //#define UART1 ((struct uart *) 0x40011000) */
/* //#define UART2 ((struct uart *) 0x40004400) */
/* //#define UART3 ((struct uart *) 0x40004800) */

/* #define UART1 USART1 */
/* #define UART2 USART2 */
/* #define UART3 USART3 */



/* static inline void uart_init(USART_TypeDef *uart, unsigned long baud) { */
/*   // https://www.st.com/resource/en/datasheet/stm32f429zi.pdf */
/*   uint8_t af = 7;           // Alternate function */
/*   uint16_t rx = 0, tx = 0;  // pins */
/*   uint32_t freq = 0;        // Bus frequency. UART1 is on APB2, rest on APB1 */

/*   if (uart == UART1) freq = APB2_FREQUENCY, RCC->APB2ENR |= BIT(4); */
/*   if (uart == UART2) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(17); */
/*   if (uart == UART3) freq = APB1_FREQUENCY, RCC->APB1ENR |= BIT(18); */

/*   if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10); */
/*   if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3); */
/*   if (uart == UART3) tx = PIN('C', 10), rx = PIN('C', 11); */

/*   gpio_set_mode(tx, GPIO_MODE_AF); */
/*   gpio_set_af(tx, af); */
/*   gpio_set_mode(rx, GPIO_MODE_AF); */
/*   gpio_set_af(rx, af); */
/*   uart->CR1 = 0;                           // Disable this UART */
/*   uart->BRR = freq / baud;                 // Set baud rate */
/*   uart->CR1 |= BIT(13) | BIT(2) | BIT(3);  // Set UE, RE, TE */
/* } */

/* static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) { */
/*   uart->DR = byte; */
/*   while ((uart->SR & BIT(7)) == 0) spin(1); */
/* } */

/* static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) { */
/*   while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++); */
/* } */

/* static inline int uart_read_ready(USART_TypeDef *uart) { */
/*   return uart->SR & BIT(5);  // If RXNE bit is set, data is ready */
/* } */

/* static inline uint8_t uart_read_byte(USART_TypeDef *uart) { */
/*   return (uint8_t) (uart->DR & 255); */
/* } */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  uint8_t byte;
  HAL_UART_Receive_IT(&huart3, &byte, 1);
    /* HAL_UART_MspInit(&huart3); */

  
  /* uart_init(UART3, 115200);              // Initialise UART */
  /* uart_write_buf(UART3, "Hola chiquita1\r\n", 16); */
  /* uart_write_buf(UART3, "Hola chiquita1\r\n", 16); */
  /* uart_write_buf(UART3, "Hola chiquita1\r\n", 16);   */

    /* uint8_t txt[100]="GPIB1\n\r"; */
    /* HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500); */
    /* HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500); */
    /* HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500); */
    /* HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityAboveNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    uint8_t txt[100]="GPIB1\r\n";
    uint8_t rec[10];
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500);
    /* HAL_UART_Receive(&huart3, &rec[0], sizeof(uint8_t)*1, 500); */
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    uint8_t txt[100]="GPIB2\r\n";
    HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
           /* uart_write_buf(UART3, "Hola chiquita2\r\n", 16); */
    osDelay(5000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    uint8_t txt[100]="GPIB3\r\n";
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart3, txt, sizeof(txt), 500);
    osDelay(30000);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

uint8_t cmd_buffer[10];
uint8_t cmd_len=0;
char msg[50] = "Your command:\r\n";
char msg1[4] = "\r\n";

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  uint8_t byte;
  HAL_UART_Receive_IT(&huart3, &byte, 1);
  cmd_buffer[cmd_len++] = byte & 0xFF;
  if (byte == '\r' || cmd_len >8){
    if (cmd_buffer[0] == '\n'){
      cmd_buffer[0] = cmd_buffer[1];
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)&msg, sizeof(msg), 500);
    HAL_UART_Transmit(&huart3, (uint8_t*)&cmd_buffer, cmd_len-1, 500);
    HAL_UART_Transmit(&huart3, (uint8_t*)&msg1, sizeof(msg1), 100);
    cmd_len = 0;
  }
}
