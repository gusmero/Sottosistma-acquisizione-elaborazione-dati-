/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  *
  *
  * Project written by : Andrea Gusmara (831141).
  *
  * last modify: 10/08/2020
  *
  * Consegna progetto:
  * Si	vuole	progettare	un	sottosistema	di	acquisizione	e	prima	elaborazione	dati	da	sensori	per	un	drone	che	verrà	utilizzato
  *	per	 l’ispezione	 di	 torri	 eoliche.	 Il	 drone	 non	 necessita	 di	 essere	 certificato	 per	 il	 volo	 civile	 e	 il	 sottosistema	 di
  *	acquisizione/elaborazione	 dati	 deve	 essere	 poco	 costoso.	 Si	 è	 pertanto	 scelto	 di	 basare	 il	 sottosistema	 su	 un
  *	microcontrollore	ARM	e	su	sensori	 facilmente	reperibili	sul	mercato.	La	versione	che	progetterete	voi	sarà	basata	sul
  *	microcontrollore	STM32	F767ZI	installato	sulla	scheda	di	sviluppo	Nucleo	usata	a	laboratorio	e	sui	sensori	sullo	shield	di
  *	espansione	 X-NUCLEO-IKS01A2,	 oltre	 ad	 altri	 apparati	 sensoriali	 non	 gestiti	 da	 questa	 board.	 Il	 sottosistema	 deve
  *	funzionare	 in	 tempo	 reale	 con	 opportuni	 vincoli	 temporali	 che saranno	 successivamente	 specificati. Il	 sottosistema
  *	interagisce	 con	 i	 seguenti	 sensori	 dello	 shield	 di	 espansione: accelerometro	 1,	 accelerometro 2,	 giroscopio,
  *	magnetometro,	barometro.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include "lsm6dsl_reg.h"
#include "lsm303agr_reg.h"
#include "lps22hb_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { false = 0, true = 1} bool;


typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUCLEO_F767ZI
#define TX_BUF_DIM          1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static axis1bit32_t data_raw_pressure;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_magnetic;
static axis3bit16_t data_raw_angular_rate;
static float coda_x_ang[10];
static float coda_y_ang[10];
static float coda_z_ang[10];
static float coda_x_acc1[10];
static float coda_y_acc1[10];
static float coda_z_acc1[10];
static float coda_x_acc2[10];
static float coda_y_acc2[10];
static float coda_z_acc2[10];
static float coda_mg_x[3];
static float coda_mg_y[3];
static float coda_mg_z[3];
static float coda_bar[40];
static float pressure;
static float acceleration_mg_mean[3];
static float acceleration2_mean_mean[3];
static float magnetic_mean[3];
static float angular_mean[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[TX_BUF_DIM];

static bool tim7_it;
static bool acc_and_giro_flag, magn_flag, mean_flag, baro_flag,display_flag;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static int32_t lsm303agr_platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t lsm303agr_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t lsm6dsl_platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t lsm6dsl_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t lps22hb_platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t lps22hb_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void acceleration2_acquisition(stmdev_ctx_t *lsm6dsl_dev_ctx, int8_t i);
static void acceleration1_acquisition(stmdev_ctx_t *lsm303_dev_ctx_xl, int8_t i);
static void magnetometer_acquisition(stmdev_ctx_t *lsm303_dev_ctx_mg, int8_t i);
static void gyroscope_acquisition(stmdev_ctx_t *lsm6dsl_dev_ctx, int8_t i);
static void pressure_acquisition(stmdev_ctx_t *lps22hb_dev_ctx, int8_t i);
static void compute_mean(int8_t size, float *buffx, float *buffy, float *buffz, float *result);
static void compute_mean_bar(int8_t size, float *buff, float result);
static void initialization();
static stmdev_ctx_t lsm6dsl_init(I2C_HandleTypeDef *hi2c);
static stmdev_ctx_t lsm303agr_init_xl(I2C_HandleTypeDef *hi2c);
static stmdev_ctx_t lsm303agr_init_mg(I2C_HandleTypeDef *hi2c);
static stmdev_ctx_t lps22hb_init_mg(I2C_HandleTypeDef *hi2c);
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
	int sens=0;
		int i=0;
		int j=0;
		int k=0;
		bool page=false;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* User initialization : Here we initialize all sensors , the channel i2c connecting
   * the display and start timers*/
  lcdInit(&hi2c1);
  initialization();
  stmdev_ctx_t lsm6dsl_dev_ctx =lsm6dsl_init(&hi2c1);
  stmdev_ctx_t lsm303_dev_ctx_xl = lsm303agr_init_xl(&hi2c1);
  stmdev_ctx_t lsm303_dev_ctx_mg = lsm303agr_init_mg(&hi2c1);
  stmdev_ctx_t lps22hb_dev_ctx = lps22hb_init_mg(&hi2c1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  uint32_t a= HAL_GetTick();
	  /*compute mean for all sernsors*/
	     if(mean_flag){

	 		mean_flag=false;
	 		compute_mean(10, &coda_x_acc1,  &coda_y_acc1,  &coda_z_acc1, &acceleration_mg_mean);
	 		compute_mean(10, &coda_x_acc2,  &coda_y_acc2,  &coda_z_acc2, &acceleration2_mean_mean);
	 		compute_mean(10, &coda_x_ang,  &coda_y_ang,  &coda_z_ang, &angular_mean);
	 		compute_mean(3, &coda_mg_x,  &coda_mg_y,  &coda_mg_z, &magnetic_mean);
	 		compute_mean_bar( 40, &coda_bar, pressure );

	     }

	     /*take data from accelerometer and gyroscope*/
	     if(acc_and_giro_flag){
	 		acc_and_giro_flag=false;
	 		acceleration1_acquisition(&lsm6dsl_dev_ctx, i);
	 		acceleration2_acquisition(&lsm303_dev_ctx_xl, i);
	 		gyroscope_acquisition(&lsm6dsl_dev_ctx, i);
	 		if (i == 10){
	 			i = 0;
	 		}else{
	 			i = i + 1;
	 		}
	 	}
	     /*take data from magnetometer*/
	     if(magn_flag){
	     	magn_flag=false;
	 		magnetometer_acquisition(&lsm303_dev_ctx_mg, j);
	 		 if (j == 3){
	 			j = 0;
	 		}else{
	 			j = j + 1;
	 		}
	 	}
	     /*take data from barometer*/
	     if(baro_flag){
	     	baro_flag=false;
	     	pressure_acquisition(&lps22hb_dev_ctx, k);
	 		 if (k == 40){
	 			k = 0;
	 		}else{
	 			k = k + 1;
	 		}
	     }

	     /*every second print to LCD the result of the learned data*/
	     if(display_flag){
	 		  display_flag=false;
	 		  /*print the result of the first accelerometer*/
	 		  if(sens==0){
	 			  /*print X and Y*/
	 			  if(!page){
	 					sprintf((char*)tx_buffer, "Acc1 X:%4d   ",
	 									  (int)acceleration_mg_mean[0]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "Y: %4d       ",
	 						 (int)acceleration_mg_mean[1]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }else{
	 				 /*print Z*/
	 					sprintf((char*)tx_buffer, "Z: %4d       ",
	 									  (int)acceleration_mg_mean[2]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "                ");
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }

	 			  /*print the result of the second accelerometer*/
	 		  }else if(sens==1){
	 			  /*print X and Y*/
	 			  if(!page){
	 					sprintf((char*)tx_buffer, "Acc2 X:%4d   ",
	 									 (int)acceleration2_mean_mean[0]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "Y: %4d       ",
	 							(int)acceleration2_mean_mean[1]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }else{
	 				  /*print Z*/
	 					sprintf((char*)tx_buffer, "Z: %4d       ",
	 							(int)acceleration2_mean_mean[2]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "                ");
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }
	 		/*print the result of the gyroscope*/
	 		  }else if(sens == 2){
	 			  /*print X and Y*/
	 			  if(!page){
	 					sprintf((char*)tx_buffer, "Ang X:%4d    ",
	 									  (int)angular_mean[0]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "Y: %4d       ",
	 							(int)angular_mean[1]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }else{
	 				  /*print Z*/
	 					sprintf((char*)tx_buffer, "Z: %4d       ",
	 							(int)angular_mean[2]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "                ");
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }
	 			  /*print the result of the magnetometer*/
	 		  }else if(sens == 3){
	 			  /*print X and Y*/
	 			  if(!page){
	 					sprintf((char*)tx_buffer, "Mag X:%4d    ",
	 									 (int) magnetic_mean[0]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "Y: %4d       ",
	 							(int)magnetic_mean[1]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }else{
	 				  /*print Z*/
	 					sprintf((char*)tx_buffer, "Z: %4d    ",
	 							(int)magnetic_mean[2]);
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,0);
	 					lcdPrint(tx_buffer);
	 					sprintf((char*)tx_buffer, "                ");
	 					tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 					HAL_Delay(10);
	 					lcdSetCursor(0,1);
	 					lcdPrint(tx_buffer);
	 			  }
	 		/*print the result of the barometer*/
	 		}else if (sens == 4){
	 			sprintf((char*)tx_buffer, "Prs :%6d    ", (int)pressure);
	 			tx_com(tx_buffer, strlen((char const*)tx_buffer));
	 			HAL_Delay(10);
	 			lcdSetCursor(0,0);
	 			lcdPrint(tx_buffer);
	 			sprintf((char*)tx_buffer, "                ");
	 			tx_com( tx_buffer, strlen( (char const*)tx_buffer ) );
	 			HAL_Delay(10);
	 			lcdSetCursor(0,1);
	 			lcdPrint(tx_buffer);
	 		}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
	     /*check if a button has been pressed and check which one*/
	     	uint8_t but = lcdReadButtons();
	     	if(but == 0 || but == 1111){
	     	}else{
	     		/*If press the first button, the sensor to be displayed on the LCD is changed forward*/
	     		if(!(but & 0x01)) {
	     			if(sens < 4){
	     				sens = sens + 1;
	     			}else{
	     				sens = 0;
	     			}
	     		}else if (!(but & 0x02)) {
	     			/*If you press the first button, the variable to be displayed on the LCD is changed*/
	     			if(page){
	     				page=false;
	     			}else{
	     				page=true;
	     			}
	     			/*If you press the fourth  button, the sensor to be displayed on the LCD is changed backwards*/
	     		}else if (!(but & 0x08)) {
	     			if(sens > 0){
	     				sens = sens - 1;
	     			}else{
	     				sens = 4;
	     			}
	     		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3199;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 31999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 * Inizialization
 */
/*Initialization of all variables that trigger the calculation of data as false*/
static void initialization(){
	display_flag = false;
	acc_and_giro_flag = false;
	magn_flag = false;
	baro_flag = false;
}
/* Initialization of all sensors*/


static stmdev_ctx_t lps22hb_init_mg(I2C_HandleTypeDef *hi2c){


	stmdev_ctx_t lps22hb_dev_ctx;
	lps22hb_dev_ctx.write_reg = lps22hb_platform_write;
	lps22hb_dev_ctx.read_reg = lps22hb_platform_read;
	lps22hb_dev_ctx.handle = &hi2c1;

	/* Check device ID */
	lps22hb_device_id_get(&lps22hb_dev_ctx, &whoamI);
	if (whoamI != LPS22HB_ID){
		while(1)/* manage here device not found */;
	}
	/* Restore default configuration */
	lps22hb_reset_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);
	do {
		lps22hb_reset_get(&lps22hb_dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	//lps22hb_block_data_update_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);

	/* Can be enabled low pass filter on output */
	lps22hb_low_pass_filter_mode_set(&lps22hb_dev_ctx, LPS22HB_LPF_ODR_DIV_2);

	/* Can be set Data-ready signal on INT_DRDY pin */
	//lps22hb_drdy_on_int_set(&lps22hb_dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hb_data_rate_set(&lps22hb_dev_ctx, LPS22HB_ODR_10_Hz);

	return lps22hb_dev_ctx;
}


static stmdev_ctx_t lsm303agr_init_xl(I2C_HandleTypeDef *hi2c){
	 /* Initialize mems driver interface */
	  stmdev_ctx_t dev_ctx_xl;
	  dev_ctx_xl.write_reg = lsm303agr_platform_write;
	  dev_ctx_xl.read_reg = lsm303agr_platform_read;
	  dev_ctx_xl.handle = (void*)LSM303AGR_I2C_ADD_XL;

	  /* Check device ID */
	  whoamI = 0;
	  lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
	  if ( whoamI != LSM303AGR_ID_XL )
	    while(1); /*manage here device not found */


	  /* Enable Block Data Update */
	  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
	  /* Set Output Data Rate */
	  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
	  /* Set accelerometer full scale */
	  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
	  /* Enable temperature sensor */
	  lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
	  /* Set device in continuos mode */
	  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);



	  return dev_ctx_xl;
}

static stmdev_ctx_t lsm6dsl_init(I2C_HandleTypeDef *hi2c){
	stmdev_ctx_t lsm6dsl_dev_ctx;
	  lsm6dsl_dev_ctx.write_reg = lsm6dsl_platform_write;
	  lsm6dsl_dev_ctx.read_reg = lsm6dsl_platform_read;
	  lsm6dsl_dev_ctx.handle = hi2c;


	  whoamI = 0;
	  lsm6dsl_device_id_get(&lsm6dsl_dev_ctx, &whoamI);
	  if ( whoamI != LSM6DSL_ID )
		  while(1)
	      	; /*manage here device not found */
	  /*
	   *  Restore default configuration
	   */
		lsm6dsl_reset_set(&lsm6dsl_dev_ctx, PROPERTY_ENABLE);
		do {
		  lsm6dsl_reset_get(&lsm6dsl_dev_ctx, &rst);
		} while (rst);
		/*
		*  Enable Block Data Update
		*/
		lsm6dsl_block_data_update_set(&lsm6dsl_dev_ctx, PROPERTY_ENABLE);
		/*
		* Set Output Data Rate
		*/
		lsm6dsl_xl_data_rate_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_ODR_12Hz5);
		lsm6dsl_gy_data_rate_set(&lsm6dsl_dev_ctx, LSM6DSL_GY_ODR_12Hz5);
		/*
		* Set full scale
		*/
		lsm6dsl_xl_full_scale_set(&lsm6dsl_dev_ctx, LSM6DSL_2g);
		lsm6dsl_gy_full_scale_set(&lsm6dsl_dev_ctx, LSM6DSL_2000dps);

		/*
		* Configure filtering chain(No aux interface)
		*/
		/* Accelerometer - analog filter */
		lsm6dsl_xl_filter_analog_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_ANA_BW_400Hz);

		/* Accelerometer - LPF1 path ( LPF2 not used )*/
		//lsm6dsl_xl_lp1_bandwidth_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);

		/* Accelerometer - LPF1 + LPF2 path */
		lsm6dsl_xl_lp2_bandwidth_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100);

		/* Accelerometer - High Pass / Slope path */
		//lsm6dsl_xl_reference_mode_set(&lsm6dsl_dev_ctx, PROPERTY_DISABLE);
		//lsm6dsl_xl_hp_bandwidth_set(&lsm6dsl_dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);

		/* Gyroscope - filtering chain */
		lsm6dsl_gy_band_pass_set(&lsm6dsl_dev_ctx, LSM6DSL_HP_260mHz_LP1_STRONG);
		return lsm6dsl_dev_ctx;
}

static stmdev_ctx_t lsm303agr_init_mg(I2C_HandleTypeDef *hi2c){
	stmdev_ctx_t dev_ctx_mg;
	dev_ctx_mg.write_reg = lsm303agr_platform_write;
	dev_ctx_mg.read_reg = lsm303agr_platform_read;
	dev_ctx_mg.handle = (void*)LSM303AGR_I2C_ADD_MG;
	/* Check device ID */
	whoamI = 0;
	lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
	if ( whoamI != LSM303AGR_ID_MG )
	while(1); /*manage here device not found */

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);
	do {
	 lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
	} while (rst);
	/* Enable Block Data Update */
	lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
	/* Set / Reset magnetic sensor mode */
	lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
	/* Enable temperature compensation on mag sensor */
	lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
	/* Set magnetometer in continuos mode */
	lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_CONTINUOUS_MODE);

	return dev_ctx_mg;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	/*the variable that allows the LCD to be updated with an interrupt at 1 hz is activated*/
	 if(htim->Instance == TIM6){
		 baro_flag=true;
	 }
	 /*the variable that allows the compute of acceletometer and gyroscope with an interrupt at 30 hz is activated*/
	 if(htim->Instance == TIM7){
		 acc_and_giro_flag=true;
	 	 }
	 /*the variable that allows the compute of mean and magnetometer with an interrupt at 10 hz is activated*/
	 if(htim-> Instance == TIM4){
		 magn_flag =true;
		 mean_flag=true;
	 }
	 /*the variable that allows the compute of barometer with an interrupt at 75 hz is activated*/
	 if(htim-> Instance == TIM3){
		display_flag =true;
		 }
}



static int32_t lps22hb_platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		/* Write multiple command */
		HAL_I2C_Mem_Write(handle, LPS22HB_I2C_ADD_H, reg,
		I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	}
	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lps22hb_platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	if (handle == &hi2c1) {
		/* Read multiple command */
		HAL_I2C_Mem_Read(handle, LPS22HB_I2C_ADD_H, reg,
		I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	}
	return 0;
}
static int32_t lsm303agr_platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
  }
  HAL_I2C_Mem_Write(&hi2c1, i2c_add, Reg,
                    I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  return 0;
}

static int32_t lsm303agr_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  uint32_t i2c_add = (uint32_t)handle;
  if (i2c_add == LSM303AGR_I2C_ADD_XL)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
  }
  HAL_I2C_Mem_Read(&hi2c1, (uint8_t) i2c_add, Reg,
                   I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  return 0;
}
static int32_t lsm6dsl_platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_H, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}

static int32_t lsm6dsl_platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
      HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_H, Reg,
                       I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}
/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len ) {
  HAL_UART_Transmit(&huart3, tx_buffer, len, 1000);
}


/*Take the accelerometer values and converted into the components of the acceleration vector on the 3 axes*/
static void acceleration1_acquisition(stmdev_ctx_t *dev_ctx, int8_t i){
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(dev_ctx, &reg.status_reg);
	memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
	lsm6dsl_acceleration_raw_get(dev_ctx, data_raw_acceleration.u8bit);
	coda_x_acc1[i] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[0]);
	coda_y_acc1[i] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[1]);
	coda_z_acc1[i] = lsm6dsl_from_fs2g_to_mg( data_raw_acceleration.i16bit[2]);
}
/*Take the accelerometer values and converted into the components of the acceleration vector on the 3 axes*/
static void acceleration2_acquisition(stmdev_ctx_t *dev_ctx, int8_t i){
			/* Read magnetic field data */
	memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
	lsm303agr_acceleration_raw_get(dev_ctx, data_raw_acceleration.u8bit);
	coda_x_acc2[i] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[0] );
	coda_y_acc2[i] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[1] );
	coda_z_acc2[i] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[2] );
}

/*I take the values of the gyroscope and converted into the components of the angular velocity vector on the 3 axes*/
static void gyroscope_acquisition(stmdev_ctx_t *dev_ctx, int8_t i){
	memset(data_raw_angular_rate.u8bit, 0x00, 3*sizeof(int16_t));
	lsm6dsl_angular_rate_raw_get(dev_ctx, data_raw_angular_rate.u8bit);
	coda_x_ang[i] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
	coda_y_ang[i] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
	coda_z_ang[i] = lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

}

/*I take the values of the magnetometer converted into the components on the 3 axes of the magnetic flux density vector*/
static void magnetometer_acquisition(stmdev_ctx_t *dev_ctx, int8_t i){
	memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
	lsm303agr_magnetic_raw_get(dev_ctx, data_raw_magnetic.u8bit);
	coda_mg_x[i] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[0]);
	coda_mg_y[i] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[1]);
	coda_mg_z[i] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[2]);

}

/*I take the values of the magnetometer converted into a pressure value*/
static void pressure_acquisition(stmdev_ctx_t *dev_ctx, int8_t i){
  memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
  lps22hb_pressure_raw_get(dev_ctx, data_raw_pressure.u8bit);
  coda_bar[i] = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);
}

/*compute the mean of the barometer*/
static void compute_mean_bar(int8_t size, float *buff, float result){
	float tot=0;
	int8_t i=0;
	while(i<=size){
		tot = tot + buff[i];
		i=i+1;
	}
	result = tot/size;
}


/*compute the mean*/
static void compute_mean(int8_t size, float *buffx, float *buffy, float *buffz, float *result){
	float tot_x=0;
	float tot_y=0;
	float tot_z=0;
	int8_t i=0;
	while(i<=size){
		tot_x = tot_x + buffx[i];
		tot_y = tot_y + buffy[i];
		tot_z = tot_z + buffz[i];
		i=i+1;
	}
	result[0]=tot_x/size;
	result[1]=tot_y/size;
	result[2]=tot_z/size;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
