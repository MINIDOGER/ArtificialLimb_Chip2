/* USER CODE BEGIN Header */

/********************************修改说明**************************************
*修改日期:20230620	编写基本(详解见初代码)功能
*修改日期:20230621	编写发送功能与指令中断
*修改日期:20230721	足底压力接收
*修改日期:20230726	原串口中断和数据输出修改DMA版本，以减小芯片负担，原程序代码也进行了保留
*修改日期:20231015	增加距离传感器读取函数与逻辑的判断发送（未测试-已测20231016）
*修改日期:20231016	距离传感器测试，修改部分错误代码，存在问题：暂时解码错误（待解决）
*修改日期:20231223	增加拟合代码
*******************************************************************************/

/*
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                            O\ = /O
//                        ____/`---'\____
//                      .   ' \\| |// `.
//                       / \\||| : |||// \
//                     / _||||| -:- |||||- \
//                       | | \\\ - /// | |
//                     | \_| ''\---/'' | |	|
//                      \ .-\__ `-` ___/-. /
//                   ___`. .' /--.--\ `. . __
//                ."" '< `.___\_<|>_/___.' >'"".
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                 \ \ `-. \_ __\ /__ _/ .-` / /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//
//         .............................................
//                  佛祖保佑   芯片不烧   永无BUG
//          佛曰:
//                  写字楼里写字间，写字间里程序员；
//                  程序人员写程序，又拿程序换酒钱。
//                  酒醒只在网上坐，酒醉还来网下眠；
//                  酒醉酒醒日复日，网上网下年复年。
//                  但愿老死电脑间，不愿鞠躬老板前；
//                  奔驰宝马贵者趣，公交自行程序员。
//                  别人笑我忒疯癫，我笑自己命太贱；
//                  不见满街漂亮妹，哪个归得程序员？
*/

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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "math.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/********************************参数声明**************************************
*参数用途:	MPU6050关节角度等信息
*修改日期:	20230620
*******************************************************************************/
struct joint Left = {
		.Hip.AngxZero = 0, //左关节数据组
		.Knee.AngxZero = 0,
		.Ankle.AngxZero = 0,
		.Hip.AngxCal = 0,
		.Knee.AngxCal = 0,
		.Ankle.AngxCal = 0,
};
struct DataUnionBuf DataLeftBufAng; //左关节数据组共同体缓存
struct DataUnionBuf DataLeftBufFoot = {
		.Point = {3,4,7,8,9,12,13,14},
};

/********************************参数声明**************************************
*参数用途:	中断数据缓存
*修改日期:	20230620
*******************************************************************************/
struct cRxBuf{
	uint8_t cRx_1;
	uint8_t cRx_2;
	uint8_t cRx_3;
	uint8_t cRx_4;
	uint8_t cRx_5;
	uint8_t cRx_6;
	uint8_t rxBuf_2[256];
	uint32_t rxBufCursor_2;
};
struct cRxBuf cRx;

/********************************参数声明**************************************
*参数用途:	主函数运行模式
*修改日期:	20230527
*******************************************************************************/
int Mode = 0;
int ModeFlag = 0;
int ModePrint = 1;
int FlagSend = 0;

/********************************参数声明**************************************
*参数用途:	拟合数据参数
*修改日期:	20230531
*******************************************************************************/
struct DataFit Normal = {
		.ploy_n = 5,
//		.ploy_n = 4,
		.Fit_Mode = 4,
		.sizenum = 0,
		.State = 0,
		.FitStart = 2,
		.Flag_Div = 0,
		.Flag_Fit = 0,
		.Flag_Send = 0,
};
//int sizenum;
//int dimension = 5;

/********************************参数声明**************************************
*参数用途:	距离传感器数据
*修改日期:	20231012
*******************************************************************************/
struct ModBus{
	uint8_t ModbusData[7];
	int ModbusCounter;
	uint16_t ModbusDataDEC;
	int ModbusFlag;
};
struct ModBus Modbus = {
	.ModbusCounter = 0,
	.ModbusFlag = 0.
};

///********************************参数声明**************************************
//*参数用途:	滤波器参数
//*修改日期:	20231129
//*******************************************************************************/
//float fc = 5.0f;     		//截止频率
//float Ts = 0.01f;    		//采样周期
//static float pi = 3.14159f; //π
//float alpha = 0;    		//滤波系数

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  DataLeftBufAng.HexBufSumAng[0] = 0x62;
  DataLeftBufAng.HexBufSumAng[13] = 0x63;
  DataLeftBufAng.HexBufSumFoot[0] = 0x64;
  DataLeftBufAng.HexBufSumFoot[33] = 0x65;
  DataLeftBufAng.HexBufDis[0] = 0x66;
  DataLeftBufAng.HexBufDis[1] = 0x01;
  DataLeftBufAng.HexBufDis[2] = 0x67;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0); //输出缓冲

	if(HAL_UART_Receive_IT(&huart1, &cRx.cRx_1, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
		Error_Handler();
		//printf("[info]Error_huart1_IT\r\n");
	}

	if(HAL_UART_Receive_IT(&huart2, &cRx.cRx_2, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
		Error_Handler();
		//printf("[info]Error_huart2_IT\r\n");
	}
//	if(HAL_UART_Receive_IT(&huart3, &cRx.cRx_3, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
//		Error_Handler();
//		//printf("[info]Error_huart3_IT\r\n");
//	}
//	if(HAL_UART_Receive_IT(&huart4, &cRx.cRx_4, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
//		Error_Handler();
//		//printf("[info]Error_huart4_IT\r\n");
//	}
//	if(HAL_UART_Receive_IT(&huart5, &cRx.cRx_5, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
//		Error_Handler();
//		//printf("[info]Error_huart5_IT\r\n");
//	}
	if(HAL_UART_Receive_IT(&huart6, &cRx.cRx_6, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
		Error_Handler();
		//printf("[info]Error_huart5_IT\r\n");
	}

//	// 开启串口1空闲中断
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
////	// 开启DMA发送通道的发送完成中断，才能实现封装发送函数里面的等待功能
////	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
//	// 清除空闲标志位，防止中断误入
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//	// 立即就要打开DMA接收
//	// 不然DMA没有提前准备，第一次接收的数据是读取不出来的
//	HAL_UART_Receive_DMA(&huart1, p_IsToReceive1, MAX_RX_LEN);

	// 开启串口3空闲中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	HAL_UART_Receive_DMA(&huart3, p_IsToReceive3, MAX_RX_LEN);

	// 开启串口4空闲中断
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart4);
	HAL_UART_Receive_DMA(&huart4, p_IsToReceive4, MAX_RX_LEN);

	// 开启串口5空闲中断
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	HAL_UART_Receive_DMA(&huart5, p_IsToReceive5, MAX_RX_LEN);

//	// 开启串口6空闲中断
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
////	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
//	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
//	HAL_UART_Receive_DMA(&huart6, p_IsToReceive6, MAX_RX_LEN);


	low_pass_filter_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Mode == 1)
		{
			if(ModeFlag == 1 && FlagSend == 1){
				for(int i=0;i<3;i++)
				{
					switch(i)
					{
					case 0:
						DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Hip.AngxCal;
						for(int j=0;j<4;j++)
						{
							DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
						}
						break;
					case 1:
						DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Knee.AngxCal;
						for(int j=0;j<4;j++)
						{
							DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
						}
						break;
					case 2:
						DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Ankle.AngxCal;
						for(int j=0;j<4;j++)
						{
							DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
						}
						break;
					default:
						break;
					}
				}
				for(int i=0;i<8;i++)
				{
					DataLeftBufFoot.DataUnionBufSend.FloatBuf = DataLeftBufFoot.Data[DataLeftBufFoot.Point[i]];
					for(int j=0;j<4;j++)
					{
						DataLeftBufFoot.HexBufSumFoot[4*i+j+1] = DataLeftBufFoot.DataUnionBufSend.HexBuf[j];
					}
				}

				DMA_USART2_Tx_Data(DataLeftBufAng.HexBufSumAng,14);

				DMA_USART2_Tx_Data(DataLeftBufFoot.HexBufSumFoot,34);

//				if(Modbus.ModbusFlag == 1){
//					DataLeftBufFoot.HexBufDis[1] = 0x02;
//					Modbus.ModbusFlag = 0;
//				}
//				else{
//					DataLeftBufFoot.HexBufDis[1] = 0x01;
//				}
//				DMA_USART2_Tx_Data(DataLeftBufFoot.HexBufDis,3);


	//			for(int i=0;i<14;i++)
	//			{
	//				HAL_UART_Transmit(&huart2,&DataLeftBufAng.HexBufSumAng[i],1,0xFFFF);
	//			}

				FlagSend = 0;


			}
			else if(ModeFlag == 0){
//				Normal.State = 1;
				ModeFlag = 1;
//				if(ModePrint == 2){
//					Mode = 2;
//				}
			}
		}
		else if(Mode == 2)
		{
//			for(int i=0;i<3;i++)
//			{
//				switch(i)
//				{
//				case 0:
//					DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Hip.AngxCal;
//					for(int j=0;j<4;j++)
//					{
//						DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
//					}
//					break;
//				case 1:
//					DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Knee.AngxCal;
//					for(int j=0;j<4;j++)
//					{
//						DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
//					}
//					break;
//				case 2:
//					DataLeftBufAng.DataUnionBufSend.FloatBuf = Left.Ankle.AngxCal;
//					for(int j=0;j<4;j++)
//					{
//						DataLeftBufAng.HexBufSumAng[4*i+j+1] = DataLeftBufAng.DataUnionBufSend.HexBuf[j];
//					}
//					break;
//				default:
//					break;
//				}
//			}
//			for(int i=0;i<8;i++)
//			{
//				DataLeftBufFoot.DataUnionBufSend.FloatBuf = DataLeftBufFoot.Data[DataLeftBufFoot.Point[i]];
//				for(int j=0;j<4;j++)
//				{
//					DataLeftBufFoot.HexBufSumFoot[4*i+j+1] = DataLeftBufFoot.DataUnionBufSend.HexBuf[j];
//				}
//			}
//
//			DMA_USART2_Tx_Data(DataLeftBufAng.HexBufSumAng,14);
//
//			DMA_USART2_Tx_Data(DataLeftBufFoot.HexBufSumFoot,34);
//
//			if(Modbus.ModbusFlag == 1){
//				DataLeftBufFoot.HexBufDis[1] = 0x02;
//				Modbus.ModbusFlag = 0;
//			}
//			else{
//				DataLeftBufFoot.HexBufDis[1] = 0x01;
//			}
//			DMA_USART2_Tx_Data(DataLeftBufFoot.HexBufDis,3);


//			for(int i=0;i<14;i++)
//			{
//				HAL_UART_Transmit(&huart2,&DataLeftBufAng.HexBufSumAng[i],1,0xFFFF);
//			}
		}
		else if(Mode == 0)
		{
			ModeFlag = 0;
			Normal.State = 0;
			while(1){
//				DMA_usart2_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//						Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
//						Right.Hip.AngAccx,Right.Knee.AngAccx,Right.Ankle.AngAccx,
//						Right.Hip.Accx,Right.Knee.Accx,Right.Ankle.Accx);
//				HAL_Delay(Delay);
				if(Mode != 0)break;
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(2);
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

/* USER CODE BEGIN 4 */

/********************************实现函数**************************************
*函数原型:	printf()
*功　　能:	输出重定向，串口2输出
*修改日期:	20230526
*******************************************************************************/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}


/********************************实现函数**************************************
*函数原型:	huart1_printf()
*功　　能:	输出重定向，串口1,2输出
*修改日期:	20230526
*******************************************************************************/
void huart1_printf(char * fmt,...)
{
    char buffer[100];
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,100,fmt,arg_ptr);
    while(i<99&&buffer[i])
    {
        HAL_UART_Transmit(&huart1,(uint8_t *)&buffer[i],1,0xFFFF);
        i++;
    }
    va_end(arg_ptr);
}

void huart2_printf(char * fmt,...)
{
    char buffer[100];
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,100,fmt,arg_ptr);
    while(i<99&&buffer[i])
    {
        HAL_UART_Transmit(&huart2,(uint8_t *)&buffer[i],1,0xFFFF);
        i++;
    }
    va_end(arg_ptr);
}


/********************************实现函数**************************************
*函数原型:	HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
*功　　能:	串口中断
*修改日期:	20230530
 * 参数		| 介绍
 * ---------+--------------------------------------
 * huart	| 产生中断的串口
*******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{


//		通信测试，基于CAN电压低的版本的硬件编写
//		if(cRx.cRx_1 == '\n')
//		{
//			float operand;
//			char operator1, operator2;
//			cRx.rxBuf_1[cRx.rxBufCursor_1] = '\0';
//			if(sscanf((const char*)cRx.rxBuf_1, "%c%c%f", &operator1, &operator2, &operand) == 3)
//			{
//				if(operator1 == 'S' && operator2 == '=')
//				{
//					Mode = (int)operand;
//					//printf("[Info] Mode=%d\r\n",Mode);
//				}
//			}
//			cRx.rxBufCursor_1 = 0;
//		}
//		else
//		{
//			if(cRx.rxBufCursor_1 < 255)
//			{
//				cRx.rxBuf_1[cRx.rxBufCursor_1++] = cRx.cRx_1;
//			}
//		}

//		激光距离传感器解码初步方案（暂不行）
//		Modbus.ModbusData[Modbus.ModbusCounter] = cRx.cRx_1;
//		Modbus.ModbusCounter++;
//		if(Modbus.ModbusCounter == 6){
//			uint16_t crc = crc16_modbus(Modbus.ModbusData,5);
//			if((((crc >> 8) & 0xff) == Modbus.ModbusData[5]) && ((crc & 0xff) == Modbus.ModbusData[6])){
//				Modbus.ModbusDataDEC = (short) (Modbus.ModbusData[3] << 8) | Modbus.ModbusData[4];
//				if(Modbus.ModbusDataDEC < 20){
//					Modbus.ModbusFlag = 1;
////					DataLeftBufAng.HexBufDis[1] = 0x01;
////					DMA_USART2_Tx_Data(DataLeftBufAng.HexBufDis,3);
//				}
//				else{
//					Modbus.ModbusFlag = 0;
////					DataLeftBufAng.HexBufDis[1] = 0x02;
////					DMA_USART2_Tx_Data(DataLeftBufAng.HexBufDis,3);
//				}
//			}
//			Modbus.ModbusCounter = 0;
//		}

//		激光距离传感器解码方案2
//		DMA_USART2_Tx_Data(cRx.cRx_1,1);
		Modbus.ModbusData[Modbus.ModbusCounter] = cRx.cRx_1;
//		huart2_printf("%d",Modbus.ModbusCounter);
		if(Modbus.ModbusData[0] == 0x50 && Modbus.ModbusData[1] == 0x03){
			Modbus.ModbusCounter++;
		}
		else{
			Modbus.ModbusCounter = 0;
			Modbus.ModbusData[0] = 0x01;
			Modbus.ModbusData[1] = 0x03;
			ModbusRead();
		}

		if(Modbus.ModbusCounter == 7){
//			DMA_USART2_Tx_Data(Modbus.ModbusData,7);
			Modbus.ModbusDataDEC = (short) (Modbus.ModbusData[3] << 8) | Modbus.ModbusData[4];
//			huart2_printf("%d\n",Modbus.ModbusDataDEC);

			if(Modbus.ModbusDataDEC < 20){
				Modbus.ModbusFlag = 1;
			}
			else{
				Modbus.ModbusFlag = 0;
			}

			Modbus.ModbusCounter = 0;
			Modbus.ModbusData[0] = 0x01;
			Modbus.ModbusData[1] = 0x03;
		}



		if(HAL_UART_Receive_IT(&huart1, &cRx.cRx_1, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart1_IT\r\n");
		}
	}

	else if(huart->Instance == USART2)
	{
		if(cRx.cRx_2 == '\n')
		{
			float operand;
			char operator1, operator2;
			cRx.rxBuf_2[cRx.rxBufCursor_2] = '\0';
			if(sscanf((const char*)cRx.rxBuf_2, "%c%c%f", &operator1, &operator2, &operand) == 3)
			{
				if(operator1 == 'S' && operator2 == '=')
				{
					Mode = (int)operand;
//					printf("[Info] Mode=%d\r\n",Mode);
				}

				switch(operator1)
				{
				case 'M':
					switch(operator2)
					{
					case 'Z':
						if(operand == 1){
							Left.Hip.AngxZero = Left.Hip.Angx;
							Left.Knee.AngxZero = Left.Knee.Angx;
							Left.Ankle.AngxZero = Left.Ankle.Angx;
							Left.WAIST.AccxZero = Left.WAIST.Angx;
						}
						else if(operand == 0){
							Left.Hip.AngxZero = 0;
							Left.Knee.AngxZero = 0;
							Left.Ankle.AngxZero = 0;
							Left.WAIST.AccxZero = 0;
						}
						break;
					case 'B':
						ModbusRead();
//						DMA_USART2_Tx_Data(DataLeftBufAng.HexBufDis,3);
						break;
					case 'P':
						ModePrint = (int)operand;
						break;
					default:
//						printf("[Info]ErrorFun\r\n");
						break;
					}
//					huart1_printf("M%c%d\n",operator2,(int)operand);
//					printf("[InfoMPU6050]");
					break;
//				case 'F':
//					switch(operator2)
//					{
//					case 'Z':
//						if(operand == 1){
//							for(int i=0;i<8;i++)
//							{
//								DataLeftBufFoot.DataZero[DataLeftBufFoot.Point[i]] = DataLeftBufFoot.Data[DataLeftBufFoot.Point[i]];
//							}
//						}
//						else if(operand == 0){
//							for(int i=0;i<8;i++)
//							{
//								DataLeftBufFoot.DataZero[DataLeftBufFoot.Point[i]] = 0;
//							}
//						}
//						break;
//					default:
//						break;
//					}
//					break;
				default:
					break;
				}
			}
			cRx.rxBufCursor_2 = 0;
		}
		else
		{
			if(cRx.rxBufCursor_2 < 255)
			{
				cRx.rxBuf_2[cRx.rxBufCursor_2++] = cRx.cRx_2;
			}
		}
		if(HAL_UART_Receive_IT(&huart2, &cRx.cRx_2, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
			Error_Handler();
			//printf("[info]Error_huart2_IT\r\n");
		}
	}

	else if(huart->Instance == USART3)
	{
		MPU6050ModDataBuf(&Left.Hip,cRx.cRx_3);
		Left.Hip.AngxCal = Left.Hip.Angx;
		if(HAL_UART_Receive_IT(&huart3, &cRx.cRx_3, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
			Error_Handler();
			//printf("[info]Error_huart3_IT\r\n");
		}
	}
	else if(huart->Instance == UART4)
	{
		MPU6050ModDataBuf(&Left.Knee,cRx.cRx_4);
		Left.Knee.AngxCal = Left.Knee.Angx - Left.Hip.Angx;
		if(HAL_UART_Receive_IT(&huart4, &cRx.cRx_4, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
			Error_Handler();
			//printf("[info]Error_huart4_IT\r\n");
		}
	}
	else if(huart->Instance == UART5)
	{
		MPU6050ModDataBuf(&Left.Ankle,cRx.cRx_5);
		Left.Ankle.AngxCal = Left.Ankle.Angx - Left.Knee.Angx;
		if(HAL_UART_Receive_IT(&huart5, &cRx.cRx_5, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
			Error_Handler();
			//printf("[info]Error_huart5_IT\r\n");
		}
	}
	else if(huart->Instance == USART6)
	{
		FootDataBuf(&DataLeftBufFoot,cRx.cRx_6);
		if(HAL_UART_Receive_IT(&huart6, &cRx.cRx_6, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
			Error_Handler();
			printf("[info]Error_huart6_IT\r\n");
		}
	}
}

/********************************实现函数**************************************
*函数原型:	void MPU6050ModDataBuf(struct Data AllData, uint8_t cRx)
*功　　能:	MPU6050角度等数据读取函数，用于串口中断
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 关节结构体，储存各关节数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void MPU6050ModDataBuf(struct Data *AllData, uint8_t cRx)
{
	  if(AllData->State.counter < 1)
	  {
		  if(cRx != 0x55)
		  {
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
		  }
		  else
		  {
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  }
	  }
	  else if(AllData->State.counter < 2)
	  {
		  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
		  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
		  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  switch(cRx)
		  {
		  case 0x51:
			  //采集加速度
			  AllData->State.Flag = 1;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;

			  //不采集加速度
//			  AllData->State.Flag = 0;
//			  AllData->State.counter = 0;
//			  AllData->State.sum = 0;
			  break;
		  case 0x52:
			  //采集角速度
			  AllData->State.Flag = 2;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;

			  //不采集角速度
//			  AllData->State.Flag = 0;
//			  AllData->State.counter = 0;
//			  AllData->State.sum = 0;
			  break;
		  case 0x53:
			  AllData->State.Flag = 3;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  default:
			  AllData->State.Flag = 0;
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData->State.counter < 11)
	  {
		  switch (AllData->State.Flag)
		  {
		  case 1:
			  AllData->Buf.rxDataAcc[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  case 2:
			  AllData->Buf.rxDataAngAcc[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  case 3:
			  AllData->Buf.rxDataAng[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  default:
			  AllData->State.Flag = 0;
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData->State.counter == 11)
	  {
		  if(AllData->State.sum != cRx)
		  {
			  AllData->State.counter = 0;

		  }
		  else
		  {
			  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  }
		  AllData->State.counter = 0;
		  AllData->State.sum = 0;

		  switch(AllData->State.Flag)
		  {
		  case 1:
			  AllData->Buf.Accx = (AllData->Buf.rxDataAcc[3]<<8)|AllData->Buf.rxDataAcc[2];
			  AllData->Accx = (float) AllData->Buf.Accx/32768*16*g;
			  AllData->Buf.Accy = (AllData->Buf.rxDataAcc[5]<<8)|AllData->Buf.rxDataAcc[4];
			  AllData->Accy = (float) AllData->Buf.Accy/32768*16*g;
			  AllData->Buf.Accz = (AllData->Buf.rxDataAcc[7]<<8)|AllData->Buf.rxDataAcc[6];
			  AllData->Accz = (float) AllData->Buf.Accz/32768*16*g;
			  break;
		  case 2:
			  AllData->Buf.AngAccx = (AllData->Buf.rxDataAngAcc[3]<<8)|AllData->Buf.rxDataAngAcc[2];
			  AllData->AngAccx = (float) AllData->Buf.AngAccx/32768*2000;
			  AllData->Buf.AngAccy = (AllData->Buf.rxDataAngAcc[5]<<8)|AllData->Buf.rxDataAngAcc[4];
			  AllData->AngAccy = (float) AllData->Buf.AngAccy/32768*2000;
			  AllData->Buf.AngAccz = (AllData->Buf.rxDataAngAcc[7]<<8)|AllData->Buf.rxDataAngAcc[6];
			  AllData->AngAccz = (float) AllData->Buf.AngAccz/32768*2000;
			  break;
		  case 3:
			  AllData->Buf.Angx = (AllData->Buf.rxDataAng[3]<<8)|AllData->Buf.rxDataAng[2];
			  AllData->Angx = (float) AllData->Buf.Angx/32768*180;
			  AllData->Buf.Angy = (AllData->Buf.rxDataAng[5]<<8)|AllData->Buf.rxDataAng[4];
			  AllData->Angy = (float) AllData->Buf.Angy/32768*180;
			  AllData->Buf.Angz = (AllData->Buf.rxDataAng[7]<<8)|AllData->Buf.rxDataAng[6];
			  AllData->Angz = (float) AllData->Buf.Angz/32768*180;

			  AllData->Angx = AllData->Angx - AllData->AngxZero;
			  if(AllData->Angx > 180)
			  {
				  AllData->Angx = AllData->Angx - 360;
			  }
			  break;
		  default:
			  break;
		  }

		  AllData->State.Flag = 0;
	  }
}

/********************************实现函数**************************************
*函数原型:	void FootDataBuf()
*功　　能:	足底压力共同体数据
*修改日期:	20230721
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 共同体数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void FootDataBuf(struct DataUnionBuf *AllData, uint8_t cRx)
{
	if(AllData->FootCounter < 1)
	{
		if(cRx != 0x64)
		{
			AllData->FootCounter = 0;
		}
		else
		{
			AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
			AllData->FootCounter++;
		}
	}
	else if(AllData->FootCounter < 33)
	{
		AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
		AllData->FootCounter++;
	}
	else if(AllData->FootCounter == 33)
	{
		if(cRx == 0x65)
		{
			AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
			for(int i=0;i<8;i++)
			{
				for(int j=0;j<4;j++)
				{
					AllData->DataUnionBufReceive.HexBuf[j] = AllData->HexBufSumFoot[4*i+j+1];
				}
				AllData->Data[AllData->Point[i]] = AllData->DataUnionBufReceive.FloatBuf;
			}
		}
		AllData->FootCounter = 0;
	}
}
/********************************实现函数**************************************
*函数原型:	void AngDataBuf()
*功　　能:	左侧角度共同体数据
*修改日期:	20230721
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 共同体数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void AngDataBuf(struct DataUnionBuf *AllData, uint8_t cRx)
{
	if(AllData->AngCounter < 1)
	{
		if(cRx != 0x62)
		{
			AllData->AngCounter = 0;
		}
		else
		{
			AllData->HexBufSumAng[AllData->AngCounter] = cRx;
			AllData->AngCounter++;
		}
	}
	else if(AllData->AngCounter < 13)
	{
		AllData->HexBufSumAng[AllData->AngCounter] = cRx;
		AllData->AngCounter++;
	}
	else if(AllData->AngCounter == 13)
	{
		if(cRx == 0x63)
		{
			AllData->HexBufSumAng[AllData->AngCounter] = cRx;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<4;j++)
				{
					AllData->DataUnionBufReceive.HexBuf[j] = AllData->HexBufSumAng[4*i+j+1];
				}
				switch(i)
				{
				case 0:
					Left.Hip.AngxCal = AllData->DataUnionBufReceive.FloatBuf;
					break;
				case 1:
					Left.Knee.AngxCal = AllData->DataUnionBufReceive.FloatBuf;
					break;
				case 2:
					Left.Ankle.AngxCal = AllData->DataUnionBufReceive.FloatBuf;
					break;
				default:
					break;
				}
			}
		}
		AllData->AngCounter = 0;
	}
}

/********************************实现函数**************************************
*函数原型:	float *createArray(int size)
*功　　能:	动态分配内存
*功　　能:	记得在使用完后 free(array); 否则会造成内存泄露
*修改日期:	20230809
 * 参数		| 介绍
 * ---------+--------------------------------------
 * size		| 生成数组大小
*******************************************************************************/
double *createArray(int size)
{
	double *array = (double *) malloc(size * sizeof(double));  // 动态分配内存空间

    if (array == NULL) {
        printf("Memory allocation failed.\n");
//        exit(1);  // 内存分配失败，退出程序
    }

    // 初始化数组
    for (int i = 0; i < size; i++) {
        array[i] = 0;
    }

    return array;
}

/********************************实现函数**************************************
*函数原型:	void DataDiv
*功　　能:	数据分割便于拟合
*修改日期:	20231201
 * 参数		| 介绍
 * ---------+--------------------------------------
 * Fit		| 数据储存位置
*******************************************************************************/
void DataDiv(struct DataFit *Fit){
	if(Fit->FitFlagKnee == 0){
		Fit->FitBufKnee[Fit->sizenum] = Left.Knee.AngxCal;
		if(Fit->sizenum > 3){
			if(	((Fit->FitBufKnee[Fit->sizenum]-Fit->FitBufKnee[Fit->sizenum-1])*
				(Fit->FitBufKnee[Fit->sizenum-1]-Fit->FitBufKnee[Fit->sizenum-2])<0
				&& Fit->sizenum > 20)
				|| Fit->sizenum > 70){
				Fit->FitFlagKnee = 1;
			}
		}
		Fit->sizenum++;
	}
	else if(Fit->FitFlagKnee == 1){
		//内存分配方式1
		double *arrayKnee = createArray(Fit->sizenum);
		double *arrayX = createArray(Fit->sizenum);
		double Init_x = 0;

		//内存分配方式2
//		double *arrayKnee = (double *)calloc(Fit->sizenum + 1 , sizeof(double));
//		double *arrayX = (double *)calloc(Fit->sizenum + 1 , sizeof(double));

		for(int i=0;i<Fit->sizenum;i++){
			Init_x += 0.01;
			arrayX[i] = Init_x;
			arrayKnee[i] = Fit->FitBufKnee[i];// /180*3.14
//			DMA_usart2_printf("%f\r\n",Fit->FitBufKnee[i]);
		}
//		DMA_usart2_printf("%d\r\n",Fit->sizenum);
//		int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//		DMA_usart2_printf("%d\r\n",sizenum);
		polyfit(Fit->sizenum, arrayX, arrayKnee, Normal.ploy_n, Fit->PKneeS1);
		Normal.Flag_Div += 1;
		if(Normal.Flag_Div >= 5){
			Normal.Flag_Div = 1;
		}
		Calculate(Normal.ploy_n, Normal.sizenum, Fit->PKneeS1, arrayX, Normal.Flag_Div);
//		DMA_usart2_printf("%f,%f,%f,%f,%f,%f\r\n",
//				Normal.PKneeS1[0],Normal.PKneeS1[1],Normal.PKneeS1[2],Normal.PKneeS1[3],Normal.PKneeS1[4],Normal.PKneeS1[5]);
		Fit->FitFlagKnee = 0;
		Fit->sizenum = 0;
		free(arrayKnee);
		free(arrayX);
	}
}

/********************************实现函数**************************************
*函数原型:	void DataDiv_2
*功　　能:	数据分割便于拟合
*修改日期:	20231201
 * 参数		| 介绍
 * ---------+--------------------------------------
 * Fit		| 数据储存位置
*******************************************************************************/
void DataDiv_2(struct DataFit *Fit){
	if(Fit->FitFlagKnee == 0){
		Fit->FitBufKnee[Fit->sizenum] = Left.Knee.AngxCal;
		Fit->FitBufFoot_12[Fit->sizenum] = DataLeftBufFoot.Data[12];
		Fit->sizenum++;

		if(DataLeftBufFoot.Data[12] > 1600 && (Fit->Flag_Div == 4 || Fit->Flag_Div == 0)){
//			if(Fit->sizenum > 2 && Fit->FitBufFoot_12[Fit->sizenum] - Fit->FitBufFoot_12[Fit->sizenum - 1] <= 0){
//				if(Fit->Flag_Div == 4){
//					Fit->Flag_Div = 1;
//					Fit->FitFlagKnee = 1;
//				}
//				else if(Fit->Flag_Div == 0){
//					Fit->Flag_Div = 1;
//					Fit->sizenum = 0;
//				}
//			}

			if(Fit->Flag_Div == 4){
				Fit->Flag_Div = 1;
				Fit->FitFlagKnee = 1;
			}
			else if(Fit->Flag_Div == 0){
				Fit->Flag_Div = 1;
				Fit->sizenum = 0;
			}
		}

		else if(Left.Knee.AngxCal < -40 && Fit->Flag_Div == 1){
			if(Fit->sizenum > 2 && (Fit->FitBufKnee[Fit->sizenum] - Fit->FitBufKnee[Fit->sizenum - 1] > 0)){
				if(Fit->Flag_Div == 1){
					Fit->Flag_Div = 2;
					Fit->FitFlagKnee = 1;
				}
				else if(Fit->Flag_Div != 1){

				}
			}
		}

		else if(DataLeftBufFoot.Data[3] > 400 && DataLeftBufFoot.Data[4] > 400 && Fit->Flag_Div == 2){
			if(Fit->Flag_Div == 2){
				Fit->Flag_Div = 3;
				Fit->FitFlagKnee = 1;
			}
			if(Fit->Flag_Div != 2){

			}
		}

		else if(DataLeftBufFoot.Data[12] > 600 && Fit->Flag_Div == 3){
			if(Fit->Flag_Div == 3){
				Fit->Flag_Div = 4;
				Fit->FitFlagKnee = 1;
			}
			if(Fit->Flag_Div != 3){

			}
		}

		else if(Fit->sizenum > 70){
//			DMA_usart2_printf("e\r\n");
			if(Fit->Flag_Div >= 4){
				Fit->Flag_Div = 1;
			}
			else{
				Fit->Flag_Div++;
			}
			Fit->FitFlagKnee = 1;
		}
	}
	else if(Fit->FitFlagKnee == 1){
		//内存分配方式1
		double *arrayKnee = createArray(Fit->sizenum);
		double *arrayX = createArray(Fit->sizenum);
		double Init_x = 0;

		//内存分配方式2
//		double *arrayKnee = (double *)calloc(Fit->sizenum + 1 , sizeof(double));
//		double *arrayX = (double *)calloc(Fit->sizenum + 1 , sizeof(double));

		for(int i=0;i<Fit->sizenum;i++){
			Init_x += 0.01;
			arrayX[i] = Init_x;
			arrayKnee[i] = Fit->FitBufKnee[i];// /180*3.14
//			DMA_usart2_printf("%f\r\n",Fit->FitBufKnee[i]);
		}
//		DMA_usart2_printf("%d\r\n",Fit->sizenum);

//		int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//		DMA_usart2_printf("%d\r\n",sizenum);

		polyfit(Fit->sizenum, arrayX, arrayKnee, Normal.ploy_n, Fit->PKneeS1);
		Normal.Flag_Send += 1;
//		if(Normal.Flag_Div >= Normal.Fit_Mode){
//			Normal.Flag_Div = 1;
//		}

		Calculate(Normal.ploy_n, Fit->sizenum, Fit->PKneeS1, arrayX, Normal.Flag_Div);
//		DMA_usart2_printf("%f,%f,%f,%f,%f,%f\r\n",
//				Normal.PKneeS1[0],Normal.PKneeS1[1],Normal.PKneeS1[2],Normal.PKneeS1[3],Normal.PKneeS1[4],Normal.PKneeS1[5]);

		Fit->FitFlagKnee = 0;
		Fit->sizenum = 0;

		for(int i=0; i<3; i++){
			switch(Normal.Flag_Div){
			case 1:
				Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_0[Normal.FitKnee_0_num-1];
				break;
			case 2:
				Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_1[Normal.FitKnee_1_num-1];
				break;
			case 3:
				Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_2[Normal.FitKnee_2_num-1];
				break;
			case 4:
				Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_3[Normal.FitKnee_3_num-1];
				break;
			default:
				break;
			}
			Fit->sizenum++;
		}

		free(arrayKnee);
		free(arrayX);
	}
}

/********************************实现函数**************************************
*函数原型:	void polyfit(n,x,y,poly_n,a)
*功　　能:	拟合y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * n		| 数据个数
 * x		| 自变量
 * y		| 因变量
 * poly_n	| 多项式项数
 * p		| 系数
*******************************************************************************/
void polyfit(int n,double x[],double y[],int poly_n,double p[])
{
	int i,j;
	double *tempx,*tempy,*sumxx,*sumxy,*ata;

	tempx = (double *)calloc(n , sizeof(double));
	sumxx = (double *)calloc((poly_n*2+1) , sizeof(double));
	tempy = (double *)calloc(n , sizeof(double));
	sumxy = (double *)calloc((poly_n+1) , sizeof(double));
	ata = (double *)calloc( (poly_n+1)*(poly_n+1) , sizeof(double) );
	for (i=0;i<n;i++)
	{
		tempx[i]=1;
		tempy[i]=y[i];
	}
	for (i=0;i<2*poly_n+1;i++)
	{
		for (sumxx[i]=0,j=0;j<n;j++)
		{
			sumxx[i]+=tempx[j];
			tempx[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (sumxy[i]=0,j=0;j<n;j++)
		{
			sumxy[i]+=tempy[j];
			tempy[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (j=0;j<poly_n+1;j++)
		{
			ata[i*(poly_n+1)+j]=sumxx[i+j];
		}
	}
	gauss_solve(poly_n+1,ata,p,sumxy);

	free(tempx);
	free(sumxx);
	free(tempy);
	free(sumxy);
	free(ata);
}

/********************************实现函数**************************************
*函数原型:	void gauss_solve(int n,double A[],double x[],double b[])
*功　　能:	高斯消元法计算得到 n 次多项式的系数
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * n		| 系数个数
 * A		| 线性矩阵
 * x		| 拟合结果
 * b		| 线性方程组的y值
*******************************************************************************/
void gauss_solve(int n,double A[],double x[],double b[])
{
	int i,j,k,r;
	double max;
	for (k=0;k<n-1;k++)
	{
		max=fabs(A[k*n+k]);					// find maxmum
		r=k;
		for (i=k+1;i<n-1;i++)
		{
			if (max<fabs(A[i*n+i]))
			{
				max=fabs(A[i*n+i]);
				r=i;
			}
		}
		if (r!=k)
		{
			for (i=0;i<n;i++)		//change array:A[k]&A[r]
			{
				max=A[k*n+i];
				A[k*n+i]=A[r*n+i];
				A[r*n+i]=max;
			}

			max=b[k];                    //change array:b[k]&b[r]
			b[k]=b[r];
			b[r]=max;
		}

		for (i=k+1;i<n;i++)
		{
			for (j=k+1;j<n;j++)
				A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
			b[i]-=A[i*n+k]*b[k]/A[k*n+k];
		}
	}

	for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
	{
		for (j=i+1,x[i]=b[i];j<n;j++)
			x[i]-=A[i*n+j]*x[j];
	}
}

/********************************实现函数**************************************
*函数原型:	void Calculate(int poly_n, int n, double p[], double x[], int Flag)
*功　　能:	计算拟合函数值
*修改日期:	20231207
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void Calculate(int poly_n, int n, double p[], double x[], int Flag){
	double *q = createArray(poly_n);
	Slop(poly_n, p, q);
	switch (Flag){
	case 1:
		Normal.FitKnee_0_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_0[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_0[i] = Horner_Algorithm(poly_n-1,q,x[i]);
//			DMA_usart2_printf("%f\r\n",Normal.FitKnee_0[i]);
		}
		break;
	case 2:
		Normal.FitKnee_1_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_1[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_1[i] = Horner_Algorithm(poly_n-1,q,x[i]);
//			DMA_usart2_printf("%f\r\n",Normal.FitKnee_1[i]);
		}
		break;
	case 3:
		Normal.FitKnee_2_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_2[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_2[i] = Horner_Algorithm(poly_n-1,q,x[i]);
			DMA_usart2_printf("%f\r\n",Normal.FitKnee_2[i]);
		}
		break;
	case 4:
		Normal.FitKnee_3_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_3[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_3[i] = Horner_Algorithm(poly_n-1,q,x[i]);
//			DMA_usart2_printf("%f\r\n",Normal.FitKnee_3[i]);
		}
		break;
	default:
		break;
	}
//	DMA_usart2_printf("%d\r\n",Flag);
	free(q);
}

/********************************实现函数**************************************
*函数原型:	double Horner_Algorithm(int poly_n, double p[], double x)
*功　　能:	秦九韶算法
*修改日期:	20231207
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
double Horner_Algorithm(int poly_n, double p[], double x){
	double y = 0.0;	//存放多项式的值
	for (int j = poly_n; j>=0; j--)
	{
		y = (x * y) + p[j];	//计算多项式的值。
	}
	return y;
}

/********************************实现函数**************************************
*函数原型:	double Slop(int poly_n, double p[], double x)
*功　　能:	计算斜率多项式
*修改日期:	20231208
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void Slop(int poly_n, double p[], double q[]){
	for(int i = 1; i<poly_n+1; i++){
		q[i-1] = i*p[i];
	}
}

/********************************实现函数**************************************
*函数原型:	void ModbusRead()
*功　　能:	读取距离传感器数值
*修改日期:	20231010
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void ModbusRead(){
	uint8_t ModbusData[] = {0x50,0x03,0x00,0x34,0x00,0x01,0xc8,0x45};
	for(int i = 0;i<8;i++){
		HAL_UART_Transmit(&huart1,(uint8_t *)&ModbusData[i],1,0xFFFF);
	}
}

/********************************实现函数**************************************
*函数原型:	void ModbusRead()
*功　　能:	计算ModbusCRC校验值
*修改日期:	20231012
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
uint16_t crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

///********************************实现函数**************************************
//*函数原型:	low_pass_filter_init()
//*功　　能:	滤波器初始化
//*修改日期:	20231129
// * 参数		| 介绍
// * ---------+--------------------------------------
// * 无		| 无
//*******************************************************************************/
//void low_pass_filter_init(void){
//    float b = 2.0 * pi * fc * Ts;
//    alpha = b / (b + 1);
//}
//
///********************************实现函数**************************************
//*函数原型:	low_pass_filter()
//*功　　能:	滤波器
//*修改日期:	20231129
// * 参数		| 介绍
// * ---------+--------------------------------------
//* value		| 值
//*******************************************************************************/
//float low_pass_filter(float value){
//    static float out_last = 0; //上一次滤波值
//    float out;
//
//  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
//    static char fisrt_flag = 1;
//    if (fisrt_flag == 1){
//        fisrt_flag = 0;
//        out_last = value;
//    }
//
//  /*************************** 一阶滤波 *********************************/
//    out = out_last + alpha * (value - out_last);
//    out_last = out;
//
//    return out;
//}

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
