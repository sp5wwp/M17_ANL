
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "radio_config.h"

#define	FRAMESIZE		160*2					//20+20=40ms frame
#define	RAW_BYTES		8*2
#define	CRC_LEN			2
#define	ENC_LEN			(RAW_BYTES)*2
#define	PLOAD_LEN		93

#define RCVD_BUFF_LEN	110
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//UART RELATED
volatile uint8_t    rcvd[RCVD_BUFF_LEN];           //receiving buffer
volatile uint8_t    rcv_cnt=0;          			//received bytes counter
volatile uint8_t    extr_val[12];          			//for extracted value
volatile uint8_t    extr_val_len=0;     			//number of digits
volatile uint32_t   val=0;              			//value
volatile uint8_t    first_tim_int=1;    			//first timer interrupt? for uart rcv timeout

volatile uint8_t    tx_pwr=0;
volatile uint32_t   trx_freq=0;

//Si 446x RELATED
static const uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
volatile uint8_t ints[8];					//interrupts

//RADIO STATE
volatile uint8_t r_initd=0;					//initialized?

//RCV FIELDS - EXTRACTED FROM RECEIVED DATA
volatile uint8_t rcv_buff[PLOAD_LEN];
volatile uint8_t rcv_voice[RAW_BYTES];

//TX-RX
volatile uint8_t first_rx=1;						//is this the first TX->RX switchover?
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//------------------------------SPI - Si4463------------------------------
/*void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
}*/

/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
}*/

void SPI_WaitForCTS(void)
{
	uint8_t cts[1]={0x00};
	const uint8_t dta[1]={0x44};

	do
	{
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
		HAL_SPI_Transmit(&hspi1, dta, 1, 100);
		HAL_SPI_Receive(&hspi1, cts, 1, 100);
		//if(cts[0]!=0xFF)
			HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
	}while(cts[0]!=0xFF);
}

void SPI_Send(uint8_t *data, uint8_t len)
{
	SPI_WaitForCTS();
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
	HAL_SPI_Transmit(&hspi1, data, len, 100);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
}

void SPI_GetResponse(uint8_t len, uint8_t *data)
{
	uint8_t cts[1]={0x00};
	uint8_t dta[1]={0x44};

	while(cts[0]!=0xFF)
	{
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
		HAL_SPI_Transmit(&hspi1, dta, 1, 100);
		HAL_SPI_Receive(&hspi1, cts, 1, 1);

		if(cts[0]!=0xFF)
			HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
	}

	HAL_SPI_Receive(&hspi1, data, len, 100);

	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
}

void SPI_ReadRxDataBuff(uint8_t len, uint8_t *data)
{
	uint8_t cmd[1]={0x77};

	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
	HAL_SPI_Transmit(&hspi1, cmd, 1, 100);
	HAL_SPI_Receive(&hspi1, data, len, 100);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
}

//------------------------------Si446x FUNCS------------------------------
void Si_Reset(void)
{
	HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, 0);
	HAL_Delay(5);
}

static void Si_SetProp(uint8_t* vals, uint8_t group, uint8_t number, uint8_t len)
{
	uint8_t data[16]={0x11,	group, len, number};

	memcpy(data + 4, vals, len);

	SPI_WaitForCTS();
	SPI_Send(data, len + 4);
}

static void Si_StartupConfig(void)
{
	uint8_t buff[17];

	for(uint16_t i=0; i<sizeof(config); i++)
	{
		memcpy(buff, &config[i], sizeof(buff));
		SPI_WaitForCTS();
		SPI_Send(&buff[1], buff[0]);
		i += buff[0];
	}
}

static void Si_Interrupts(void* buff)
{
	uint8_t data = 0x20;

	SPI_Send(&data, 1);
	if(buff!=NULL)
		SPI_GetResponse(8, buff);
}

/*static void Si_InterruptsNoCTS(void* buff)
{
	uint8_t v = 0x20;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
	HAL_SPI_Transmit(&hspi2, &v, 1, 100);
	HAL_SPI_Receive(&hspi2, buff, 8, 100);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
}*/

static void Si_Interrupts2(void* buff, uint8_t clearPH, uint8_t clearMODEM, uint8_t clearCHIP)
{
	uint8_t data[] = {
		0x20,
		clearPH,
		clearMODEM,
		clearCHIP
	};

	SPI_Send(data, 4);
	if(buff!=NULL)
		SPI_GetResponse(8, buff);
}

static uint8_t Si_GetState(void)
{
	uint8_t state[2]={0, 0};
	uint8_t get_state_cmd=0x33;

	SPI_WaitForCTS();
	SPI_Send(&get_state_cmd, 1);
	SPI_GetResponse(2, state);

	return state[0]&0x0F;
}

static void Si_SetState(uint8_t newState)
{
	uint8_t data[2]={0x34, newState};

	SPI_Send(data, 2);
}

void Si_Sleep(void)
{
	uint8_t state=Si_GetState();

	if(state==7 || state==5)
		return;

	Si_SetState(1);
}

void Si_SetTxPower(uint8_t pwr)
{
	/*uint8_t msg[5]={0x11, 0x22, 0x01, 0x01, pwr};

	SPI_Send(&msg, 5);*/
	Si_SetProp(&pwr, 0x22, 0x01, 1);
}

void Si_ClearFIFO(uint8_t fifos)
{
	uint8_t msg[3]={0x15, fifos};

	SPI_Send(&msg, 2);
}

void Si_WriteTxDataBuff(uint8_t *data, uint8_t len)
{
	uint8_t cmd = 0x66;

	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, len, 100);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);
}

void Si_StartTx(uint8_t ch, uint8_t end_state, uint8_t tx_len)
{
	uint8_t msg[7]={0x31, ch, (uint8_t)end_state<<4, 0, tx_len, 0, 0};

	SPI_WaitForCTS();
	SPI_Send(&msg, 7);
}

void Si_StartRx(uint8_t ch, uint8_t len)
{
	//Si_SetState(3);		//RX state
	Si_ClearFIFO(3);	//clear both FIFOs
	//Si_Interrupts2(NULL, 0, 0, 0xFF);	//clear some interrupts

	uint8_t msg[8]={0x32, ch, 0, 0, len, 0x08, 0x08, 0x08};

	SPI_Send(msg, 8);
}

void Si_TxData(uint8_t *d, uint8_t len, uint8_t ch)
{
	Si_ClearFIFO(3);
	Si_Interrupts2(NULL, 0, 0, 0xFF);

	Si_WriteTxDataBuff(d, len);
	Si_StartTx(ch, 3, len);	//3 - READY state after TX complete
}

void Si_FreqSet(uint32_t freq)	//freq in Hz
{
	uint8_t inte=0x3C;	//default values
	uint64_t frac=0x00080000;

	freq=freq*(30.0/32.0);
	inte=freq/7500000-1;
	frac=(freq-(uint32_t)(inte+1)*7500000)/75;
	frac=(uint64_t)frac*(1<<19)/100000+(uint32_t)(1<<19);

	uint8_t vals[4]={inte, (frac>>16)&0xFF, (frac>>8)&0xFF, frac&0xFF};

	Si_SetProp(&vals, 0x40, 0x00, 4);
}

uint8_t Si_GetRSSI(void)
{
	uint8_t v=0x50;
	uint8_t r[4];

	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
	HAL_SPI_Transmit(&hspi1, &v, 1, 100);
	HAL_SPI_Receive(&hspi1, r, 4, 100);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 1);

	return r[0];
}

void Si_GetInfo(uint8_t *resp)
{
	uint8_t cmd=0x01;

	SPI_Send(&cmd, 1);
	SPI_WaitForCTS();
	SPI_GetResponse(8, resp);
}

uint8_t checkTRX(void)
{
	uint8_t info[8];

	Si_GetInfo(info);

	if(info[1]==0x44 && info[2]==0x63)
		return 0;

	return 1;
}

//------------------------------RF SWITCH FUNCS------------------------------
static void RF_SetTX(void)
{
	HAL_GPIO_WritePin(RX_SW_GPIO_Port, RX_SW_Pin, 0);
	HAL_GPIO_WritePin(TX_SW_GPIO_Port, TX_SW_Pin, 1);
	//r_tx=1;
}

static void RF_SetRX(void)
{
	HAL_GPIO_WritePin(RX_SW_GPIO_Port, RX_SW_Pin, 1);
	HAL_GPIO_WritePin(TX_SW_GPIO_Port, TX_SW_Pin, 0);
	//r_tx=0;
}
//--------------------------INTERRUPTS----------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Si4463 TRX module interrupt
	if(GPIO_Pin==GPIO_PIN_8 && r_initd)
	{
		//Si nIRQ interrupt request
		Si_Interrupts(ints);	//check pending interrupt flags

		//PACKET_RX_PEND flag set?
		if(ints[2]&(1<<4))
		{
			//retrieve RX buff contents
			SPI_ReadRxDataBuff(PLOAD_LEN, rcv_buff);
			Si_ClearFIFO(3);

			//scanner mode
			HAL_UART_Transmit(&huart2, "AT+FRAME=", 9, 100);
			HAL_UART_Transmit(&huart2, rcv_buff, PLOAD_LEN, 100);
			HAL_UART_Transmit(&huart2, "\r\n", 2, 100);
		}

		//clear pending flags
		Si_Interrupts(NULL);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rcvd[0]=='A' && rcvd[1]=='T' && rcvd[2]=='+')
	{
		if(rcvd[3]=='F' && rcvd[4]=='R')
		{
			if(rcvd[5]=='A' && rcvd[6]=='M' && rcvd[7]=='E' && rcvd[PLOAD_LEN+9]=='\r' && rcvd[PLOAD_LEN+10]=='\n')
			{
				RF_SetTX();
				memcpy(rcv_buff, &rcvd[9], PLOAD_LEN);
				Si_TxData(rcv_buff, PLOAD_LEN, 0);
				HAL_TIM_Base_Start_IT(&htim21);
				HAL_UART_Transmit(&huart2, "OK\r\n", 4, 100);
				memset(rcvd, 0, sizeof(rcvd));
				rcv_cnt=0;
				HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
			}
			else if(rcvd[5]=='E' && rcvd[6]=='Q')
			{
				if(rcvd[7]=='?' && rcvd[8]=='\r' &&  rcvd[9]=='\n')
				{
					uint8_t resp[22];
					sprintf(resp, "FREQ=%lu\r\n", trx_freq);
					HAL_UART_Transmit(&huart2, resp, strlen(resp), 100);
					memset(rcvd, 0, sizeof(rcvd));
					rcv_cnt=0;
					HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
				}
				else if(rcvd[7]=='=' && rcvd[17]=='\r' &&  rcvd[18]=='\n')
				{
					for(uint8_t i=8; i<20; i++)
					{
						if((rcvd[i]<'0' || rcvd[i]>'9') && (rcvd[i]!='\r' && rcvd[i]!='\n'))
						{
							extr_val_len=0;
							break;
						}
						if(rcvd[i]=='\r')
						{
							extr_val_len=i-8;
							break;
						}
					}
					if(extr_val_len==9)
					{
						uint32_t mult=1;
						for(int8_t i=extr_val_len-1; i>=0; i--)
						{
							val+=(rcvd[8+i]-'0')*mult;
							mult*=10;
						}
						trx_freq=val;
						Si_FreqSet(trx_freq);
						uint8_t resp[22];
						sprintf(resp, "FREQ=%lu\r\n", val);
						HAL_UART_Transmit(&huart2, resp, strlen(resp), 100);
					}
					else
					{
						HAL_UART_Transmit(&huart2, "ERROR\r\n", 7, 100);
					}

					val=0;
					memset(rcvd, 0, sizeof(rcvd));
					rcv_cnt=0;
					HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
				}
			}
		}
		else if(rcvd[3]=='P' && rcvd[4]=='W' && rcvd[5]=='R')
		{
			if(rcvd[6]=='?')
			{
				if(rcvd[7]=='\r' && rcvd[8]=='\n')
				{
					uint8_t resp[22];
					sprintf(resp, "PWR=%u\r\n", tx_pwr);
					HAL_UART_Transmit(&huart2, resp, strlen(resp), 100);
					memset(rcvd, 0, sizeof(rcvd));
					rcv_cnt=0;
					HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
				}
			}
			else if(rcvd[6]=='=' && (rcvd[8]=='\r' || rcvd[9]=='\r' || rcvd[10]=='\r'))
			{
				for(uint8_t i=7; i<15; i++)
				{
					if((rcvd[i]<'0' || rcvd[i]>'9') && (rcvd[i]!='\r' && rcvd[i]!='\n'))
					{
						extr_val_len=0;
						break;
					}
					if(rcvd[i]=='\r')
					{
						extr_val_len=i-7;
						break;
					}
				}
				if(extr_val_len!=0)
				{
					uint32_t mult=1;
					for(int8_t i=extr_val_len-1; i>=0; i--)
					{
						val+=(rcvd[7+i]-'0')*mult;
						mult*=10;
					}
					tx_pwr=val;
					Si_SetTxPower(tx_pwr);
					uint8_t resp[22];
					sprintf(resp, "PWR=%u\r\n", val);
					HAL_UART_Transmit(&huart2, resp, strlen(resp), 100);
				}
				else
				{
					HAL_UART_Transmit(&huart2, "ERROR\r\n", 7, 100);
				}

				val=0;
				memset(rcvd, 0, sizeof(rcvd));
				rcv_cnt=0;
				HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
			}
		}
	}

	rcv_cnt++;
	HAL_UART_Receive_IT(&huart2, &rcvd[rcv_cnt], 1);
	TIM2->CNT=0;
	HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)	//10ms UART receive timeout
	{
		if(first_tim_int)
			first_tim_int=0;
		else
		{
			memset(rcvd, 0, sizeof(rcvd));
			rcv_cnt=0;
			HAL_UART_AbortReceive_IT(&huart2);
			HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
			HAL_TIM_Base_Stop_IT(&htim2);
			TIM2->CNT=0;
		}
	}
	else if(htim->Instance==TIM21)	//30ms after starting TX switch back to RX mode
	{
		if(first_rx)
		{
			first_rx=0;
		}
		else
		{
			Si_StartRx(0, PLOAD_LEN);
			HAL_TIM_Base_Stop_IT(&htim21);
			TIM21->CNT=0;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  memset(rcvd, 0, RCVD_BUFF_LEN);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, 0);
  RF_SetRX();
  HAL_Delay(500);
  Si_Reset();
  if(checkTRX())	//fucked up comms with Si4463?
  {
	  HAL_UART_Transmit(&huart2, "M17_ANL INIT ERR\r\n", 17, 100);
  	  while(1);
  }

  Si_StartupConfig();
  Si_Interrupts(NULL);
  Si_Sleep();
  trx_freq=439575000;
  Si_FreqSet(trx_freq);
  Si_SetTxPower(tx_pwr);
  Si_StartRx(0, PLOAD_LEN);
  r_initd=1;	//we need this to avoid getting Si IRQ request right after power-up sequence

  HAL_UART_Transmit(&huart2, "M17_ANL INIT OK\r\n", 17, 100);
  HAL_UART_Receive_IT(&huart2, &rcvd[0], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_16;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 31999;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 29;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RX_SW_Pin|NSEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_SW_GPIO_Port, TX_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRX_SDN_GPIO_Port, TRX_SDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RX_SW_Pin NSEL_Pin */
  GPIO_InitStruct.Pin = RX_SW_Pin|NSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_SW_Pin TRX_SDN_Pin */
  GPIO_InitStruct.Pin = TX_SW_Pin|TRX_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NIRQ_Pin */
  GPIO_InitStruct.Pin = NIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NIRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
