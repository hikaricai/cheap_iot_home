#include "main.h"
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM3_CLK_ENABLE()


/* Definition for TIMx's NVIC */
TIM_HandleTypeDef    TimHandle;
uint32_t uwPrescalerValue = 0;
uint32_t g_count = 0;
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}
static GPIO_PinState dht11_read()
{
  return HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_2);
}
static void dht11_write(GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2,PinState);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripheral clock #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();
  
//  /*##-2- Configure the NVIC for TIMx ########################################*/
//  /* Set the TIMx priority */
//  HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);

//  /* Enable the TIMx global Interrupt */
//  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}
void dht11_time_init()
{
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 900000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 0xFFFF;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  BSP_LED_Toggle(LED1);
}
void delay_us(int32_t count)
{
  
  HAL_TIM_Base_Start(&TimHandle);
  while(count >= TimHandle.Instance->CNT);
  HAL_TIM_Base_Stop(&TimHandle);
  TimHandle.Instance->CNT = 0;
}
static void DHT11_SET_IO_IN()
{
  GPIO_InitTypeDef gpio_init_structure;
  __HAL_RCC_GPIOI_CLK_ENABLE();
  gpio_init_structure.Pin = GPIO_PIN_2;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOI, &gpio_init_structure);
}
static void DHT11_SET_IO_OUT()
{
  GPIO_InitTypeDef  gpio_init_structure;
  /* Enable the GPIO_LED clock */
  LED1_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN_2;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOI, &gpio_init_structure);
}
void DHT11_Prepare(void)
{
  DHT11_SET_IO_OUT(); 
  dht11_write(0); 
  //HAL_Delay(20); 
}
static void DHT11_Rst(void)   
{
                    
  dht11_write(1); 
  delay_us(30);     
 
}

static uint8_t DHT11_Check(void)    
{
  uint8_t retry=0;
  DHT11_SET_IO_IN();//SET INPUT 
  while (dht11_read()&&retry<100)
  {
    retry++;
    delay_us(1);
  }; 
  if(retry>=100)
    return 1;
  else 
    retry=0;
  while (!dht11_read()&&retry<100)
  {
    retry++;
    delay_us(1);
  };
  if(retry>=100)
    return 1;    
  return 0;
}

uint8_t DHT11_Read_Bit(void)  
{
 
  uint8_t retry=0;
  while(dht11_read()&&retry<100)
  {
    retry++;
    delay_us(1);
  }
  retry=0;
  while(!dht11_read()&&retry<100)
  {
   
  retry++;
  delay_us(1);
   
  }
  delay_us(40);
  if(dht11_read())
    return 1;
  else 
    return 0;   
 
}
uint8_t DHT11_Read_Byte(void)    
{
  uint8_t i,dat;
  dat=0;
  for (i=0;i<8;i++) 
  {
    dat<<=1; 
    dat|=DHT11_Read_Bit();
  }    
  return dat;
}

uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi)    
{
         
  uint8_t buf[5];
  uint8_t i;
  DHT11_Rst();
  if(DHT11_Check()==0)
  {
   
    for(i=0;i<5;i++)
    {
     
      buf[i]=DHT11_Read_Byte();
     
    }
    if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
    {
     
      *humi=buf[0];
      *temp=buf[2];
      return 0;  
    }
 
  } 

  return 1;
    
 
}

uint8_t DHT11_Init(void)
{
  DHT11_Rst();
  return DHT11_Check();
 
}
