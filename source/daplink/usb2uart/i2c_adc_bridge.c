#include "i2c_adc_bridge.h"
#include "util.h"
#include "DAP_config.h"

#ifdef ENABLE_PWR_INST

I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc11;
ADC_HandleTypeDef hadc12;

HAL_StatusTypeDef Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef Read_From_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len);

#if 0 //old code for reference, todo: remove it later. 
#define I2Cx_RCC				RCC_APB1Periph_I2C1 
#define I2Cx						I2C1
#define I2C_GPIO_RCC		RCC_APB2Periph_GPIOB
#define I2C_GPIO				GPIOB

void i2c_start(void);
void i2c_stop(void);
void i2c_address_direction(uint8_t address, uint8_t direction);
void i2c_transmit(uint8_t byte);
uint8_t i2c_receive_ack(void);
uint8_t i2c_receive_nack(void);
#endif 

#define I2C_PIN_SDA			GPIO_PIN_7 
#define I2C_PIN_SCL			GPIO_PIN_6
#define SLAVE_ADDRESS		0x55 //9.5.4.1 I2C Interface (http://www.ti.com/lit/ds/symlink/bq27441-g1.pdf)

void I2C_BQ27441GiBridge_Initialize(void)
{
	#if 0 //old reference code
	// Initialization struct
	I2C_InitTypeDef I2C_InitStruct;  
	GPIO_InitTypeDef GPIO_InitStruct;
		
	// Step 2: Initialize GPIO as open drain alternate function
	RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SDA;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	// Step 1: Initialize I2C
	I2C_DeInit(I2Cx);
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx, &I2C_InitStruct);
	I2C_Cmd(I2Cx, ENABLE);
	#endif 
	
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2C_PIN_SDA | I2C_PIN_SCL;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  __HAL_AFIO_REMAP_I2C1_ENABLE();
  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* This is also required before configuring the I2C peripheral
   * in STM32F1xx devices */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  HAL_I2C_Init(&hi2c1);
}

HAL_StatusTypeDef Read_From_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len) {
  HAL_StatusTypeDef returnValue;
  uint8_t addr[2];

  /* We compute the MSB and LSB parts of the memory address */
  addr[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
  addr[1] = (uint8_t) (MemAddress & 0xFF);

  /* First we send the memory location address where start reading data */
  returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, addr, 2, HAL_MAX_DELAY);
  if(returnValue != HAL_OK)
    return returnValue;

  /* Next we can retrieve the data from device */
  returnValue = HAL_I2C_Master_Receive(hi2c, DevAddress, pData, len, HAL_MAX_DELAY);

  return returnValue;
}

HAL_StatusTypeDef Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len) {
  HAL_StatusTypeDef returnValue;
  uint8_t *data;

  /* First we allocate a temporary buffer to store the destination memory
   * address and the data to store */
  data = (uint8_t*)malloc(sizeof(uint8_t)*(len+2));

  /* We compute the MSB and LSB parts of the memory address */
  data[0] = (uint8_t) ((MemAddress & 0xFF00) >> 8);
  data[1] = (uint8_t) (MemAddress & 0xFF);

  /* And copy the content of the pData array in the temporary buffer */
  memcpy(data+2, pData, len);

  /* We are now ready to transfer the buffer over the I2C bus */
  returnValue = HAL_I2C_Master_Transmit(hi2c, DevAddress, data, len + 2, HAL_MAX_DELAY);
  if(returnValue != HAL_OK)
    return returnValue;

  free(data);

  /* We wait until the EEPROM effectively stores data in memory.
   * The technique shown in the book doesn't work here, due some
   * limitations of the I2C peripheral in STM32F1 devices. We
   * so use the dedicated HAL routine.
   */
  while(HAL_I2C_IsDeviceReady(hi2c, DevAddress, 1, HAL_MAX_DELAY) != HAL_OK);

  return HAL_OK;
}

void I2C_BQ27441GiBridge_Close(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	#if 0
	//todo
	//awar: see if there is a need to disable/reset what was originally enabled/reset during initialization
	//also see if there is a need to disable the associated GPIO too. 
  __HAL_AFIO_REMAP_I2C1_ENABLE();
  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();
	
  /* This is also required before configuring the I2C peripheral
   * in STM32F1xx devices */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
	#endif 

  HAL_I2C_DeInit(&hi2c1);
}

#if 0 
void i2c_start()
{
	// Wait until I2Cx is not busy anymore
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	
	// Generate start condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	
	// Wait for I2C EV5. 
	// It means that the start condition has been correctly released 
	// on the I2C bus (the bus is free, no other devices is communicating))
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
}

void i2c_stop()
{
	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// Wait until I2C stop condition is finished
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));
}

void i2c_address_direction(uint8_t address, uint8_t direction)
{
	// Send slave address
	I2C_Send7bitAddress(I2Cx, address, direction);
	
	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (direction == I2C_Direction_Transmitter)
	{
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if (direction == I2C_Direction_Receiver)
	{	
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void i2c_transmit(uint8_t byte)
{
	// Send data byte
	I2C_SendData(I2Cx, byte);
	// Wait for I2C EV8_2.
	// It means that the data has been physically shifted out and 
	// output on the bus)
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t i2c_receive_ack()
{
	// Enable ACK of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	
	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx);
}

uint8_t i2c_receive_nack()
{
	// Disable ACK of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	
	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx);
}

void i2c_write(uint8_t address, uint8_t data)
{
	//GPIO_WriteBit(I2C_GPIO, I2C_PIN_SDA, Bit_SET);
	//GPIO_WriteBit(I2C_GPIO, I2C_PIN_SCL, Bit_SET);
	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Transmitter);
	i2c_transmit(data);
	i2c_stop();
}

void i2c_read(uint8_t address, uint8_t* data)
{
	//GPIO_WriteBit(I2C_GPIO, I2C_PIN_SDA, Bit_RESET);
	//GPIO_WriteBit(I2C_GPIO, I2C_PIN_SCL, Bit_RESET);
	i2c_start();
	i2c_address_direction(address << 1, I2C_Direction_Receiver);
	*data = i2c_receive_nack();
	i2c_stop();
}

void I2C_BQ27441GiBridge_Close()
{
	I2C_DeInit(I2Cx);
	RCC_APB1PeriphClockCmd(I2Cx_RCC, DISABLE);
	I2C_Cmd(I2Cx, DISABLE);
	RCC_APB2PeriphClockCmd(I2C_GPIO_RCC, DISABLE);
}
#endif 

void ADC_GnssBridge_Initialize(void)
{
	#if 0 //todo: old code for reference, will be removed later. 
	// Initialization struct
	ADC_InitTypeDef ADC_InitStruct;
	
	// Step 1: Initialize ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	// Select input channel for ADC1
	// ADC1 channel 0 (PA6)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);  
	
	// Step 2: Initialize GPIOA (PA7)
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	#endif 
	
	ADC_ChannelConfTypeDef sConfig;

  /* Enable ADC peripheral */
  __HAL_RCC_ADC1_CLK_ENABLE();

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc11.Instance = ADC1;
  hadc11.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc11.Init.ContinuousConvMode = ENABLE;
  hadc11.Init.DiscontinuousConvMode = DISABLE;
  hadc11.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc11.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc11.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc11);

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc11, &sConfig);
	
	HAL_ADCEx_Calibration_Start(&hadc11);
  HAL_ADC_Start(&hadc11);
	
	// Step 2: Initialize GPIOA (PA7)
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint16_t ADC_GnssBridge_GetCurrentValue(void)
{
	#if 0 //old code for reference only. 
	// Start ADC conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	// Wait until ADC conversion finished
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
	#endif 
	
	HAL_ADC_PollForConversion(&hadc11, HAL_MAX_DELAY);
  return HAL_ADC_GetValue(&hadc11);
}

void ADC_GnssBridge_Close(void)
{
	#if 0 // todo: see how to properly close ADC interface via HAL. 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);	
	ADC_Cmd(ADC1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
	#endif 
	
	//todo: see if it is enough or i shall need to disable GPIO as well 
	 __HAL_RCC_ADC1_CLK_DISABLE();
}

void ADC_CellularBridge_Initialize(void)
{
	#if 0 //todo: old code for reference, will be removed later. 
	// Initialization struct
	ADC_InitTypeDef ADC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// Step 1: Initialize ADC2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC2, &ADC_InitStruct);
	ADC_Cmd(ADC2, ENABLE);
	// Select input channel for ADC2
	// ADC1 channel 0 (PA6)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_55Cycles5);  
	
	// Step 2: Initialize GPIOA (PA6)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	#endif
	
	ADC_ChannelConfTypeDef sConfig;

  /* Enable ADC peripheral */
  __HAL_RCC_ADC2_CLK_ENABLE();

  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc12.Instance = ADC2;
  hadc12.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc12.Init.ContinuousConvMode = ENABLE;
  hadc12.Init.DiscontinuousConvMode = DISABLE;
  hadc12.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc12.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc12.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc12);

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc12, &sConfig);
	
	HAL_ADCEx_Calibration_Start(&hadc12);
  HAL_ADC_Start(&hadc12);
	
	// Step 2: Initialize GPIOA (PA6)
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint16_t ADC_CellularBridge_GetCurrentValue(void)
{
	#if 0 //old code for reference only. 
	// Start ADC conversion
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	// Wait until ADC conversion finished
	while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));

	return ADC_GetConversionValue(ADC2);
	#endif 
	
  HAL_ADC_PollForConversion(&hadc12, HAL_MAX_DELAY);
  return HAL_ADC_GetValue(&hadc12);
}

void ADC_CellularBridge_Close(void)
{
	#if 0 // todo: see how to properly close ADC interface via HAL. 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, DISABLE);	
	ADC_Cmd(ADC2, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
	#endif 
	
	//todo: see if it is enough or i shall need to disable GPIO as well 
	__HAL_RCC_ADC2_CLK_DISABLE();
}

#endif //ENABLE_PWR_INST
