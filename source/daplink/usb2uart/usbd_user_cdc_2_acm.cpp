#ifdef ENABLE_PWR_INST
#include "i2c_adc_bridge.h"
#endif 

#ifdef ENABLE_2ND_COM_PORT
#define SIZE_DATA (64)
static uint8_t recvCmd[SIZE_DATA] = {};
#define I2C_CONFIG_SZ 				11
#define I2C_READ_SZ 					9
#define I2C_CLOSE_SZ 					10
#define ADC_CONFIG_SZ 				15
#define ADC_CLOSE_SZ 					14
#define MAX_ADC_READING_SIZE  4
#define WARN_MSG_SIZE					31

static void trasmit_ADC_Reading(uint16_t adcReading, bool isGNSS);	
static uint8_t recvCmd_I2C_config[I2C_CONFIG_SZ] 					= "I2C_CONFIG";
static uint8_t recvCmd_I2C_read[I2C_READ_SZ] 							= "I2C_READ";     
static uint8_t recvCmd_I2C_close[I2C_CLOSE_SZ] 						= "I2C_CLOSE";   
static uint8_t recvCmd_ADC_gnss_config[ADC_CONFIG_SZ] 		= "ADC_GNSS_CONFIG"; 
static uint8_t recvCmd_ADC_gnss_close[ADC_CLOSE_SZ] 			= "ADC_GNSS_CLOSE";   
static uint8_t recvCmd_ADC_cell_config[ADC_CONFIG_SZ] 		= "ADC_CELL_CONFIG"; 
static uint8_t recvCmd_ADC_cell_close[ADC_CLOSE_SZ] 			= "ADC_CELL_CLOSE";   

bool i2c_enable = false;
bool adc_gnss_enable = false;
bool adc_cell_enable = false;
	
static uint8_t counter = 0;

void I2C_ProcessCommand(uint8_t *recvCmd)
{
	int32_t len_data = 0;

	if(i2c_enable != true && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_I2C_config, I2C_CONFIG_SZ-1) == 0))			
	{
		#ifdef ENABLE_PWR_INST	
		I2C_BQ27441GiBridge_Initialize();
		#endif 
		
		i2c_enable = true;
		len_data = USBD_CDC_2_ACM_DataFree(); 

		if (len_data > SIZE_DATA) 	
		{	
			len_data = SIZE_DATA;	
		}

		if (len_data) 	
		{		
			uint8_t sendCmd_I2C_config_success[19] = "I2C_CONFIG_SUCCESS";	
			USBD_CDC_2_ACM_DataSend(sendCmd_I2C_config_success, 19);			
		}	
	}	
	else if(i2c_enable == true && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_I2C_read, I2C_READ_SZ-1) == 0))
	{		
		uint8_t responseBuff[2];
		
		#ifdef ENABLE_PWR_INST	
		//test code for verification; eventually only read/write shall be used by user via PC
		uint8_t data[3];
		data[0] = 0x00;  /* Set address to first register for control */
		data[1] = 0x02;  /* First byte of FW_VERSION sub-command (0x02) */
		data[2] = 0x00;  /* Second byte of FW_VERSION sub-command (0x00) */
		
		if(counter == 0)
		{
			//i2c_write(0x55, data[0]);
			//Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len)
		}
		
		if(counter == 1)
		{
			//i2c_write(0x55, data[1]);
			//Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len)
		}
		
		if(counter == 2)
		{
			//i2c_write(0x55, data[2]);
		  //Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len)
		}
		
		if(counter == 3)
		{
			//i2c_write(0x55, 0x00);
			//i2c_read(0x55, &responseBuff[0]);
			//i2c_read(0x55, &responseBuff[1]);
			//rather use
			//Write_To_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len)
			//Read_From_I2C1(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t len) 
			
			//responseBuff[0] = (uint8_t)((responseBuff[0]% 10) + '0');
			//responseBuff[1] = (uint8_t)((responseBuff[1]% 10) + '0');
			
			uint16_t pBytes = ((uint16_t) (responseBuff[1] << 8)) + responseBuff[0];
			
			if (((pBytes >> 8) == 0x01) && ((pBytes & 0xff) == 0x09)) {
				
				util_assert(0);
			}
			
			len_data = USBD_CDC_2_ACM_DataFree(); 

			if (len_data > SIZE_DATA)	{
				len_data = SIZE_DATA;
			}
			if (len_data)	{
				//uint8_t sendCmd_I2C_read_data[20] = "i2c_dummy_rsp \n";
				//USBD_CDC_2_ACM_DataSend(sendCmd_I2C_read_data, 20);		
				//USBD_CDC_2_ACM_DataSend(responseBuff, 2);	
				
				trasmit_ADC_Reading(pBytes, true);
			}	
		}
		#endif /* ENABLE_PWR_INST */
		
		counter++;
		
		if(counter > 3)
		{
			counter = 0;
		}
	}
	else if(i2c_enable == true && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_I2C_close, I2C_CLOSE_SZ-1) == 0))
	{
		#ifdef ENABLE_PWR_INST								
		I2C_BQ27441GiBridge_Close();
		#endif 
		i2c_enable = false;
	}
}

static void trasmit_ADC_Reading(uint16_t adcReading, bool isGNSS)
{		
	//ensure that ADC reading is received in 12bit resolution 
	if(adcReading <= 4095)	{
		uint8_t dataString[MAX_ADC_READING_SIZE + 6];
		
		if(isGNSS == true)	{
			//eventually to make it gnss:xxxx|
			dataString[0] = 'g';
			dataString[1] = 'n';
			dataString[2] = 's';
			dataString[3] = 's';
			dataString[4] = ':';
		}
		else	{
			//eventually to make it cell:xxxx|
			dataString[0] = 'c';
			dataString[1] = 'e';
			dataString[2] = 'l';
			dataString[3] = 'l';
			dataString[4] = ':';					
		}	
		
		uint8_t temp[MAX_ADC_READING_SIZE];
		int8_t i = 0;
		uint8_t j = 0;
					
		do	{			
			temp[i++] = (uint8_t)(adcReading % 10) + '0'; //convert integer to character
			adcReading /= 10;
			
		} while(adcReading);
		
		//append actaul adc reading after the format string 
		while(i > 0)	{			
			dataString[5+j] = temp[i-1];
			j++;
			i--;
		}
		
		//append the terminator at the end of relative string 
		dataString[5+j] = '|';			
		//send the total number of bytes 	
		USBD_CDC_2_ACM_DataSend(dataString, 5+j+1);	
	}		
	else	{		
		//util_assert(0);	
	}
}

void ADC_ProcessCommand(uint8_t *recvCmd)
{
	if(adc_gnss_enable == false && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_ADC_gnss_config, ADC_CONFIG_SZ-1) == 0))			
	{			
		#ifdef ENABLE_PWR_INST								
		ADC_GnssBridge_Initialize();							
		#endif 	
		adc_gnss_enable = true;								
	}
	else if(adc_gnss_enable == true && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_ADC_gnss_close, ADC_CLOSE_SZ-1) == 0))
	{
		#ifdef ENABLE_PWR_INST								
		ADC_GnssBridge_Close();
		#endif 
		adc_gnss_enable = false;
	}
	else if(adc_cell_enable == false && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_ADC_cell_config, ADC_CONFIG_SZ-1) == 0))
	{
		#ifdef ENABLE_PWR_INST								
		ADC_CellularBridge_Initialize();							
		#endif 	
		adc_cell_enable = true;									
	}
	else if(adc_cell_enable == true && (strncasecmp((const char *) recvCmd, (const char *) recvCmd_ADC_cell_close, ADC_CLOSE_SZ-1) == 0))
	{
		#ifdef ENABLE_PWR_INST								
		ADC_CellularBridge_Close();
		#endif 
		adc_cell_enable = false; 
	}
}

void cdc_process_event_2_pwr_inst()
{	
	int32_t len_data = 0;
	len_data = USBD_CDC_2_ACM_DataRead(recvCmd, SIZE_DATA);
	
	if(len_data) {							/* modified this to enable/disbale 
																I2C/ADC data transmittion */ 
		I2C_ProcessCommand(recvCmd);
		ADC_ProcessCommand(recvCmd);
		
		#if 0
		else
		{
			len_data = USBD_CDC_2_ACM_DataFree(); 
			if (len_data > SIZE_DATA) {
			len_data = SIZE_DATA;
			}

			if (len_data >= WARN_MSG_SIZE) {
				uint8_t sendCmd_warning[WARN_MSG_SIZE] = "WARN: RECEIVED INVALID COMMAND";
				USBD_CDC_2_ACM_DataSend(sendCmd_warning, WARN_MSG_SIZE);		
			}
		}
		#endif 
	}
		
	if(adc_gnss_enable == true)	{
		uint16_t gnssCurrnetVal = 0;
		len_data = USBD_CDC_2_ACM_DataFree(); 
		
		if (len_data > SIZE_DATA)	{	
			len_data = SIZE_DATA;
		}

		if (len_data >= MAX_ADC_READING_SIZE + 6)	{							/* check if there is enough capacity 
																															in buffer to transmit	the maximum 
																															possible ADC reading */ 
			#ifdef ENABLE_PWR_INST	
			gnssCurrnetVal = ADC_GnssBridge_GetCurrentValue();	
			#endif 							
			trasmit_ADC_Reading(gnssCurrnetVal, true);
		}
	}
		
	if(adc_cell_enable == true)	{
		uint16_t cellularCurrnetVal = 0;
		len_data = USBD_CDC_2_ACM_DataFree(); 
		
		if (len_data > SIZE_DATA)	{	
			len_data = SIZE_DATA;	
		}

		if (len_data >= MAX_ADC_READING_SIZE + 6) {	
			#ifdef ENABLE_PWR_INST	
			cellularCurrnetVal = ADC_CellularBridge_GetCurrentValue();	
			#endif
			trasmit_ADC_Reading(cellularCurrnetVal, false);	
		}
	}
}
#endif /* ENABLE_2ND_COM_PORT */
