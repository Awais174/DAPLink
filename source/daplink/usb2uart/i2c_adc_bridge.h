/**
 * @file    seriah.h
 * @brief   Interface for serial driver
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef I2C_ADC_H
#define I2C_ADC_H

//stm hal
#include "stm32f1xx_hal.h"
//daplink libs
#include "RTL.h"
#include "rl_usb.h"
#include "main.h"
#include <util.h>
//system libs
#include "string.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

	//todo: make it generic 
	void I2C_BQ27441GiBridge_Initialize(void);
	void I2C_BQ27441GiBridge_Close(void);

	#if 0 //old code for reference. 
	//these ones are fine 	
	void I2C_write(uint8_t address, uint8_t data);
	void I2C_read(uint8_t address, uint8_t* data);
	#endif 
	
	//todo: make these generic
	void ADC_GnssBridge_Initialize(void);
	uint16_t ADC_GnssBridge_GetCurrentValue(void);
	void ADC_GnssBridge_Close(void);
	void ADC_CellularBridge_Initialize(void);
	uint16_t ADC_CellularBridge_GetCurrentValue(void);
	void ADC_CellularBridge_Close(void);
	
#ifdef __cplusplus
}
#endif

#endif 
