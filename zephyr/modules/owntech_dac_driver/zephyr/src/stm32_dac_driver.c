/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @author  Clément Foucher <clement.foucher@laas.fr>
 */


// Zephyr
#include <zephyr.h>

// STM32 LL
#include <stm32_ll_dac.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>

// Current file header
#include "stm32_dac_driver.h"


/////
// Init function

static int dac_stm32_init(const struct device* dev)
{
	DAC_TypeDef* dac_dev = ((struct stm32_dac_driver_data*)dev->data)->dac_struct;

	if (dac_dev == DAC1)
	{
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);
	}
	else if (dac_dev == DAC2)
	{
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC2);
	}
	else if (dac_dev == DAC3)
	{
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC3);
	}

	return 0;
}


/////
// API

static const struct dac_driver_api dac_funcs =
{
	.setconstvalue = dac_stm32_set_const_value,
	.setfunction   = dac_stm32_set_function,
	.pinconfigure  = dac_stm32_pin_configure,
	.start         = dac_stm32_start,
	.stop          = dac_stm32_stop
};

static void dac_stm32_set_const_value(const struct device* dev, uint8_t channel, uint32_t value)
{
	struct stm32_dac_driver_data* data = (struct stm32_dac_driver_data*)dev->data;
	DAC_TypeDef* dac_dev = data->dac_struct;

	uint8_t dac_channel = __LL_DAC_DECIMAL_NB_TO_CHANNEL(channel);

	if (data->dac_mode != dac_mode_constant)
	{
		data->dac_mode = dac_mode_constant;

		LL_DAC_SetSignedFormat(dac_dev, dac_channel, LL_DAC_SIGNED_FORMAT_DISABLE);

		LL_DAC_SetWaveAutoGeneration(dac_dev, dac_channel, LL_DAC_WAVE_AUTO_GENERATION_NONE);

		LL_DAC_DisableTrigger(dac_dev, dac_channel);
		LL_DAC_DisableDMADoubleDataMode(dac_dev, dac_channel);
	}

	LL_DAC_ConvertData12RightAligned(dac_dev, dac_channel, value);
}

static void dac_stm32_set_function(const struct device* dev, uint8_t channel, const dac_function_config_t* function_config)
{
	struct stm32_dac_driver_data* data = (struct stm32_dac_driver_data*)dev->data;
	DAC_TypeDef* dac_dev = data->dac_struct;

	uint8_t dac_channel = __LL_DAC_DECIMAL_NB_TO_CHANNEL(channel);

	data->dac_mode = dac_mode_function;

	if (function_config->dac_function == dac_function_sawtooth)
	{
		LL_DAC_SetSignedFormat(dac_dev, dac_channel, LL_DAC_SIGNED_FORMAT_DISABLE);

		LL_DAC_SetWaveAutoGeneration(dac_dev, dac_channel, LL_DAC_WAVE_AUTO_GENERATION_SAWTOOTH);
		LL_DAC_SetWaveSawtoothResetTriggerSource(dac_dev, dac_channel, function_config->trigger_source);
		LL_DAC_SetWaveSawtoothStepTriggerSource(dac_dev, dac_channel, function_config->step_trigger_source);
		LL_DAC_SetWaveSawtoothPolarity(dac_dev, dac_channel, function_config->polarity);
		LL_DAC_SetWaveSawtoothResetData(dac_dev, dac_channel, function_config->reset_data);
		LL_DAC_SetWaveSawtoothStepData(dac_dev, dac_channel, function_config->step_data);

		LL_DAC_EnableTrigger(dac_dev, dac_channel);
		LL_DAC_DisableDMADoubleDataMode(dac_dev, dac_channel);
	}
}

static void dac_stm32_pin_configure(const struct device* dev, uint8_t channel, const dac_pin_config_t* pin_config)
{
	struct stm32_dac_driver_data* data = (struct stm32_dac_driver_data*)dev->data;
	DAC_TypeDef* dac_dev = data->dac_struct;

	uint8_t dac_channel = __LL_DAC_DECIMAL_NB_TO_CHANNEL(channel);

	LL_DAC_ConfigOutput(dac_dev, dac_channel, LL_DAC_OUTPUT_MODE_NORMAL, pin_config->pin_buffer_enable, pin_config->pin_connect);
}

static void dac_stm32_start(const struct device* dev, uint8_t channel)
{
	struct stm32_dac_driver_data* data = (struct stm32_dac_driver_data*)dev->data;
	DAC_TypeDef* dac_dev = data->dac_struct;

	uint8_t dac_channel = __LL_DAC_DECIMAL_NB_TO_CHANNEL(channel);

	LL_DAC_Enable(dac_dev, dac_channel);

	while (LL_DAC_IsReady(dac_dev, dac_channel) == 0)
	{
		// Wait
	}
}

static void dac_stm32_stop(const struct device* dev, uint8_t channel)
{
	struct stm32_dac_driver_data* data = (struct stm32_dac_driver_data*)dev->data;
	DAC_TypeDef* dac_dev = data->dac_struct;

	uint8_t dac_channel = __LL_DAC_DECIMAL_NB_TO_CHANNEL(channel);

	LL_DAC_Disable(dac_dev, dac_channel);
}


/////
// Device definitions

// DAC 1
#if DT_NODE_HAS_STATUS(DAC1_NODELABEL, okay)

struct stm32_dac_driver_data dac1_data =
{
	.dac_struct = DAC1,
	.dac_mode   = dac_mode_unset
};

DEVICE_DT_DEFINE(DAC1_NODELABEL,
                 dac_stm32_init,
                 device_pm_control_nop,
                 &dac1_data,
                 NULL,
                 PRE_KERNEL_1,
                 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                 &dac_funcs
                );

#endif // DAC 1

// DAC 2
#if DT_NODE_HAS_STATUS(DAC2_NODELABEL, okay)

struct stm32_dac_driver_data dac2_data =
{
	.dac_struct = DAC2,
	.dac_mode   = dac_mode_unset
};

DEVICE_DT_DEFINE(DAC2_NODELABEL,
                 dac_stm32_init,
                 device_pm_control_nop,
                 &dac2_data,
                 NULL,
                 PRE_KERNEL_1,
                 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                 &dac_funcs
                );

#endif // DAC 2

// DAC 3
#if DT_NODE_HAS_STATUS(DAC3_NODELABEL, okay)

struct stm32_dac_driver_data dac3_data =
{
	.dac_struct = DAC3,
	.dac_mode   = dac_mode_unset
};

DEVICE_DT_DEFINE(DAC3_NODELABEL,
                 dac_stm32_init,
                 device_pm_control_nop,
                 &dac3_data,
                 NULL,
                 PRE_KERNEL_1,
                 CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                 &dac_funcs
                );

#endif // DAC 3
