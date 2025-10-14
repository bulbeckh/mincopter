
#include <arch/arm/stm32/RCOutput.h>

using namespace stm32;

extern const AP_HAL::HAL& hal;

STM32RCOutput::STM32RCOutput(void) :
	_enable_mask{},
	_channel{}
{ }

void STM32RCOutput::init(void* /* unused */)
{
	/* Will use PC6-PC9 as the four PWM channels (TIM3_CH1 - TIM3_CH4) */

	// TODO Change this to be configurable
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();

	// Configure PC6-PC9 as timer channel outputs
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* TIM3 runs on APB1 @ 84MHz
	 *
	 * Prescaler = 83+1 -> 1Mhz
	 *
	 * Period = 19999+1 -> 50Hz
	 *
	 * No clock division
	 *
	 * Gives us a 50Hz (20ms) frequency in which we pulse between 1000 and 2000 for ESC control */

	// Configurw PWM timer
	rcout_handle.Instance = TIM3;
	rcout_handle.Init.Prescaler = 83;
	rcout_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	rcout_handle.Init.Period = 19999;
	rcout_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_PWM_Init(&rcout_handle);

	// Configure PWM pulse generation
	TIM_OC_InitTypeDef sconfig_oc = {0};

	sconfig_oc.OCMode = TIM_OCMODE_PWM1;
	sconfig_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	sconfig_oc.OCFastMode = TIM_OCFAST_DISABLE;

	// Set each channel to enabled by default
	for (uint8_t i=0;i<STM32_RCOUT_MAX_CHANNELS;i++) {
		_enable_mask[i] = true;
		_channel[i] = 0;
	}

	// TODO Make this configurable
	sconfig_oc.Pulse = _channel[0];
	HAL_TIM_PWM_ConfigChannel(&rcout_handle, &sconfig_oc, TIM_CHANNEL_1);

	sconfig_oc.Pulse = _channel[1];
	HAL_TIM_PWM_ConfigChannel(&rcout_handle, &sconfig_oc, TIM_CHANNEL_2);

	sconfig_oc.Pulse = _channel[2];
	HAL_TIM_PWM_ConfigChannel(&rcout_handle, &sconfig_oc, TIM_CHANNEL_3);

	sconfig_oc.Pulse = _channel[3];
	HAL_TIM_PWM_ConfigChannel(&rcout_handle, &sconfig_oc, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&rcout_handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&rcout_handle, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&rcout_handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&rcout_handle, TIM_CHANNEL_4);

	return;
}

void STM32RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
	// TODO Implement different output channel frequencies
	
	return;
}

uint16_t STM32RCOutput::get_freq(uint8_t ch) {
	// TODO
	return 50;
}

void STM32RCOutput::enable_ch(uint8_t ch)
{
	if (ch >= STM32_RCOUT_MAX_CHANNELS) {
		hal.scheduler->panic("RCOUT - channel out of range\r\n");
	}

	_enable_mask[ch] = true;

	return;
}

void STM32RCOutput::disable_ch(uint8_t ch)
{
	if (ch >= STM32_RCOUT_MAX_CHANNELS) {
		hal.scheduler->panic("RCOUT - channel out of range\r\n");
	}

	_enable_mask[ch] = false;

	return;
}

void STM32RCOutput::write(uint8_t ch, uint16_t period_us)
{
	if (ch >= STM32_RCOUT_MAX_CHANNELS) {
		hal.scheduler->panic("RCOUT - channel out of range\r\n");
	}

	// TODO Replace this with a lookup between channel and TIM_CHANNEL_xx
	// TODO This needs to be refactored to have an early check for enabled/disabled and write 0 to PWM out
	switch(ch) {
		case 0:
			if (_enable_mask[0]) { 
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_1, period_us);
			} else {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_1, 0);
			}
			break;
		case 1:
			if (_enable_mask[1]) {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_2, period_us);
			} else {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_2, 0);
			}
			break;
		case 2:
			if (_enable_mask[2]) {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_3, period_us);
			} else {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_3, 0);
			}
			break;
		case 3:
			if (_enable_mask[3]) {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_4, period_us);
			} else {
				__HAL_TIM_SET_COMPARE(&rcout_handle, TIM_CHANNEL_4, 0);
			}
			break;
	}
}

void STM32RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
	// TODO
	return;
}

uint16_t STM32RCOutput::read(uint8_t ch) {
	if (ch >= STM32_RCOUT_MAX_CHANNELS) {
		hal.scheduler->panic("RCOUT - channel out of range\r\n");
	}

    return _channel[ch];
}

void STM32RCOutput::read(uint16_t* period_us, uint8_t len)
{
	if (len >= STM32_RCOUT_MAX_CHANNELS) {
		hal.scheduler->panic("RCOUT - channel out of range\r\n");
	}

	// Write each channel value to period_us
	for (uint8_t i=0;i<len;i++) period_us[i] = _channel[i];

	return;
}

