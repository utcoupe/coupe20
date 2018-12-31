#include "pwm.h"

Pwm::Pwm(TIM_HandleTypeDef* timer)
{
  m_timer_ptr = timer;
}

Pwm::~Pwm()
{
  delete m_timer_ptr;
}

void Pwm::set_timer_freq(uint32_t freq)
{
  uint32_t sys_freq = HAL_RCC_GetPCLK1Freq();

  m_timer_ptr->Init.Prescaler = 0;
  m_timer_ptr->Init.CounterMode = TIM_COUNTERMODE_UP;
  m_timer_ptr->Init.Period = (uint16_t)(2*sys_freq/(freq));
  m_timer_ptr->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  m_timer_ptr->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(m_timer_ptr) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void Pwm::set_channel_duty_cycle(uint32_t Channel, 
                                 uint8_t duty_cycle)
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = m_timer_ptr->Init.Period*duty_cycle/0xFF;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(m_timer_ptr, &sConfigOC, Channel) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}
  start(Channel);
}

void Pwm::set_duty_cycle(uint8_t duty_cycle)
{
  set_channel_duty_cycle(TIM_CHANNEL_1,duty_cycle);
}

void Pwm::start(uint32_t Channel)
{
  HAL_TIM_PWM_Start(m_timer_ptr,Channel);
}