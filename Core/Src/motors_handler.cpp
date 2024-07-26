#include "motors_handler.h"
#include "stm32f1xx.h"
#include <UART_handler.h>

Motor::Motor(int id):
id_(id)
{
    /* PWM PINS */
    if (id < 2){
        GPIOA->CRL &= ~(0x1UL << (id * 0x4UL + 26UL));
        GPIOA->CRL |= 0x2UL << (id * 0x4UL + 26UL);
        
        GPIOA->CRL &= ~(0x1UL << (id * 0x4uL + 24UL));
        GPIOA->CRL |= 0x2UL << (id * 0x4uL + 24UL);

        TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) << (id * 0x8UL);
        TIM3->CCMR1 |= (TIM_CCMR1_OC1PE) << (id * 0x8UL);
    }
    else{
        GPIOB->CRL &= ~(0x1UL << (id * 0x4UL + 2UL));
        GPIOB->CRL |= 0x2UL << (id * 0x4UL + 2UL);

        GPIOB->CRL &= ~(0x1UL << (id * 0x4uL + 0UL));
        GPIOB->CRL |= 0x2UL << (id * 0x4uL + 0UL);

        TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) << (id * 0x8UL);
        TIM3->CCMR2 |= (TIM_CCMR2_OC3PE) << (id * 0x8UL);
    }

    TIM3->CCER |= TIM_CCER_CC1E << (id * 0x4UL);

    print("Motor %u initialised", id_);
}

void Motor::setDuty(uint16_t duty_count)
{
    volatile uint32_t *CCR_base = &(TIM3->CCR1);
    *(CCR_base + id_) = duty_count;
}

MotorsHandler::MotorsHandler(int motors_num)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->PSC = 2UL;
    TIM3->ARR = 512UL;

    for (int i = 0; i < motors_num; i++)
    {
        motors_.push_back(Motor(i));
    }

    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void MotorsHandler::setMotorSpeed(uint16_t speed, uint8_t id)
{
    motors_[id].setDuty(speed);

    TIM3->EGR |= TIM_EGR_UG;
}
