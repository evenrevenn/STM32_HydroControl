#include "angle_detectors.h"
#include "stm32f1xx.h"

#include "motors_handler.h"
#include <string.h>
#include <UART_handler.h>

Potentiometer::Potentiometer(uint8_t id):
id_(id),
min_max_{0xFFFFU, 0xFFFFU}
{
    /* ADC PINS */
	GPIOA->CRL &= ~(0x3UL << (id * 0x4UL + 0x2UL));
	GPIOA->CRL &= ~(0x3UL << (id * 0x4uL + 0x0UL));

	uint32_t temp = 0;

	if (id > 0){
		temp = ADC1->SQR1 & ADC_SQR1_L;
		temp = temp + (1 << ADC_SQR1_L_Pos);
		temp &= ADC_SQR1_L;

		ADC1->SQR1 &= ~(ADC_SQR1_L);
		ADC1->SQR1 |= temp;
	}

	temp = 0b111UL << (id * 3);
	ADC1->SMPR2 |= temp;

	temp = static_cast<uint32_t>(id) << (id * 5);
	ADC1->SQR3 |= temp;
}

bool Potentiometer::readCalibratedFromFlash()
{
	std::pair<uint16_t,uint16_t> min_max_new;
	uint16_t *min_max_addr = getFlashAddress();

	min_max_new.first = *(min_max_addr + 0);
	min_max_new.second = *(min_max_addr + 1);

	if (min_max_new.first != 0xFFFF && min_max_new.first != 0xFFFF){
		min_max_ = min_max_new;
		return true;
	}
	else{
		return false;
	}
}

bool Potentiometer::writeCalibratedToFlash() const
{
	std::pair<uint16_t,uint16_t> min_max_stored;
	uint16_t *min_max_addr = getFlashAddress();

	min_max_stored.first = *(min_max_addr + 0);
	min_max_stored.second = *(min_max_addr + 1);

	if (min_max_stored.first != 0xFFFF && min_max_stored.second != 0xFFFF){
		return false;
	}
	if (FLASH->CR & FLASH_CR_LOCK){
		return false;
	}

	while(FLASH->SR & FLASH_SR_BSY);

	
	FLASH->CR |= FLASH_CR_PG;
	*(min_max_addr + 0) = min_max_.first;
	
	while(FLASH->SR & FLASH_SR_BSY);
	if (*(min_max_addr + 0) != min_max_.first){
		return false;
	}


	FLASH->CR |= FLASH_CR_PG;
	*(uint16_t *)(min_max_addr + 1) = min_max_.second;

	while(FLASH->SR & FLASH_SR_BSY);
	if (*(min_max_addr + 1) != min_max_.second){
		return false;
	}

	return true;
}

uint16_t *Potentiometer::getFlashAddress() const
{
    return (uint16_t *) PotentiometersHandler::FLASH_CALIB_BASE_ADDR + (2 * id_ * sizeof(uint16_t));
}

PotentiometersHandler::PotentiometersHandler(uint8_t potentiometers_num):
	conv_buffer_((const uint16_t *)pvPortCalloc(potentiometers_num * BUFFER_LENGTH, sizeof(uint16_t))),
	flash_is_valid_(true)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	/* Power-Up ADC1 */
	if (ADC1->CR2 & ADC_CR2_ADON){
		return;
	}

	ADC1->CR2 |= ADC_CR2_ADON;
	while((ADC1->CR2 & ADC_CR2_ADON) == 0);

	potentiometers_.reserve(potentiometers_num);
	for (int i = 0; i < potentiometers_num; i++)
	{
		potentiometers_.push_back(Potentiometer(i));
	}

	for (Potentiometer &potentiometer : potentiometers_){
		if (!potentiometer.readCalibratedFromFlash()){
			print("Potentiometer %u: angles read FAILED", potentiometer.id_);
			flash_is_valid_ = false;
		}
		else{
			print("Potentiometer %u: angles read SUCCESS, %u : %u", potentiometer.id_, \
																	  potentiometer.min_max_.first, \
																	  potentiometer.min_max_.second);
		}
	}

	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT;

	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_TEIE | DMA_CCR_TCIE;

	DMA1_Channel1->CNDTR = (uint16_t)potentiometers_num * BUFFER_LENGTH;
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(conv_buffer_);


	/* Calibration */
	for(volatile int i = 0; i < 10000; i++);
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);
}

void PotentiometersHandler::startConversion()
{
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	ADC1->CR2 |= ADC_CR2_ADON;
}

void PotentiometersHandler::stopConversion()
{
	ADC1->CR2 &= ~(ADC_CR2_ADON);
	DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
}

uint16_t PotentiometersHandler::getConversionResult(uint8_t potentiometer_id)
{
	uint32_t average = conv_buffer_[potentiometer_id];

	for (uint8_t i = 1; i < BUFFER_LENGTH; i ++)
	{
		average += conv_buffer_[i * potentiometers_.size() + potentiometer_id];
	}

    return average / BUFFER_LENGTH;
}

void PotentiometersHandler::updateMinMax(uint8_t id, Potentiometer::MinMax min_max_flag)
{
	Potentiometer& potentiometer = potentiometers_.at(id);
	
	switch (min_max_flag){
	case Potentiometer::Min:
		potentiometer.min_max_.first = getConversionResult(id);
		break;

	case Potentiometer::Max:
		potentiometer.min_max_.second = getConversionResult(id);
		break;
	}
}

bool PotentiometersHandler::saveCalibrationToFlash()
{
	if (FLASH->CR & FLASH_CR_LOCK){
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = FLASH_CALIB_BASE_ADDR;
	FLASH->CR |= FLASH_CR_STRT;

	while (FLASH->SR & FLASH_SR_BSY);

	bool ret = true;
	for (const Potentiometer &potentiometer : potentiometers_)
	{
		if (potentiometer.writeCalibratedToFlash()){
			print("Potentiometer %u: angles save SUCCESS, %u : %u", potentiometer.id_, \
																	  potentiometer.min_max_.first, \
																	  potentiometer.min_max_.second);
		}
		else{
			print("Potentiometer %u: angles save FAILED", potentiometer.id_);
			ret = false;
		}
	}

    return ret;
}

bool PotentiometersHandler::flashIsValid() const
{
    return flash_is_valid_;
}
