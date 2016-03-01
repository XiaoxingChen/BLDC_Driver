/**
  ******************************************************************************
  * @file    incEncoder.h
  * @author  Chenxx
  * @version V1.0
  * @date    2016-01-31
  * @brief   This file provides the increament encoder class.
  ******************************************************************************/
#ifndef __INCENCODER_H
#define __INCENCODER_H
#include "stm32f10x.h"
class CTimer;
class incEncoder
{
public:
	//
	//struct physical value
	//
	struct phyval_Typdef
	{
		uint16_t pos_raw;
		int16_t vel_raw;
		int16_t acc_raw;
	};

	incEncoder(TIM_TypeDef* TIMx = TIM3);
	void CTimer_Init();

	void Timer_Overflow();
	void update_phy_val(); //update physical value
	//
	// little function
	//
	uint16_t Get_pos_raw() const;
	const phyval_Typdef* Get_phyval_() const;
	const CTimer* Get_Timer_() const;
	void start() const;
	void stop() const;

	int8_t pos_H_raw_;
private:
	void IcEncoderInit(); // Input capture
	void IcConfig(int TimChx);
	
	CTimer* Timer_;
	
	
	phyval_Typdef phy_val_;
	const uint8_t TimCh_;
	const static uint16_t ENCODER_PPR;
};
//extern incEncoder Encoder1;

#endif
//end of file
