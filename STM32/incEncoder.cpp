/**
  ******************************************************************************
  * @file    incEncoder.cpp
  * @author  Chenxx
  * @version V1.0
  * @date    2016-01-31
  * @brief   This file provides the increament encoder functions.
  ******************************************************************************/
#include "incEncoder.h"
#include "CTimer.h"

const uint16_t ENCODER_LINES = 512;
const uint16_t incEncoder::ENCODER_PPR = ENCODER_LINES*4;
const uint16_t ENCODER_OVERFLOW = 65535 - (65535%(ENCODER_LINES*4));

incEncoder::incEncoder(TIM_TypeDef* TIMx)
	:TimCh_(CTimer::TimCh1 | CTimer::TimCh2)
{
	Timer_ = new CTimer(TIMx, Timer_->EmptyInit);
	CTimer_Init();
	phy_val_.pos_raw = Get_pos_raw();
}
void incEncoder::CTimer_Init()
{
	Timer_->ClkConfig();
	Timer_->Config(ENCODER_OVERFLOW, 1); //prescaler = 1.
	Timer_->PinConfig((CTimer::TimChType)TimCh_, GPIO_Mode_IPU);
	IcConfig(CTimer::TimCh1);
	IcConfig(CTimer::TimCh2);
	
	TIM_EncoderInterfaceConfig(Timer_->GetTIMx_(), 
		TIM_EncoderMode_TI12, 
		TIM_ICPolarity_Rising, 
		TIM_ICPolarity_Rising);
	
	Timer_->INTE = new CInte_Timer(Timer_);
	Timer_->INTE->Init(2, 1);
}

void incEncoder::IcConfig(int TimChx)
{
	//while the function lib does not us "|=" to judge channel
	//we cannot set more than 1 channel at one time
	//so the type of TimChx must be CTimer::TimChType
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	if(TimChx == CTimer::TimCh1)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	else if(TimChx == CTimer::TimCh2)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
	else if(TimChx == CTimer::TimCh3)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
	else if(TimChx == CTimer::TimCh4)
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	else
		while(1); //input error            
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;   
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x06;     
  TIM_ICInit(Timer_->GetTIMx_() , &TIM_ICInitStructure);
}

void incEncoder::Timer_Overflow()
{
	if((Timer_->GetTIMx_()->CR1 >> 4) & 1) //under flow
	{
		pos_H_raw_--;
	}
	else // over flow
	{
		pos_H_raw_++;
	}
}
void incEncoder::update_phy_val()
{
	int32_t vel;
	int32_t acc;
	
	vel = Get_pos_raw() - phy_val_.pos_raw + (int32_t)ENCODER_OVERFLOW*pos_H_raw_;
	acc = vel - phy_val_.vel_raw;
	
	pos_H_raw_ = 0;
	phy_val_.pos_raw = Get_pos_raw();
	phy_val_.vel_raw = vel;
	phy_val_.acc_raw = acc;
}
uint16_t incEncoder::Get_pos_raw() const
{
	return (Timer_->GetTIMx_()->CNT);
//	return (Timer_->GetTIMx_()->CNT % ENCODER_PPR);
}

const incEncoder::phyval_Typdef* incEncoder::Get_phyval_() const
{
	return &phy_val_;
}
const CTimer* incEncoder::Get_Timer_() const
{
	return Timer_;
}

void incEncoder::start() const
{
	Timer_->INTE->EnableLine(TIM_IT_Update);
	Timer_->Start();
}

void incEncoder::stop() const
{
	Timer_->Stop();
	Timer_->INTE->DisableLine(TIM_IT_Update);
}

//incEncoder Encoder1(TIM3);
//end of file
