/**
  ******************************************************************************
  * @file    BLDC_Driver.cpp
  * @author  Chenxx
  * @version V1.0
  * @date    2016-01-30
  * @brief   This file achieve the BLDC_Hall
  ******************************************************************************/
#include "BLDC_Driver.h"
#include "BLDC_Hall.h"
#include "pwmGtor.h"
#include "incEncoder.h"
#include "CTimer.h"
#include "Cpid.h"
/******************************************************************************
* @brief   This part for BLDC_Driver
******************************************************************************/
BLDC_Driver::BLDC_Driver():
	ctrl_mode_(open_loop), tar_ctrl_val_(0),
	pwmIO_(NULL),npwmIO_(NULL),hall_(NULL),encoder_(NULL),vctrl_(NULL)
{}
void BLDC_Driver::Init(BLDC_Driver::InitTypdef* init)
{
	pwmIO_ = new pwmGtor(init->pwm_Timer, init->pwm_TimCh, init->pwm_Freq);
	npwmIO_ = new nPwmIO(init->npwmIO_Port, init->npwmIO_Pin);
	hall_ = new BLDC_Hall(init->hallIO_Port, init->hallIO_Pin);
	encoder_ = new incEncoder(init->encoder_Timer);
	vctrl_ = new Cpid;
	//TODO pctrl_ new
	pinc_ctrl_ = new Cpid;
	
	vctrl_->Set_Kp(1);
	vctrl_->Set_Ki(500);
	vctrl_->Set_Kd(0);
	vctrl_->Set_dead_band(2);
	
	pctrl_->Set_Kp(1);
	pctrl_->Set_Ki(100);
	pctrl_->Set_Kd(1);
	pctrl_->Set_dead_band(2);
	
	pinc_ctrl_->Set_Kp(1);
	pinc_ctrl_->Set_Ki(100);
	pinc_ctrl_->Set_Kd(0);
	pinc_ctrl_->Set_dead_band(2);
	
	pwmIO_->start();
	encoder_->start();
}

/**************************************************/
//
//void BLDC_Driver::Commutate_Hall()
//
/**************************************************/
const u8 BLDC_Driver::TAB_P[6] = {0x34, 0x2c, 0x29, 0x19, 0x1a, 0x32};
const u8 BLDC_Driver::TAB_N[6] = {0x19, 0x1a, 0x32, 0x34, 0x2c, 0x29};
void BLDC_Driver::CommutateHall()
{
	const uint8_t* hall_tab;
	if(rotateDir_)
	{
		hall_tab = BLDC_Driver::TAB_P;
	}
	else
	{
		hall_tab = BLDC_Driver::TAB_N;
	}
	switch (hall_->Get_halldata())
	{
		case	1:
			npwmIO_->Write_IO(hall_tab[0]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[0]&0x7, pwmDuty_raw_);
			break;
		case	3:
			npwmIO_->Write_IO(hall_tab[1]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[1]&0x7, pwmDuty_raw_);
			break;
		case	2:
			npwmIO_->Write_IO(hall_tab[2]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[2]&0x7, pwmDuty_raw_);
			break;
		case	6:
			npwmIO_->Write_IO(hall_tab[3]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[3]&0x7, pwmDuty_raw_);
			break;
		case	4:
			npwmIO_->Write_IO(hall_tab[4]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[4]&0x7, pwmDuty_raw_);
			break;
		case	5:
			npwmIO_->Write_IO(hall_tab[5]>>3);
			pwmIO_->set_PwmDuty_raw(hall_tab[5]&0x7, pwmDuty_raw_);
			break;
		default: 
			break;
	}
}
/**************************************************/
//
//void BLDC_Driver::SetOpenLoopDuty(uint16_t duty_phy, u8 dir)
// PS: duty_phy mean physic duty in 100.00%
//
/**************************************************/
void BLDC_Driver::SetOpenLoopDuty(int16_t duty_phy)
{
	uint16_t AutoReloadValue = pwmIO_->Get_Timer_()->GetTIMx_()->ARR+1;
	
	duty_phy = (duty_phy>9900)?9900:duty_phy;
	duty_phy = (duty_phy<-9900)?(-9900):duty_phy;
	
	if(duty_phy < 0)
	{
		rotateDir_ = 1;
		duty_phy = 0-duty_phy;
	}
	else // duty_phy >= 0
	{
		rotateDir_ = 0;
	}
	pwmDuty_raw_ = (uint32_t)duty_phy*AutoReloadValue/10000;
	if(encoder_->Get_phyval_()->vel_raw < 200)
		CommutateHall();
}

//
// Get_hall_() const
//
const BLDC_Hall* BLDC_Driver::Get_hall_() const
{
	return hall_;
}

//
// Get_encoder_() const
//
const incEncoder* BLDC_Driver::Get_encoder_() const
{
	return encoder_;
}

//
//	const Cpid* Get_vctrl_();
//
Cpid* BLDC_Driver::Get_vctrl_()
{
	return vctrl_;
}

//
// Cpid* Get_pctrl()
//
Cpid* BLDC_Driver::Get_pctrl_()
{
	return pctrl_;
}

//
// void update()
//
void BLDC_Driver::update()
{
	encoder_->update_phy_val();
	
	if(ctrl_mode_ == pos_ctrl)
	{
		int32_t temp;
		temp = pctrl_->calculate(tar_ctrl_val_, encoder_->Get_pos_raw());
		SetPwm16bDuty( temp+(int32_t)pwmDuty_raw_ );
		
		if(encoder_->Get_phyval_()->vel_raw < 500) 
			CommutateHall();
	}
	else if(ctrl_mode_ == vel_ctrl)
	{
		int32_t temp;
		temp = vctrl_->calculate((int16_t)tar_ctrl_val_, encoder_->Get_phyval_()->vel_raw);
		SetPwm16bDuty( (int32_t)temp + GetPwm16bDuty());
		
		if(encoder_->Get_phyval_()->vel_raw < 500) 
			CommutateHall();
	}
	else if(ctrl_mode_ == cur_ctrl)
	{}
	else if(ctrl_mode_ == posinc_ctrl)
	{
		int32_t temp;
		temp = pinc_ctrl_->calculate(calc_Target_pos(tar_ctrl_val_), encoder_->Get_pos_raw());
		SetPwm16bDuty( (int32_t)temp + GetPwm16bDuty());
		
		if(encoder_->Get_phyval_()->vel_raw < 500) 
			CommutateHall();
	}
	else // open_loop
	{
		int16_t veltemp = encoder_->Get_phyval_()->vel_raw;
		if( veltemp < 500 && veltemp > -500) 
			CommutateHall();
	}
}

//
// void Encoder_Overflow()
//
void BLDC_Driver::Encoder_Overflow()
{
	encoder_->Timer_Overflow();
}

//
//
//
int32_t BLDC_Driver::calc_Target_pos(int16_t pos_inc)
{
	int32_t TarPos;
	TarPos = (uint32_t)(encoder_->Get_pos_raw()) + pos_inc;
	return TarPos;
}

//
//	Get_rotateDir_() const
//
uint8_t BLDC_Driver::Get_rotateDir_() const
{
	return rotateDir_;
}

//
//	Get_pwmDuty_phy() const
//
uint16_t BLDC_Driver::Get_pwmDuty_phy() const
{
	uint16_t AutoReloadValue = pwmIO_->Get_Timer_()->GetTIMx_()->ARR+1;
	if(rotateDir_ == 0)
		return ((uint32_t)pwmDuty_raw_*10000/AutoReloadValue);
	else if(rotateDir_ == 1)
		return (0-(int32_t)pwmDuty_raw_*10000/AutoReloadValue);
	else 
		return 0;
}

/**************************************************/
//
//void BLDC_Driver::SetPwm16bDuty(int32_t duty)
//
//  PS: duty_raw come from pid calculator. So the 
//	data type is int32_t. There is an amplitude 
//	limiting filter inside.
/**************************************************/
void BLDC_Driver::SetPwm16bDuty(int32_t duty)
{
//	duty = (duty>32767)?32767:duty;
//	duty = (duty<-32767)?(-32767):duty;
	
	duty = (duty>32767)?32767:duty;
	duty = (duty<-32767)?(-32767):duty;
	
	uint16_t AutoReloadValue = pwmIO_->Get_Timer_()->GetTIMx_()->ARR+1;
	int32_t duty_raw = (int32_t)duty*AutoReloadValue/32767;
		if(duty_raw >= 0)
	{
		pwmDuty_raw_ = (uint16_t)duty_raw;
		rotateDir_ = 0;
	}
	else
	{
		pwmDuty_raw_ = (uint16_t)(0-duty_raw);
		rotateDir_ = 1;
	}
}
/**************************************************/
//
//int16_t BLDC_Driver::GetPwm16bDuty()
//
/**************************************************/
int16_t BLDC_Driver::GetPwm16bDuty()
{
	int16_t pwm16bDuty;
	uint16_t AutoReloadValue = pwmIO_->Get_Timer_()->GetTIMx_()->ARR+1;
	pwm16bDuty = (int32_t)pwmDuty_raw_*32767/AutoReloadValue;
	if(rotateDir_ == 0)
		return pwm16bDuty;
	else
		return (0-pwm16bDuty);
}

/******************************************************************************
* @brief   This part for BLDC_Driver::nPwmIO
******************************************************************************/
BLDC_Driver::nPwmIO::nPwmIO(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
	:GPIOx_(GPIOx)
{
	//check if only 3 continuou pins were selected
	uint16_t PinTemp = GPIO_Pin;
	while(1)
	{
		if(PinTemp == 0x7)	//0x7 = 0b 0000 0111
			break;
		else if(PinTemp == 0)
			while(1); //error!
		PinTemp >>= 1;
		PIN_POS++;
	}
	ClkConfig();
	PinConfig();
}
void BLDC_Driver::nPwmIO::ClkConfig()
{
	uint32_t RCC_APB2Periph_GPIOx;
	RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOA<<(((uint32_t)GPIOx_-(uint32_t)GPIOA)/0x400);
	// num / 0x400 == num >> 10 ?
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE); 
}
void BLDC_Driver::nPwmIO::PinConfig()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = 0x7<<PIN_POS; //0x7 = 0b 0000 0111
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Init(GPIOx_, &GPIO_InitStructure); 
	
}

void BLDC_Driver::nPwmIO::Write_IO(uint16_t data)
{
	static uint16_t IO_MASK = ~(0x7<<PIN_POS);
	GPIOx_->ODR &= IO_MASK;
	GPIOx_->ODR |= data << PIN_POS;
}


BLDC_Driver BLDC;
//end of file
