/**
******************************************************************************
* @file    BLDC_Driver.cpp
* @author  Chenxx
* @version V1.0
* @date    2016-02-22
* @brief   This file defines the BLDC physical model.
*						This file is for PC.
******************************************************************************/
#include "BLDC_Driver.h"
#include "Model_BLDC.h"
#include <math.h>
#include <string.h>
//
//constructor
//
CBLDC_Driver::CBLDC_Driver()
	:ctrl_mode_(open_loop), 
	tar_ctrl_val_(0),
	vctrl_(NULL),
	pctrl_(NULL)
{}

//
//void CBLDC_Driver::Init()
//
void CBLDC_Driver::Init()
{
	vctrl_ = new Cpid;
	pctrl_ = new Cpid;
	MotModel_ = &Mot_Model;	//caution


	vctrl_->Set_Kp(1);
	vctrl_->Set_Ki(500);
	vctrl_->Set_Kd(0);
	vctrl_->Set_dead_band(2);

	pctrl_->Set_Kp(1);
	pctrl_->Set_Ki(100);
	pctrl_->Set_Kd(1);
	pctrl_->Set_dead_band(2);
}

//
//	const Cpid* Get_vctrl_();
//
Cpid* CBLDC_Driver::Get_vctrl_()
{
	return vctrl_;
}

//
// Cpid* Get_pctrl()
//
Cpid* CBLDC_Driver::Get_pctrl_()
{
	return pctrl_;
}

//
// void update()
//
void CBLDC_Driver::update()
{

	if (ctrl_mode_ == pos_ctrl)
	{
		int32_t temp;
		temp = pctrl_->calculate(tar_ctrl_val_, Get_Pos_());
		SetPwm16bDuty(temp + GetPwm16bDuty());


	}
	else if (ctrl_mode_ == vel_ctrl)
	{
		int32_t temp;
		temp = vctrl_->calculate((int16_t)tar_ctrl_val_, (int16_t)Get_Vel_());
		SetPwm16bDuty((int32_t)temp + GetPwm16bDuty());

	}
	else if (ctrl_mode_ == cur_ctrl)
	{
	}
	else if (ctrl_mode_ == posinc_ctrl)
	{

	}
	else // open_loop
	{
		/*SetPwm16bDuty((int16_t)tar_ctrl_val_);*/
	}
}

uint16_t CBLDC_Driver::Get_Pos_()
{
	uint16_t pos_16b;
	pos_16b = (uint16_t)(Mot_Model.Get_phyVal_().Pos * 65535 / (2 * M_PI * 31) + 0.5);
	return pos_16b;
}
uint16_t CBLDC_Driver::Get_Vel_()
{
	uint16_t vel_16b;
	vel_16b = (uint16_t)(int16_t)(Mot_Model.Get_phyVal_().Vel / (2 * M_PI) * 4096 / 1000 + 0.5);
	return vel_16b;
}
uint16_t CBLDC_Driver::Get_Acc_()
{
	return 0;
}
uint16_t CBLDC_Driver::Get_Cur_()
{
	uint16_t cur_16b;
	cur_16b = Mot_Model.Get_coilCur_()*1000; //mA
	return cur_16b;
}
uint16_t CBLDC_Driver::Get_PwmDuty10k()
{
	return (uint16_t)(Mot_Model.Get_phyVal_().PwmDuty * 10000 + 0.5);
}
void CBLDC_Driver::Set_PwmDuty10k(int16_t duty10k)
{
	double dutyF;
	duty10k = (duty10k > 9900) ? 9900 : duty10k;
	duty10k = (duty10k < -9900) ? -9900 : duty10k;
	dutyF = (double)duty10k / 10000.0;
	Mot_Model.set_PwmDutyF(dutyF);
}

//
//void CBLDC_Driver::SetPwm16bDuty(int32_t duty16b)
//
void CBLDC_Driver::SetPwm16bDuty(int32_t duty)
{
	duty = (duty > 32767) ? 32767 : duty;
	duty = (duty < -32767) ? (-32767) : duty;
	
	double dutyF;
	dutyF = (double)duty / 32767;
	Mot_Model.set_PwmDutyF(dutyF);
}

//
//int16_t CBLDC_Driver::GetPwm16bDuty()
//
int16_t CBLDC_Driver::GetPwm16bDuty()
{
	int16_t duty16b;
	duty16b = (int16_t)(Mot_Model.Get_phyVal_().PwmDuty*32767 + 0.5);
	return duty16b;
}

CBLDC_Driver::~CBLDC_Driver()
{
}

CBLDC_Driver driver;
//end of file
