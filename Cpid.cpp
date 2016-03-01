/******************************************************************************
* @file    Cpid.cpp
* @author  Chenxx
* @version V1.0
* @date    2016-02-02
* @brief   This file achieve the position controller
******************************************************************************/
#include "Cpid.h"

Cpid::Cpid()
{
	m_.Kp = 1;
	m_.Ki = 0;
	m_.Kd = 0;
	m_.dead_B = 2;
	m_.Ek = 0;
	m_.Ek1 = 0;
}

int32_t Cpid::calculate(int32_t ref_val, int32_t act_val)
{
	//int32_t m_Ek = 0;		//the error in this time E(k)
	//int32_t m_Ek1 = 0;		//the error in last time E(k-1)
	int32_t m_Ek2 = 0;			//the error before last time E(k-2)
	//int16_t dead_B	//the dead band to avoid shake
	
	int32_t Coeff_A;					//the coefficent of Kp
	int32_t Coeff_B;					//the coefficent of Ki
	int32_t Coeff_C;					//the coefficent of Kd
	
	int32_t delta_val;				//calculated data that will be added to current data
	
	/*************Updata the error of each time*************/
	m_Ek2 = m_.Ek1;
	m_.Ek1 = m_.Ek;
	m_.Ek = ref_val - act_val;
	
	if((m_.Ek<m_.dead_B)&&(m_.Ek>(0-m_.dead_B)))	//do not calculate when error is small
		return 0;
	
	Coeff_A = m_.Ek-m_.Ek1;									// for Kp
	Coeff_B = m_.Ek;												// for Ki
	Coeff_C = (m_.Ek-m_.Ek1)-(m_.Ek1-m_Ek2);// for Kd
	
	delta_val = (Coeff_A*m_.Kp) +
							(Coeff_B*m_.Ki) +
							(Coeff_C*m_.Kd);
	return delta_val;
}

const Cpid::ParamTypDef& Cpid::Get_m_() const
{
	return m_;
}

//
// void Cpid::Set_Kp(uint16_t value)
//
void Cpid::Set_Kp(uint16_t value)
{
	m_.Kp = value;
}	

//
// void Cpid::Set_Ki(uint16_t value)
//
void Cpid::Set_Ki(uint16_t value)
{
	m_.Ki = value;
}	

//
// void Cpid::Set_Kd(uint16_t value)
//
void Cpid::Set_Kd(uint16_t value)
{
	m_.Kd = value;
}	

//
// void Cpid::Set_dead_band(int16_t value)
//
void Cpid::Set_dead_band(int16_t value)
{
	m_.dead_B = value;
}
//end of file
