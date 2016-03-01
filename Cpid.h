/******************************************************************************
* @file    Cpid.h
* @author  Chenxx
* @version V1.0
* @date    2016-02-02
* @brief   This file define the pid controller
******************************************************************************/
#ifndef __CPID_H
#define __CPID_H
#include "stdint.h"
class Cpid
{
public:
	struct ParamTypDef
	{
		uint16_t Kp;
		uint16_t Ki;
		uint16_t Kd;
		int16_t dead_B;//dead band
		uint8_t shift_p;
		uint8_t shift_i;
		uint8_t shift_d;
		int32_t Ek;			// error in last time E(k)
		int32_t Ek1;		// error in last time E(k-1)
//		int32_t Ek2;// error before last time E(k-2)
	};
	
	int32_t calculate(int32_t ref_val, int32_t act_val);
	Cpid();
	const ParamTypDef& Get_m_() const;
	void Set_Kp(uint16_t);
	void Set_Ki(uint16_t);
	void Set_Kd(uint16_t);
	void Set_dead_band(int16_t);
private:
	ParamTypDef m_;
	
	
};
#endif
//end of file
