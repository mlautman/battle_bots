/*
 * m_wireless.h
 *
 * Created: 10/25/2012 10:58:20 PM
 *  Author: mlautman 
 */ 


#ifndef WIRELESSX_H_
#define WIRELESSX_H_

#include "m_bus.h"
#include "m_rf.h"
#include <stdlib.h>

#ifndef chan
#define chan 1
#endif

#ifndef pack_len 
#define pack_len 8
#endif




extern char m_wireless_init( char RXaddress ); 
/* ***********************************************************************************************
		 -------------------- wireless_init( char* rxtx) ----------------------
	initializes the m_bus then upens up rf communication with anther M2 over channel 1
	param:
		char equaling either "rx" or "tx" 
	RETURNS:
		1: module acknowledged setup
		0: something went wrong
		-1: incorrect input parameter

*********************************************************************************************** */

void m_change_buddy( char buddy) ;

extern char m_wireless_int(int i) ;
/* ***********************************************************************************************
		 -------------------- m_wireless_int( int i) ----------------------
	sends an integer value over rf to another M2
	PARAM:  int i
	RETURNS:
		1: 	successful transmission to the mRF module 
		0: 	something went wrong
*********************************************************************************************** */

extern char  m_wireless_uint(unsigned int i);
/* ***********************************************************************************************
		 -------------------- wireless_uint() ----------------------
	sends an unsigned int  value over rf to another M2 
	param: unsigned int i 
	RETURNS:
		1: 	successful transmission to the mRF module 
		0: 	something went wrong
*********************************************************************************************** */

extern char  m_wireless_string(char* i,int numC);
/* ***********************************************************************************************
		 -------------------- wireless_string( char* , int) ----------------------
	sends an string value over rf to another M2 
	param:
		char* a string.
		int numC : size of string

	RETURNS:
		1: 	successful transmission to the mRF module 
		0: 	something went wrong
*********************************************************************************************** */

extern char  m_wireless_char(char i);
/* ***********************************************************************************************
		 -------------------- wireless_char( char* , int) ----------------------
	sends an char value over rf to another M2 
	param:
		char* a string.
		int numC : size of string
	RETURNS:
		1: 	successful transmission to the mRF module 
		0: 	something went wrong
*********************************************************************************************** */

int min(int a, int b);

void smallDelay(int value);

//char getLowChar(int integerVal);
//char getHighChar(int integerVal);

#endif /* WIRELESS_H_ */
