/*
 * wireless.c
 *
 * Created: 10/25/2012 10:58:02 PM
 *  Author: yunkai
 */ 
#include <stdlib.h>
#include "m_wireless.h"
#include <string.h>

char TX_address;
int j=0;
char m_wireless_init( char RX_address ){
	m_bus_init();
	return  m_rf_open(chan,RX_address,pack_len);
}
void m_change_buddy(char buddy){
	TX_address=buddy;
}
char m_wireless_int(int i) {
	char string[pack_len] = {0};
	itoa(i,string,10);
	return m_rf_send(TX_address,string, pack_len);
}
char m_wireless_uint(unsigned int i) {
	char string[pack_len] = {0};
	utoa(i,string,10);
	return m_rf_send(TX_address,string, pack_len);
}
char m_wireless_char(char i) {
	char string[pack_len] = {0};
	string[0] = i;
	return m_rf_send(TX_address,string, pack_len);
}
char m_wireless_string(char* i,int numC)
{
	char a = 1 ;
	char b = 0 ;
	char string[pack_len] = {0};
	while (numC>0){
		int m;
		for (m=0; m < min(numC,8) ;m++)
		{
			string[m] = i[m];
		}
		b = m_rf_send(TX_address,string, pack_len);
		numC=numC-8;
		if ( b == 0 )
			a=b;
	}
	return a;
}


int min (int a, int b){
	if ( a<b ){
		return a;
	} else if ( b<=a ){
		return b;
	} else return b;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void smallDelay(int value)
{
	int i;
	for ( i=0;i<value;i++){
		j=1;
	}
}
/*
char getLowChar( int integerVal){
	char* cP = (char *) &integerVal;
	return *(cP+1);	
}
char getHighChar( int integerVal){
	char* cp = (char*) &integerVal;
	return *cp;
}*/
