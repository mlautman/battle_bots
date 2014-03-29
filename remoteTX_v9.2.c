#define 	p_length 	12
#define 	chan		1
#define	 	RX_add 		0x25
#define 	TX_add 		0x17
#define 	ADC_DELAY 	2000

#include <avr/io.h>
#include "m_imu.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_general.h"
#include "m_rf.h"
#include <stdlib.h>
#include "m_wireless.h"

void setup_timer_3();

//////////////////////////////////////////////////
//
// 	Interface 
//
//////////////////////////////////////////////////

void setup_pins(void);
void update_ADC( int,int,int,int);
void check_buttons(void);
void setupUSB(void);

void debug_ADC_sums(int timer_3_cnt, long VRightSum,long VLeftSum, long HLeftSum, long HRightSum );
void debug_ADC_vals(int  Calibration,  long VRightSum,long VLeftSum, long HLeftSum, long HRightSum );

bool debug_ADC = false ;
//////////////////////////////////////////////////
//
// 	drive Modes
//
//////////////////////////////////////////////////
void set_drive_mode( bool L_bump, bool R_bump);

bool Mario 		= 	false;
bool single_joy 	= 	false;
bool double_joy 	= 	false;
bool tank_mode	 	= 	false;


//////////////////////////////////////////////////
//
// 	Comm Stuff
//
//////////////////////////////////////////////////

void TX_comm(void);
void deal_with_new(void );

char 	new	 		=	0;
char 	receive_buffer[p_length]=	{0}	;
char 	send_buffer[p_length]	=	{0} 	;   // [ leftB, rightB, VLadc, = , VRadc, = , HLadc, =,  HRadc, =, _  ]

						/* send_buffer[0]  	= leftB
						   send_buffer[1]  	= rightB
						   send_buffer[2 & 3] 	= VLadc
						   send_buffer[4 & 5] 	= VRadc
						   send_buffer[6 & 7] 	= HLadc
						   send_buffer[8 & 9] 	= HRadc
						   send_buffer[11]  	= playMode ( mk = 64 , sj = 1 , dj = 11 , tank  = 100 ) 
						 */


int main(void) {
	m_clockdivide(2);
	m_bus_init();
	setup_timer_3();
	m_rf_open(chan,TX_add,p_length);
	setup_pins();
	if (debug_ADC){ setupUSB();}

	long VLeftSum=0;
	long HLeftSum=0;
	long VRightSum=0;
	long HRightSum=0;
//	int  VLeftSum=0;
//	int  HLeftSum=0;
//	int  VRightSum=0;
//	int  HRightSum=0;


	int cnt_RB = 0;
	int cnt_LB = 0;
	

	/////////////////////////////////////////////////////////////////////////////////
	//
	// give me a half second to go from switching on the m2 to holding the config buttons.
	//
	//////////////////////////////////////////////////////////////////////////////////

	m_green(1);
	int timer_3_cnt=0;
	int Calibration = 50;
	for (timer_3_cnt=0 ; timer_3_cnt< Calibration ; ){
		if(check(TIFR3,OCF3A)){		 ///// timer 3 runs at 100Hz
			set(TIFR3,OCF3A );			
			timer_3_cnt++;
		}
	}

	m_red(ON);

	for (timer_3_cnt=0 ; timer_3_cnt<Calibration ; ){
		if(check(TIFR3,OCF3A)){		 ///// timer 3 runs at 100Hz
			set(TIFR3,OCF3A );			
			timer_3_cnt++;
		

		update_ADC(0,0,0,0);
		
		check_buttons();
		if (send_buffer[1]==1){ cnt_RB++ ;}
		if (send_buffer[0]==1){ cnt_LB++ ;}

		VLeftSum  += *(int*)(&send_buffer[2]);
		HLeftSum  += *(int*)(&send_buffer[6]); 
		VRightSum += *(int*)(&send_buffer[4]); 
		HRightSum += *(int*)(&send_buffer[8]); 

//		if (debug_ADC){while(!m_usb_rx_available()); m_usb_rx_flush();}
		if (debug_ADC){debug_ADC_sums( timer_3_cnt,  VRightSum, VLeftSum,  HLeftSum,  HRightSum );}
		}	
	}

	if (debug_ADC){while(!m_usb_rx_available()); m_usb_rx_flush();}

	int VLeftOffset = (int)(VLeftSum/Calibration);
	int HLeftOffset = (int)(HLeftSum/Calibration);
	int VRightOffset= (int)(VRightSum/Calibration);
	int HRightOffset= (int)(HRightSum/Calibration);
	
	bool L_bump  = ( cnt_LB > Calibration/2 );
	bool R_bump = ( cnt_RB > Calibration/2 );

	
	if (debug_ADC){while(!m_usb_rx_available()); m_usb_rx_flush();}
	if (debug_ADC){debug_ADC_vals( Calibration,   VRightSum, VLeftSum,  HLeftSum,  HRightSum );}
	m_green(0);
	m_red(OFF);

	set_drive_mode( L_bump , R_bump );
	
	while (1){
		check_buttons();
		update_ADC( VLeftOffset,HLeftOffset,VRightOffset,HRightOffset );
		TX_comm();
		if (debug_ADC){deal_with_new();}
	}
}
void set_drive_mode( bool L_bump, bool R_bump){
	if 	( L_bump && R_bump ) 	{ Mario 	= true;}	
	else if (!L_bump && R_bump ) 	{ tank_mode 	= true;}
	else if ( L_bump && !R_bump ) 	{ double_joy 	= true;}
	else if ( !L_bump && !R_bump ) 	{ single_joy 	= true;}

	if 	( Mario )	 	{ send_buffer[11] = 64; m_red(1); m_green(1);} 
	else if ( tank_mode )	 	{ send_buffer[11] =100;	m_red(1); m_green(0);} 
	else if ( double_joy ) 		{ send_buffer[11] = 11;	m_red(0); m_green(1);}  
	else if ( single_joy ) 		{ send_buffer[11] =  1;	m_red(0); m_green(0);}  
}



void setupUSB(){
	m_usb_init();
	m_green(ON);
	while (!m_usb_isconnected());
	m_green(OFF);
}

void check_buttons(){
		send_buffer[0]=( check(PINB, 0) ? 1 :  0);
		send_buffer[1]=( check(PINB ,4) ? 1 :  0);
}


void update_ADC( int VLOff,int HLOff,int VROff, int HROff ){
	clear(ADCSRA,ADEN);

	clear(ADCSRB, MUX5);		// set pin to F0
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);

	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	

	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	int VL = 1023 -  (ADC) - VLOff;	
	send_buffer[2]= *(char*)&VL;
	send_buffer[3]= *( ((char*)&VL) + 1);

	clear(ADCSRA,ADEN);

	clear(ADCSRB, MUX5);		// set pin to F1
	clear(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);

	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	

	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	int VR =1023 - (ADC) - VROff;
	send_buffer[4]= *(char*)&VR;
	send_buffer[5]= *( ((char*)&VR) + 1);


	clear(ADCSRA,ADEN);

	clear(ADCSRB, MUX5);		// set pin to F4
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	clear(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	

	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	int HL = 1023- (ADC ) - HLOff;
	send_buffer[6]= *(char*)&HL;
	send_buffer[7]= * (((char*)&HL)+1);


	clear(ADCSRA,ADEN);

	clear(ADCSRB, MUX5);		// set pin to F5
	set(ADMUX, MUX2);
	clear(ADMUX, MUX1);
	set(ADMUX, MUX0);
	
	set(ADCSRA, ADEN);		// start conversion process
	set(ADCSRA, ADSC);	
	
	while(!check(ADCSRA,ADIF));		// wait to finish
	set(ADCSRA,ADIF);
	int HR = ADC - HROff; 
	send_buffer[8]= *((char*)&HR);
	send_buffer[9]= *(((char*)&HR)+1);
}



void setup_pins(){

	clear(DDRB, 0 );	//set digital pins to inputs for limit switches
	clear(DDRB, 4 );	//


	clear(ADMUX,REFS1);  	// set Vref = Vcc
	set(ADMUX,REFS0); 

	set(ADCSRA,ADPS2);	// set ADC prescaller to 128 -> a 125Khz
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);

	set(DIDR0,ADC0D);
	set(DIDR0,ADC1D);
	set(DIDR0,ADC4D);
	set(DIDR0,ADC5D);
}


void TX_comm(){ 
	if (m_rf_send( RX_add, send_buffer, p_length) == 0 ){m_red(1);m_green(1);}; 
}
void  setup_timer_3(){

	//	while (!check(TIFR3, TOV3));		 ///// timer 3 runs at 100Hz
	//	set(TIFR3,OCF3A);			

	OCR3A =156; 				//1600 /625/ 256  >> 100Hz clock

	set(TCCR3B, CS32); 	clear(TCCR3B, CS31); 	clear(TCCR3B, CS30);	// <-- clock Prescaller 16MHz/1024
	clear(TCCR3B, WGM33);	set(TCCR3B, WGM32);
	clear(TCCR3A, WGM31);	clear(TCCR3A, WGM30);	//(mode 4) UP to OCR3A
}


void debug_ADC_sums(int timer_3_cnt, long VRightSum,long VLeftSum, long HLeftSum, long HRightSum ){
		m_usb_tx_int(timer_3_cnt);
		m_usb_tx_string(" me  \t");
		m_usb_tx_long(VRightSum);
		m_usb_tx_char('\t');
		m_usb_tx_long(HRightSum);
		m_usb_tx_char('\t');
		m_usb_tx_long(VLeftSum);
		m_usb_tx_char('\t');
		m_usb_tx_long(HLeftSum);
		m_usb_tx_string("\t\n\r");
}
void debug_ADC_vals( int Calibration, long VRightSum,long VLeftSum, long HLeftSum, long HRightSum ){

	int VLeftOffset = (int)(VLeftSum/Calibration);
	int HLeftOffset = (int)(HLeftSum/Calibration);
	int VRightOffset= (int)(VRightSum/Calibration);
	int HRightOffset= (int)(HRightSum/Calibration);

 	m_usb_tx_string("VLeftOffset\t");
	m_usb_tx_int(VLeftOffset);
	m_usb_tx_char('\t');
 	m_usb_tx_string("HLeftOffset\t");
	m_usb_tx_int(HLeftOffset);
	m_usb_tx_char('\t');
 	m_usb_tx_string("VRightOffset\t");
	m_usb_tx_int(VRightOffset);
	m_usb_tx_char('\t');
 	m_usb_tx_string("LRightOffset\t");
	m_usb_tx_int(HRightOffset);
	m_usb_tx_char('\t');
}

void deal_with_new(void ){
	new = 0;
	int i; int a;

	if (send_buffer[11] == 1 ){ m_usb_tx_string("Single Joystick Driving\t"); }
	if (send_buffer[11] ==11 ){ m_usb_tx_string("Double Joystick Driving\t"); }
	if (send_buffer[11] ==64 ){ m_usb_tx_string("Mario Kart RACING!\t"); }
	if (send_buffer[11] == 3 ){ m_usb_tx_string("Keyboard control \t"); }

	m_usb_tx_string("Button L:  ");
	if (send_buffer[1]==1)	{m_usb_tx_string(" on"); }
	else 			{m_usb_tx_string("off");}
	m_usb_tx_string(": \t");

	m_usb_tx_string("\tButton R:  ");
	if (send_buffer[0]==1)	{m_usb_tx_string(" on") ;}
	else 			{m_usb_tx_string("off");}
	m_usb_tx_string(": \t");

	for (i=0; i < 4 ; i++){
		m_usb_tx_string("\tpot ");
		m_usb_tx_int(i*2+1);
		m_usb_tx_string(": \t");
		a = *(int*)&send_buffer[i*2+2];
		m_usb_tx_int(a);
	}
	m_usb_tx_string("\n\r");
}
