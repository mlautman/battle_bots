//  --------------------------------------------------------------------------------
//	M2 Port Expansion Board
// 	Version 1.0
//	Date: 6/13/2012
//	Author: Nikolay Vladimirov
// ---------------------------------------------------------------------------------

#ifndef m_port__
#define m_port__

#include "m_general.h"
#include "m_bus.h"

#define DDRH 0x01
#define DDRG 0x00
#define PORTH 0x13
#define PORTG 0x12
#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

unsigned char m_port_init(unsigned char address);
// FUNCTIONALITY
// initialize an mPort device on a specific address:
// - registers are sequential
// - sequential write mode is off
//
// TAKES:
// address: The I2C address of the mPort (between 0x20 and 0x27)
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_port_set(unsigned char address, unsigned char reg, unsigned char pin);
// FUNCTIONALITY
// Sets reg:bit
//
// TAKES:
// address: The I2C address of the mPort (between 0x20 and 0x27)
// reg: can be one of the following: PORTH, PORTG, DDRH, or DDRG
// pin: 0-7
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_port_clear(unsigned char address, unsigned char reg, unsigned char pin);
// FUNCTIONALITY
// clears reg:bit
//
// TAKES:
// address: The I2C address of the mPort (between 0x20 and 0x27)
// reg: can be one of the following: PORTH, PORTG, DDRH, or DDRG
// pin: 0-7
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_port_check(unsigned char address, unsigned char reg, unsigned char pin);
// FUNCTIONALITY
// check the state of reg:bit
//
// TAKES:
// address: The I2C address of the mPort (between 0x20 and 0x27)
// reg: can be one of the following: PORTH, PORTG
// pin: 0-7
//
// RETURNS:
// 1 : true
// 0 : false

#endif
