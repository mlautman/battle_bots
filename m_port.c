//  --------------------------------------------------------------------------------
//	M2 Port Expansion Board
// 	Version 1.0
//	Date: 6/13/2012
//	Author: Nikolay Vladimirov
// ---------------------------------------------------------------------------------

#include "m_general.h"
#include "m_bus.h"
#include "m_port.h"

//unsigned char m_read_register(unsigned char addr, unsigned char reg);
//unsigned char m_write_register(unsigned char addr, unsigned char reg, unsigned char value);

unsigned char m_port_init(unsigned char address){
	if(m_write_register(address, 0x05, 0x38))
        return 1;
    else
        return 0;
}

unsigned char m_port_set(unsigned char address, unsigned char reg, unsigned char pin){
	unsigned char value = m_read_register(address, reg);
	if(reg == DDRH || reg == DDRG){
        m_write_register(address, reg, value &= ~(1 << pin));
        return 1;
	}
	else if(reg == PORTH || reg == PORTG){
        m_write_register(address, reg, value |= 1 << pin);
        return 1;
    }
    else return 0;
}

unsigned char m_port_clear(unsigned char address, unsigned char reg, unsigned char pin){
	unsigned char value = m_read_register(address, reg);
	if(reg == PORTH || reg == PORTG){
        m_write_register(address, reg, value &= ~(1 << pin));
        return 1;
	}
	else if(reg == DDRH || reg == DDRG){
        m_write_register(address, reg, value |= 1 << pin);
        return 1;
    }
    else return 0;
}

unsigned char m_port_check(unsigned char address, unsigned char reg, unsigned char pin){
	return check(m_read_register(address, reg), pin);
}
