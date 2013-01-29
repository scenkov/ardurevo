#ifndef __EEPROM_H
#define __EEPROM_H

//#define EEPROM_DEBUG_ENABLE		//Enable debug
#include <AP_Hal.h>
#include <stddef.h>
#include <stdint.h>
#include <wirish.h>

#define EEPROM_FLASH 0					//Defines the type of Flash EEPROM
#define EEPROM_I2C   1					//Defines the type of EEPROM via I2C
#define EEPROM_TYPE_ENABLE EEPROM_I2C	//Defines the type of EEPROM used

#if EEPROM_TYPE_ENABLE == EEPROM_FLASH


#elif EEPROM_TYPE_ENABLE == EEPROM_I2C

	#define MC24C64		//Defines the EEPROM MC24C64
	#define EEPROM_ADDRESS	0xA0

	#ifndef EEPROM_PAGE_SIZE
		#if defined (MC24C64)
			#define EEPROM_PAGE_SIZE	(uint32_t)0x10000
		#else
			#error	"No EEPROM type specified."
		#endif
	#endif

	#ifndef EEPROM_START_ADDRESS
		#if defined (MC24C64)
			#define EEPROM_START_ADDRESS	0x00
		#else
			#error	"No EEPROM type specified.."
		#endif
	#endif

	class EEPROMClass
	{
	public:
		uint16_t Status;

		EEPROMClass(void);
		uint16_t init();
		uint16_t format(void);
		uint16_t read(uint16_t address);
		uint16_t read(uint16_t address, uint16_t *data);
		uint16_t write(uint16_t address, uint16_t data);

	};


#endif

extern EEPROMClass EEPROM;

//static functions - access to utilities to emulate EEPROM
extern void eeprom_read_block(void *pointer_ram, const void *pointer_eeprom, size_t n);
extern void eeprom_write_block(const void *pointer_ram, void *pointer_eeprom, size_t n);

extern uint8_t eeprom_read_byte(const uint8_t *addr);
extern uint16_t eeprom_write_byte(uint8_t *addr, uint8_t value);

extern uint16_t eeprom_read_word(const uint16_t *addr);
extern void eeprom_write_word(uint16_t *addr, uint16_t value);

extern uint32_t eeprom_read_dword(const uint32_t *addr);
extern void eeprom_write_dword(uint32_t *addr, uint32_t value);

#endif	/* __EEPROM_H */
