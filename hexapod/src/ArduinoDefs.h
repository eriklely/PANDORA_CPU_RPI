//=======================================================================================
// Arduino definitions
// Gebruik de Arduino library
//=======================================================================================


#ifndef ARDUINO_DEFS_H 
#define ARDUINO_DEFS_H

#include <stdint.h>
#include <unistd.h>    // Voor usleep

// PROGMEM compatibiliteit
#define PROGMEM

// Programma geheugen leesfuncties
#define pgm_read_byte(addr)        (*((const char *)(addr)))
#define pgm_read_word(addr)        (((*((const unsigned char *)(addr) + 1)) << 8) + \
                                   (*((const unsigned char *)(addr))))
#define pgm_read_byte_near(addr)   (*((const char *)(addr)))
#define pgm_read_byte_far(addr)    (*((const char *)(addr)))
#define pgm_read_word_near(addr)   (((*((const unsigned char *)(addr) + 1)) << 8) + \
                                   (*((const unsigned char *)(addr))))
#define pgm_read_word_far(addr)    (((*((const unsigned char *)(addr) + 1)) << 8) + \
                                   (*((const unsigned char *)(addr))))

// String macro's
#define PSTR(str)                  (str)
#define F(str)                     (str)

// Type definities
typedef const char* PGM_P;
typedef unsigned char byte;
typedef unsigned short word;
typedef bool boolean;

// Tijdvertragingsfuncties
#define delay(ms)                  usleep((ms) * 1000)
#define delayMicroseconds(us)      usleep(us)

// Functie declaraties
extern unsigned long millis(void);
extern unsigned long micros(void);

// Wiskundige hulpfuncties
extern long min(long a, long b);
extern long max(long a, long b);
extern long map(long x, long in_min, long in_max, long out_min, long out_max);

#endif // ARDUINO_DEFS_H 