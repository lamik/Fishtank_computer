#ifndef _FT_DATATYPES_H_
#define _FT_DATATYPES_H_

#define FT_FALSE           (0)
#define FT_TRUE            (1)

typedef uint8_t ft_uint8_t;
typedef char ft_char8_t;
typedef signed char ft_schar8_t;
typedef unsigned char ft_uchar8_t;
typedef int  ft_int16_t;
typedef uint16_t ft_uint16_t;
typedef unsigned long ft_uint32_t;
typedef long ft_int32_t;
typedef void ft_void_t;

typedef unsigned char ft_bool_t;

typedef unsigned char  ft_prog_uchar8_t;
typedef signed char   ft_prog_char8_t;
typedef unsigned int ft_prog_uint16_t;

#define FT_PROGMEM __ATTR_PROGMEM__
#define 	__ATTR_PROGMEM__   __attribute__((__progmem__))
#define ft_random(x)		(rand() % (x))
#define ft_pgm_read_byte_near(x)   (*(x))
#define ft_pgm_read_byte(x)        (*(x))
#define ft_pgm_read_word(addr)   (*(ft_int16_t*)(addr))
#define TRUE 1
#define FLASE 0

#endif /*_FT_DATATYPES_H_*/


/* Nothing beyond this*/




