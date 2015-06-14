#ifndef _FT_DATATYPES_H_
#define _FT_DATATYPES_H_

/*
File:   FT_DataTypes.h
*/

#ifdef __cplusplus
extern "C" {
#endif

#define FT_FALSE           		(0)
#define FT_TRUE					(1)

typedef unsigned char			ft_uint8_t;
typedef signed char				ft_int8_t;
typedef char					ft_char8_t;
typedef unsigned int			ft_uint16_t;
typedef int						ft_int16_t;
typedef unsigned long			ft_uint32_t;
typedef long					ft_int32_t;
typedef void					ft_void_t;

typedef unsigned char			ft_bool_t;

typedef const char				ft_const_char8_t;
typedef const unsigned char		ft_const_uint8_t;
typedef const unsigned int		ft_const_uint16_t;
typedef const unsigned long		ft_const_uint32_t;

typedef const char				ft_prog_char8_t;
typedef const unsigned char		ft_prog_uint8_t;
typedef const unsigned int		ft_prog_uint16_t;
typedef const unsigned long		ft_prog_uint32_t;

#ifdef __cplusplus
}
#endif


#endif /*_FT_DATATYPES_H_*/


/* Nothing beyond this*/




