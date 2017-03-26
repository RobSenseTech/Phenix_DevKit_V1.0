/***************************************************************************
@FILE driver_define.h
***************************************************************************/
#ifndef __INCLUDE_DRV_DEFINE_H
#define __INCLUDE_DRV_DEFINE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
 /****************************************************************************
 * Type Declarations
 ****************************************************************************/
/* config constants.  */
//#define CONFIG_HAVE_LONG_DOUBLE


// for param.c support
/**
******************************
struct px4_parameters_t {
	
	const struct param_info_s __param__%s;   //name 1
	const struct param_info_s __param__%s;   //name 2
	const struct param_info_s __param__%s;   //name 3
	const unsigned int param_count;
};

struct px4_parameters_t px4_parameters = {
	{
		"%s",      //name1
		PARAM_TYPE_%s,
		%s%s;      //value
	},
	{
		"%s",      //name2
		PARAM_TYPE_%s,
		%s%s;      //value
	},
	%d
};
**********************************/

// you can update this to decide what pwm is valid.
#define REVERSE_PWM_MASK       0x000F
// max actuator define
#define PILOT_MAX_ACTUATOR      4;



#define FAR

#define  ERROR                 -1

#define OK						0
#define DEV_FAILURE				0
#define DEV_SUCCESS				1

// class instance for primary driver of each class
enum CLASS_DEVICE {
	CLASS_DEVICE_PRIMARY = 0,
	CLASS_DEVICE_SECONDARY = 1,
	CLASS_DEVICE_TERTIARY = 2
};

// use this to avoid issues between C++11 with NuttX and C++10 on
// other platforms.
#if !(defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L)
# define constexpr const
#endif

/* The difference between 1 and the least value greater than 1 that is
 * representable in the given floating-point type, b1-p.
 */

#define FLT_EPSILON 1.1920929e-07F  /* 1E-5 */


/* Useful constants.  */
#define MAXFLOAT	3.40282347e+38F

#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define M_TWOPI         (M_PI * 2.0)
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_3PI_4		2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_DEG_TO_RAD 	0.01745329251994
#define M_RAD_TO_DEG 	57.2957795130823
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3	1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */


#define M_E_F			2.7182818284590452354f
#define M_LOG2E_F		1.4426950408889634074f
#define M_LOG10E_F		0.43429448190325182765f
#define M_LN2_F			_M_LN2_F
#define M_LN10_F		2.30258509299404568402f
#define M_PI_F			3.14159265358979323846f
#define M_TWOPI_F       (M_PI_F * 2.0f)
#define M_PI_2_F		1.57079632679489661923f
#define M_PI_4_F		0.78539816339744830962f
#define M_3PI_4_F		2.3561944901923448370E0f
#define M_SQRTPI_F      1.77245385090551602792981f
#define M_1_PI_F		0.31830988618379067154f
#define M_2_PI_F		0.63661977236758134308f
#define M_2_SQRTPI_F	1.12837916709551257390f
#define M_DEG_TO_RAD_F 	0.01745329251994f
#define M_RAD_TO_DEG_F 	57.2957795130823f
#define M_SQRT2_F		1.41421356237309504880f
#define M_SQRT1_2_F		0.70710678118654752440f
#define M_LN2LO_F       1.9082149292705877000E-10f
#define M_LN2HI_F       6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      _M_LN2_F
#define M_INVLN2_F      1.4426950408889633870E0f  /* 1 / log(2) */
 
 
 #endif /* __INCLUDE_DRV_DEFINE_H */
