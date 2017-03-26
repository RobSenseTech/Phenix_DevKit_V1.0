#pragma once

#include "board_config.h"
/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * Its not an elegant solution but we can improve it in future.
 */

#define HAL_BOARD_SITL     3
#define HAL_BOARD_SMACCM   4 // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_FLYMAPLE 6
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_QURT     9
#define HAL_BOARD_EMPTY    99

// default board subtype is -1
#define HAL_BOARD_SUBTYPE_NONE -1

/**
   HAL Linux sub-types, starting at 1000
 */
#define HAL_BOARD_SUBTYPE_LINUX_NONE     1000
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD 1001
#define HAL_BOARD_SUBTYPE_LINUX_PXF      1002
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO    1003
#define HAL_BOARD_SUBTYPE_LINUX_ZYNQ     1004
#define HAL_BOARD_SUBTYPE_LINUX_BBBMINI  1005
#define HAL_BOARD_SUBTYPE_LINUX_BEBOP    1006
#define HAL_BOARD_SUBTYPE_LINUX_RASPILOT 1007
#define HAL_BOARD_SUBTYPE_LINUX_MINLURE  1008
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 1009
#define HAL_BOARD_SUBTYPE_LINUX_BH       1010
#define HAL_BOARD_SUBTYPE_LINUX_QFLIGHT  1011
#define HAL_BOARD_SUBTYPE_LINUX_PXFMINI  1012
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO2   1013

/**
   HAL PX4 sub-types, starting at 2000
 */
#define HAL_BOARD_SUBTYPE_PX4_V1         2000
#define HAL_BOARD_SUBTYPE_PX4_V2         2001

/**
   HAL VRBRAIN sub-types, starting at 4000
 */
#define HAL_BOARD_SUBTYPE_VRBRAIN_V45    4000
#define HAL_BOARD_SUBTYPE_VRBRAIN_V51    4001
#define HAL_BOARD_SUBTYPE_VRBRAIN_V52    4002
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V51   4003
#define HAL_BOARD_SUBTYPE_VRUBRAIN_V52   4004

// InertialSensor driver types
#define HAL_INS_MPU60XX_SPI 2
#define HAL_INS_MPU60XX_I2C 3
#define HAL_INS_HIL     4
#define HAL_INS_PX4     5
#define HAL_INS_FLYMAPLE 6
#define HAL_INS_L3G4200D 7
#define HAL_INS_VRBRAIN  8
#define HAL_INS_MPU9250_SPI  9
#define HAL_INS_L3GD20   10
#define HAL_INS_LSM9DS0 11
#define HAL_INS_RASPILOT 12
#define HAL_INS_MPU9250_I2C 13
#define HAL_INS_BH          14
#define HAL_INS_QFLIGHT  15
#define HAL_INS_QURT     16

// barometer driver types
#define HAL_BARO_BMP085     1
#define HAL_BARO_MS5611_I2C 2
#define HAL_BARO_MS5611_SPI 3
#define HAL_BARO_MS5607_I2C 4
#define HAL_BARO_PX4        5
#define HAL_BARO_HIL        6
#define HAL_BARO_VRBRAIN    7
#define HAL_BARO_MS5637_I2C 8
#define HAL_BARO_QFLIGHT    9
#define HAL_BARO_QURT      10

// compass driver types
#define HAL_COMPASS_HMC5843   1
#define HAL_COMPASS_PX4       2
#define HAL_COMPASS_HIL       3
#define HAL_COMPASS_VRBRAIN   4
#define HAL_COMPASS_AK8963_MPU9250 5
#define HAL_COMPASS_AK8963_I2C  6
#define HAL_COMPASS_HMC5843_MPU6000 7
#define HAL_COMPASS_RASPILOT  8
#define HAL_COMPASS_AK8963_MPU9250_I2C  9
#define HAL_COMPASS_BH                  10
#define HAL_COMPASS_QFLIGHT   11
#define HAL_COMPASS_QURT      12

// Heat Types
#define HAL_LINUX_HEAT_PWM 1

/**
   CPU classes, used to select if CPU intensive algorithms should be used

   Note that these are only approximate, not exact CPU speeds.
 */
#define HAL_CPU_CLASS_16   1   // DEPRECATED: 16Mhz, AVR2560 or similar
#define HAL_CPU_CLASS_75   2   // 75Mhz, Flymaple or similar
#define HAL_CPU_CLASS_150  3   // 150Mhz, PX4 or similar, assumes
                               // hardware floating point. Assumes tens
                               // of kilobytes of memory available
#define HAL_CPU_CLASS_1000 4   // GigaHz class, SITL, BeagleBone etc,
                               // assumes megabytes of memory available

/**
   operating system features:

   HAL implementations may define the following extra feature defines
   to 1 if available

  HAL_OS_POSIX_IO    :  has posix-like filesystem IO
  HAL_OS_SOCKETS     :  has posix-like sockets
 */

#define HAL_BOARD_NAME "PX4"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/fs/microsd/APM/TERRAIN"
#define HAL_PARAM_DEFAULTS_PATH "/etc/defaults.parm"
#define HAL_INS_DEFAULT HAL_INS_PX4
#define HAL_BARO_DEFAULT HAL_BARO_PX4
#define HAL_COMPASS_DEFAULT HAL_COMPASS_PX4
#define HAL_SERIAL0_BAUD_DEFAULT 115200

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V2
#define HAL_STORAGE_SIZE            16384

#define HAL_GPIO_A_LED_PIN        27
#define HAL_GPIO_B_LED_PIN        26
#define HAL_GPIO_C_LED_PIN        25
#define HAL_GPIO_LED_ON           LOW
#define HAL_GPIO_LED_OFF          HIGH

#define EXTERNAL_LED_GPS     28    // GPS LED - AN10
#define EXTERNAL_LED_ARMED   29    // Armed LED - AN11
#define EXTERNAL_LED_MOTOR1  30    // Motor1 LED - AN8
#define EXTERNAL_LED_MOTOR2  31    // Motor2 LED - AN12

#ifndef HAL_PARAM_DEFAULTS_PATH
#define HAL_PARAM_DEFAULTS_PATH NULL
#endif
