#include "FreeRTOS_Print.h" 
#include "ringbuffer.h"
#include "xuartps.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "spi/spi_backend.h"

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM				0xD3

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */

/* keep lowpass low to avoid noise issues */
#define RATE_100HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_200HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_200HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_400HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_400HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_400HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_800HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_800HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_800HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_800HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38
#define ADDR_LOW_ODR			0x39


/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define I3G4250D_DEFAULT_RATE			800 //data output frequency
#define I3G4250D_DEFAULT_RANGE_DPS		2000
#define I3G4250D_DEFAULT_FILTER_FREQ		35
#define I3G4250D_TEMP_OFFSET_CELSIUS		40


struct SDeviceViaSpi _devInstance;

static void write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	SpiTransfer(&_devInstance, cmd, NULL, sizeof(cmd));
}

static void write_checked_reg(unsigned reg, uint8_t value)
{
    uint8_t i;
	write_reg(reg, value);
}

static void reset()
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
                          REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, RANGE_2000DPS);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	bits |= RATE_800HZ_LP_30HZ;
	write_checked_reg(ADDR_CTRL_REG1, bits);
//	set_driver_lowpass_filter(I3G4250D_DEFAULT_RATE, I3G4250D_DEFAULT_FILTER_FREQ);

}

uint8_t read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	SpiTransfer(&_devInstance, cmd, cmd, sizeof(cmd));

	return cmd[1];
}

#if 0
static void prvSpiTask( void *pvParameters )
{
    uint8_t v;
    uint16_t xl,xh,yl,yh,zl,zh;
    int16_t x,y,z;
	float			_gyro_range_scale;
    float		new_range_scale_dps_digit = 70e-3f;
    float xf,yf,zf;


	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * 3.14159265358979323846f;
    Print_Err("reg4=%x\n", read_reg(ADDR_CTRL_REG4));
    while(1)
    {
        v = read_reg(ADDR_WHO_AM_I);
        xl=read_reg(ADDR_OUT_X_L); 
        xh=read_reg(ADDR_OUT_X_H); 
        yl=read_reg(ADDR_OUT_Y_L); 
        yh=read_reg(ADDR_OUT_Y_H); 
        zl=read_reg(ADDR_OUT_Z_L); 
        zh=read_reg(ADDR_OUT_Z_H); 

        x = (xh<<8)|xl;
        y = (yh<<8)|yl;
        z = (zh<<8)|zl;

        xf = x;
        yf = y;
        zf = z;
	xf= (((float)xf* _gyro_range_scale)) * 1.0f;
	yf= (((float)yf* _gyro_range_scale)) * 1.0f;
	zf= (((float)zf* _gyro_range_scale)) * 1.0f;

        Print_Info("v=%x x=%f y=%f z=%f\n", v, xf, yf, zf);
    //    Print_Info("v=%x x=%d y=%d z=%d\n", v, x, y, z);
		vTaskDelay( 500 / portTICK_RATE_MS );
    }
}
#else
static void prvSpiTask( void *pvParameters )
{
	float			_gyro_range_scale;
    float		new_range_scale_dps_digit = 70e-3f;
    float xf,yf,zf;
    int16_t xl,xh,yl,yh,zl,zh;
    int16_t x,y,z;

	struct {
		uint8_t		cmd;
	    int8_t		temp;
		uint8_t		status;
		int8_t		x[2];
		int8_t		y[2];
		int8_t		z[2];
	} raw_report;



	_gyro_range_scale = new_range_scale_dps_digit / 180.0f * 3.14159265358979323846f;
    while(1)
    {

        /* fetch data from the sensor */
        memset(&raw_report, 0, sizeof(raw_report));
        raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
        SpiTransfer(&_devInstance, (uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

        x = (((int16_t)raw_report.x[1]) << 8) | raw_report.x[0];
        y = (((int16_t)raw_report.y[1]) << 8) | raw_report.y[0];
        z = (((int16_t)raw_report.z[1]) << 8) | raw_report.z[0];

        xf= ((x * _gyro_range_scale)) * 1.0f;
        yf= ((y * _gyro_range_scale)) * 1.0f;
        zf= ((z * _gyro_range_scale)) * 1.0f;

        Print_Info("len=%d x=%x y=%x z=%x xf=%f yf=%f zf=%f\n", sizeof(raw_report), x, y, z ,xf, yf, zf);
    //    Print_Info("v=%x x=%d y=%d z=%d\n", v, x, y, z);
        
        xl=read_reg(ADDR_OUT_X_L); 
        xh=read_reg(ADDR_OUT_X_H); 
        yl=read_reg(ADDR_OUT_Y_L); 
        yh=read_reg(ADDR_OUT_Y_H); 
        zl=read_reg(ADDR_OUT_Z_L); 
        zh=read_reg(ADDR_OUT_Z_H); 
        x = (xh<<8)|xl;
        y = (yh<<8)|yl;
        z = (zh<<8)|zl;
        
        xf= ((x * _gyro_range_scale)) * 1.0f;
        yf= ((y * _gyro_range_scale)) * 1.0f;
        zf= ((z * _gyro_range_scale)) * 1.0f;

        Print_Err("x=%x y=%x z=%x xf=%f yf=%f zf=%f\n", x, y, z, xf, yf, zf);

       // uint8_t a[9] = {0x01,0x23,0x45,0x67,0x89,0xab,0xcd, 0xef, 0x01};
      //  memcpy(&raw_report, a, sizeof(a));
      //  Print_Warn("len=%d cmd =%x,tmp=%d, status=%x, x=%x y=%x z=%x\n",sizeof(raw_report), raw_report.cmd, raw_report.temp, raw_report.status,raw_report.x,raw_report.y,raw_report.z);

		vTaskDelay( 500 / portTICK_RATE_MS );
    }
}
#endif
void SpiTest()
{
    _devInstance.spi_id = 1;
	DeviceViaSpiCfgInitialize(&_devInstance,
							ESPI_DEVICE_TYPE_GYRO, 
							"i3g4250d",
							ESPI_CLOCK_MODE_2,
							(11*1000*1000));
    reset();
	Print_Info("create spi task:%d\n", xTaskCreate(prvSpiTask, "spi read", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL));
}
