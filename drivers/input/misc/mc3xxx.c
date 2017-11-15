/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
//#include <linux/mc3xxx.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

#include <linux/miscdevice.h>

#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

#include <asm/uaccess.h>
//#include <mach/system.h>
//#include <mach/hardware.h>
#include <linux/fs.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define MCUBE_FUNC_DEBUG
#define MCUBE_DOT_CALIBRATION
//#define MC3XXX_ENABLE_INT


#ifdef MCUBE_FUNC_DEBUG
    #define MC_PRINT(x...)        printk(x)
#else
    #define MC_PRINT(x...)
#endif

#define MC3XXX_I2C_ADDR				0x4c
#define MC3XXX_DEV_NAME				"mc3xxx"
//#define MC3XXX_INPUT_NAME     		"accelerometer"
#define MC3XXX_DRIVER_VERSION		"1.0.0"
#define MC3XXX_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define MC3XXX_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

// Register address define
#define MC3XXX_REG_XOUT			0x00
#define MC3XXX_REG_INTEN			0x06
#define MC3XXX_REG_MODE			0x07
#define MC3XXX_REG_SRFA				0x08
#define MC3XXX_REG_XOUT_EX_L		0x0D
#define MC3XXX_REG_OUTCFG			0x20
#define MC3XXX_REG_XOFFL			0x21
#define MC3XXX_REG_PCODE			0x3B

// Mode
#define MC3XXX_MODE_AUTO			0
#define MC3XXX_MODE_WAKE			1
#define MC3XXX_MODE_SNIFF			2
#define MC3XXX_MODE_STANDBY		3

// Range
#define MC3XXX_RANGE_2G			0
#define MC3XXX_RANGE_4G			1
#define MC3XXX_RANGE_8G_10BIT		2
#define MC3XXX_RANGE_8G_14BIT		3

// Bandwidth
#define MC3XXX_BW_512HZ			0
#define MC3XXX_BW_256HZ			1
#define MC3XXX_BW_128HZ			2
#define MC3XXX_BW_64HZ				3
#define MC3XXX_BW_32HZ				4
#define MC3XXX_BW_16HZ				5
#define MC3XXX_BW_8HZ				6

// initial value
#define MC3XXX_RANGE_SET			MC3XXX_RANGE_2G  /* +/-2g */
#define MC3XXX_BW_SET				MC3XXX_BW_128HZ /* 128HZ  */
#define MC3XXX_MIN_DELAY			1
#define MC3XXX_MAX_DELAY			200
#define ABSMIN                      -1024
#define ABSMAX                      1024

#define POLL_INTERVAL_MIN_MS	1
#define POLL_INTERVAL_MAX_MS	10000
#define POLL_DEFAULT_INTERVAL_MS 200

// product code
#define MC3XXX_PCODE_3210		0x90
#define MC3XXX_PCODE_3230		0x19
#define MC3XXX_PCODE_3250		0x88
#define MC3XXX_PCODE_3410		0xA8
#define MC3XXX_PCODE_3410N		0xB8
#define MC3XXX_PCODE_3430		0x29
#define MC3XXX_PCODE_3430N		0x39
#define MC3XXX_PCODE_3510B		0x40
#define MC3XXX_PCODE_3530B		0x30
#define MC3XXX_PCODE_3510C		0x10
#define MC3XXX_PCODE_3433		0x60

// 1g constant value
#define GRAVITY_1G_VALUE		1000


#define REMAP_IF_MC3250_READ(nDataX, nDataY) \
            if (MC3XXX_PCODE_3250 == s_bPCODE)          \
            {                                                                         \
                int    _nTemp = 0;                                           \
                                                                                       \
                _nTemp = nDataX;                                         \
                nDataX = nDataY;                                         \
                nDataY = -_nTemp;                                      \
            }

#define REMAP_IF_MC3250_WRITE(nDataX, nDataY) \
            if (MC3XXX_PCODE_3250 == s_bPCODE)            \
            {                                                                           \
                int    _nTemp = 0;                                             \
                                                                                         \
                _nTemp = nDataX;                                           \
                nDataX = -nDataY;                                         \
                nDataY = _nTemp;                                          \
            }

#define REMAP_IF_MC34XX_N(nDataX, nDataY)        \
            if ((MC3XXX_PCODE_3410N == s_bPCODE) ||  \
                 (MC3XXX_PCODE_3430N == s_bPCODE))     \
            {                                                                          \
                nDataX = -nDataX;                                        \
                nDataY = -nDataY;                                       \
            }

#define IS_MC35XX()                                                  \
            ((MC3XXX_PCODE_3510B == s_bPCODE) ||   \
             (MC3XXX_PCODE_3510C == s_bPCODE) ||   \
             (MC3XXX_PCODE_3530B == s_bPCODE) ||   \
             (MC3XXX_PCODE_3433 == (s_bPCODE & 0xF1)))

#define REMAP_IF_MC35XX(nDataX, nDataY)          \
            if (IS_MC35XX())                                            \
            {                                                                       \
                nDataX = -nDataX;                                     \
                nDataY = -nDataY;                                    \
            }

enum mc3xxx_orientation {
	MC3XXX_TOP_LEFT_DOWN = 0,		// 0: top, left-down
	MC3XXX_TOP_RIGHT_DOWN,		// 1: top, reight-down
	MC3XXX_TOP_RIGHT_UP,			// 2: top, right-up
	MC3XXX_TOP_LEFT_UP,			// 3: top, left-up
	MC3XXX_BOTTOM_LEFT_DOWN,		// 4: bottom, left-down
	MC3XXX_BOTTOM_RIGHT_DOWN,	// 5: bottom, right-down
	MC3XXX_BOTTOM_RIGHT_UP,		// 6: bottom, right-up
	MC3XXX_BOTTOM_LEFT_UP		// 7: bottom, left-up
};

struct mc3xxx_hwmsen_convert {
	signed char sign[3];
	unsigned char map[3];
};

enum mc3xxx_axis {
	MC3XXX_AXIS_X = 0,
	MC3XXX_AXIS_Y,
	MC3XXX_AXIS_Z,
	MC3XXX_AXIS_NUM
};

struct mc3xxxacc {
	signed short x, y, z;
};

struct mc3xxx_data {
	struct i2c_client *mc3xxx_client;
	struct sensors_classdev cdev;
	atomic_t delay;
	atomic_t enable;
	atomic_t selftest_result;
	struct input_dev *input;
	struct mc3xxxacc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int IRQ;
	unsigned char mode;
	unsigned char orientation;
	unsigned char data_trace_enable;
	struct regulator *vdd;
	bool power_enabled;
	int vpp;
	//unsigned int gain;	
};

#ifdef MCUBE_DOT_CALIBRATION
#define GSENSOR									0x95
//#define GSENSOR_IOCTL_INIT					_IO(GSENSOR,  0x01)
//#define GSENSOR_IOCTL_READ_CHIPINFO			_IOR(GSENSOR, 0x02, int)
#define GSENSOR_IOCTL_READ_SENSORDATA			_IOR(GSENSOR, 0x03, int)
//#define GSENSOR_IOCTL_READ_OFFSET				_IOR(GSENSOR, 0x04, GSENSOR_VECTOR3D)
//#define GSENSOR_IOCTL_READ_GAIN				_IOR(GSENSOR, 0x05, GSENSOR_VECTOR3D)
#define GSENSOR_IOCTL_READ_RAW_DATA				_IOR(GSENSOR, 0x06, int)
//#define GSENSOR_IOCTL_SET_CALI				_IOW(GSENSOR, 0x06, SENSOR_DATA)
#define GSENSOR_IOCTL_GET_CALI					_IOW(GSENSOR, 0x07, SENSOR_DATA)
#define GSENSOR_IOCTL_CLR_CALI					_IO(GSENSOR, 0x08)
#define GSENSOR_MCUBE_IOCTL_READ_RBM_DATA		_IOR(GSENSOR, 0x09, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_SET_RBM_MODE		_IO(GSENSOR, 0x0a)
#define GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE		_IO(GSENSOR, 0x0b)
#define GSENSOR_MCUBE_IOCTL_SET_CALI			_IOW(GSENSOR, 0x0c, SENSOR_DATA)
#define GSENSOR_MCUBE_IOCTL_REGISTER_MAP		_IO(GSENSOR, 0x0d)
#define GSENSOR_IOCTL_SET_CALI_MODE				_IOW(GSENSOR, 0x0e,int)
#define GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID		_IOR(GSENSOR, 0x0f, int)
#define GSENSOR_MCUBE_IOCTL_READ_FILEPATH		_IOR(GSENSOR, 0x10, char[256])

typedef struct{
	int x;
	int y;
	int z;
} SENSOR_DATA;

//#define CALIB_PATH			"/data/data/com.mcube.acc/files/mcube-calib.txt"
#define DATA_PATH	    "/sdcard/mcube-register-map.txt"
static char file_path[128] = "/data/data/com.mcube.acc/files/mcube-calib.txt";
//static char file_path[128] = "/productinfo/mcube-calib.txt";
//static char factory_path[128] ="/data/data/com.mcube.acc/files/fac-calib.txt";

//static GSENSOR_VECTOR3D mc3xxx_gain;
static SENSOR_DATA gain_data = {0}, offset_data = {0};
static unsigned char offset_buf[9] = {0};
static unsigned char offset_curr[6] = {0};
static struct file *fd_file = NULL;
static int load_cali_cnt = 30;
static bool IsRbmMode = false;
static mm_segment_t oldfs = 0;
static SENSOR_DATA mc3xxx_cali_data = {0};
#endif

static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
} u_i2c_addr = {{0x00},};

static unsigned char s_bPCODE = 0;
static unsigned short	 s_uiGain = 0;
static struct miscdevice mc3xxx_device;

static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on);
static int mc3xxx_chip_init(struct i2c_client *client);

static struct sensors_classdev sensors_cdev = {
		.name = "mc3xxx",
		.vendor = "mCube",
		.version = 1,
		.handle = SENSORS_ACCELERATION_HANDLE,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.max_range = "19.6",	/* 2g */
		.resolution = "0.156",	/* 15.63mg */
		.sensor_power = "0.13",	/* typical value */
		.min_delay = POLL_INTERVAL_MIN_MS * 1000, /* in microseconds */
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = POLL_DEFAULT_INTERVAL_MS, /* in millisecond */
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
		.sensors_self_test = NULL,
};

#define MC3XXX_DEFAULT_PLACEMENT		MC3XXX_TOP_RIGHT_DOWN

static const struct mc3xxx_hwmsen_convert mc3xxx_cvt[] = {
	{{   1,    1,    1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 0: top   , left-down
	{{ -1,    1,    1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 1: top   , right-down
	{{ -1,  -1,    1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 2: top   , right-up
	{{   1,  -1,    1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 3: top   , left-up
	{{ -1,    1,  -1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 4: bottom, left-down
	{{   1,    1,  -1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 5: bottom, right-down
	{{   1,  -1,  -1}, { MC3XXX_AXIS_X,  MC3XXX_AXIS_Y,  MC3XXX_AXIS_Z}}, // 6: bottom, right-up
	{{ -1,  -1,  -1}, { MC3XXX_AXIS_Y,  MC3XXX_AXIS_X,  MC3XXX_AXIS_Z}}, // 7: bottom, left-up
};

static const unsigned short mc3xxx_data_resolution[4] = {
	256, // +/- 2g, 10bit
	128, // +/- 4g, 10bit
	64,  // +/- 8g, 10bit
	1024 // +/- 8g, 14bit
};



#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h);
static void mc3xxx_early_resume(struct early_suspend *h);
#endif

static int mc3xxx_smbus_read_byte(struct i2c_client *client,
				  unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int mc3xxx_smbus_write_byte(struct i2c_client *client,
				   unsigned char reg_addr, unsigned char data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, data);
	if (dummy < 0)
		return -1;

	return 0;
}

static int mc3xxx_smbus_read_block(struct i2c_client *client,
					unsigned char reg_addr,
					unsigned char len, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}


#ifdef MCUBE_DOT_CALIBRATION
static int mc3xxx_smbus_write_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char len, unsigned char *data)
{
	signed int dummy = 0;

	dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return dummy;
		
	return 0;
}
#endif

static bool mc3xxx_validate_pcode(unsigned char bPCode)
{
    if (   (MC3XXX_PCODE_3210  == bPCode) || (MC3XXX_PCODE_3230  == bPCode)
        || (MC3XXX_PCODE_3250  == bPCode)
        || (MC3XXX_PCODE_3410  == bPCode) || (MC3XXX_PCODE_3430  == bPCode)
        || (MC3XXX_PCODE_3410N == bPCode) || (MC3XXX_PCODE_3430N == bPCode)
        || (MC3XXX_PCODE_3510B == bPCode) || (MC3XXX_PCODE_3530B == bPCode)
        || (MC3XXX_PCODE_3510C == bPCode) || (MC3XXX_PCODE_3433 == (bPCode & 0xF1)) )
    {
        return true;
    }    

    return false;
}

static bool mc3xxx_is_high_end(unsigned char pcode)
{
	if ((MC3XXX_PCODE_3230 == pcode) || (MC3XXX_PCODE_3430 == pcode) ||
	     (MC3XXX_PCODE_3430N == pcode) || (MC3XXX_PCODE_3530B == pcode) ||
	     (MC3XXX_PCODE_3433 == (pcode & 0xF1)))
		return false;
	else
		return true;
}

static bool mc3xxx_is_mc3510(unsigned char pcode)
{
	if ((MC3XXX_PCODE_3510B == pcode) || (MC3XXX_PCODE_3510C == pcode))
		return true;
	else
		return false;
}

static bool mc3xxx_is_mc3530(unsigned char pcode)
{
	if ((MC3XXX_PCODE_3530B == pcode) || (MC3XXX_PCODE_3433 == (pcode & 0xF1)))
		return true;
	else
		return false;
}

static int mc3xxx_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data;

	MC_PRINT("%s called\n", __func__);

	if (4 > mode) {
		data = 0x40 | mode;
		comres = mc3xxx_smbus_write_byte(client, MC3XXX_REG_MODE, data);
	} else {
		comres = -1;
	}

	return comres;
}

#ifdef MC3XXX_ENABLE_INT
static int mc3xxx_set_int_enable(struct i2c_client *client,
				 unsigned char InterruptType,
				 unsigned char value)
{
}
#endif /* MC3XXX_ENABLE_INT */

static int mc3xxx_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	MC_PRINT("%s called\n", __func__);

	comres = mc3xxx_smbus_read_byte(client, MC3XXX_REG_MODE, mode);
	*mode &= 0x03;

	return comres;
}

static int mc3xxx_set_range(struct i2c_client *client, unsigned char range)
{
	int comres = 0;
	unsigned char data = 0;

	MC_PRINT("%s called\n", __func__);

	if (4 > range) {
		if (mc3xxx_is_mc3510(s_bPCODE)) {
			data = 0x25;
			comres = mc3xxx_smbus_write_byte(client, MC3XXX_REG_OUTCFG, data);
			if (0 == comres)
				s_uiGain = 1024;
			return comres;
		} else if (mc3xxx_is_mc3530(s_bPCODE))	{
			data = 0x02;
			comres = mc3xxx_smbus_write_byte(client, MC3XXX_REG_OUTCFG, data);
			if (0 == comres)
				s_uiGain = 64;
			return comres;
		}

		if (mc3xxx_is_high_end(s_bPCODE)) {
			data = (range << 2) | 0x33;
			comres = mc3xxx_smbus_write_byte(client, MC3XXX_REG_OUTCFG, data);
			if (0 == comres)
				s_uiGain = mc3xxx_data_resolution[range];
		}
		else {
			//data = 0x32;
			s_uiGain = 86;
		}
		
	} else {
		comres = -1 ;
	}

	return comres;
}

static int mc3xxx_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data;

	MC_PRINT("%s called\n", __func__);

	comres = mc3xxx_smbus_read_byte(client, MC3XXX_REG_OUTCFG, &data);
	*range = ((data >> 2) & 0x03);

	return comres;
}

static int mc3xxx_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data = 0;

	MC_PRINT("%s called\n", __func__);

	if (7 > BW) {
		comres = mc3xxx_smbus_read_byte(client, MC3XXX_REG_OUTCFG, &data);
		data &= ~(0x07 << 4);
		data |= (BW << 4);
		comres += mc3xxx_smbus_write_byte(client, MC3XXX_REG_OUTCFG, data);
	} else {
		comres = -1 ;
	}

	return comres;
}

static int mc3xxx_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data = 0;

	MC_PRINT("%s called\n", __func__);

	comres = mc3xxx_smbus_read_byte(client, MC3XXX_REG_OUTCFG, &data);
	*BW = ((data >> 4) & 0x07);

	return comres;
}

#if defined(MC3XXX_ENABLE_INT)
#endif /* defined(MC3XXX_ENABLE_INT) */

static int mc3xxx_read_accel_xyz(struct mc3xxx_data *mc3xxx,
				 struct mc3xxxacc *acc)
{
	int comres;
	unsigned char data[6];
	signed short raw[3] = { 0 };
	const struct mc3xxx_hwmsen_convert *pCvt;

	//MC_PRINT("%s called\n", __func__);

	if (true == mc3xxx_is_high_end(s_bPCODE)) {
		comres = mc3xxx_smbus_read_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOUT_EX_L, 6, data);
		raw[0] = (signed short)(data[0] + (data[1] << 8));
		raw[1] = (signed short)(data[2] + (data[3] << 8));
		raw[2] = (signed short)(data[4] + (data[5] << 8));
	} else {
		comres = mc3xxx_smbus_read_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOUT, 3, data);
		raw[0] = (signed char)data[0];
		raw[1] = (signed char)data[1];
		raw[2] = (signed char)data[2];
	}

	if (comres)
	{
		printk("%s: i2c error!\n", __func__);
		return comres;
	}

	if (mc3xxx->data_trace_enable) {
		printk("%s: %d, %d, %d\n",__func__, raw[0], raw[1], raw[2]);
	}
	else {
		//MC_PRINT("%s: %d, %d, %d\n",__func__, raw[0], raw[1], raw[2]);
	}

	REMAP_IF_MC3250_READ(raw[0], raw[1]);
	REMAP_IF_MC34XX_N(raw[0], raw[1]);
	REMAP_IF_MC35XX(raw[0], raw[1]);

	pCvt = &mc3xxx_cvt[mc3xxx->orientation];
	acc->x = pCvt->sign[MC3XXX_AXIS_X] * raw[pCvt->map[MC3XXX_AXIS_X]];
	acc->y = pCvt->sign[MC3XXX_AXIS_Y] * raw[pCvt->map[MC3XXX_AXIS_Y]];
	acc->z = pCvt->sign[MC3XXX_AXIS_Z] * raw[pCvt->map[MC3XXX_AXIS_Z]];


	acc->x = acc->x * GRAVITY_1G_VALUE / s_uiGain;
	acc->y = acc->y * GRAVITY_1G_VALUE / s_uiGain;
	acc->z = acc->z * GRAVITY_1G_VALUE / s_uiGain;

	return comres;
}

#ifdef MCUBE_DOT_CALIBRATION
static void mc3xxx_read_cali_file(struct mc3xxx_data *mc3xxx);
static int mc3xxx_read_true_data(struct mc3xxx_data *mc3xxx, struct mc3xxxacc *acc);

static struct file *openFile(char *path, int flag, int mode) 
{ 
	struct file *fp = NULL; 
	 
	fp=filp_open(path, flag, mode); 
	if (IS_ERR(fp) || !fp->f_op) 
	{
		printk("%s: open return NULL!\n", __func__);
		return NULL; 
	}
	else 
	{
		return fp; 
	}
} 
 
static int readFile(struct file *fp, char *buf, int readlen) 
{ 
	if (fp->f_op && fp->f_op->read) 
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
	else 
		return -1; 
} 

static int writeFile(struct file *fp, char *buf, int writelen) 
{ 
	if (fp->f_op && fp->f_op->write) 
		return fp->f_op->write(fp,buf,writelen, &fp->f_pos); 
	else 
		return -1; 
}
 
static int closeFile(struct file *fp) 
{ 
	filp_close(fp,NULL); 
	return 0; 
} 

static void initKernelEnv(void) 
{ 
	oldfs = get_fs(); 
	set_fs(KERNEL_DS);
} 

static int mc3xxx_read_rbm_xyz(struct mc3xxx_data *mc3xxx, struct mc3xxxacc *acc)
{
	int comres = -1;
	unsigned char data[6] = { 0 };
	s16 raw[3] = { 0 };
	const struct mc3xxx_hwmsen_convert *pCvt = NULL;

	if ((0 == gain_data.x) || (0 == gain_data.y) || (0 == gain_data.z))
	{
		acc->x = 0;
		acc->y = 0;
		acc->z = 0;
		return 0;
	}

	comres = mc3xxx_smbus_read_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOUT_EX_L, 6, data);
	if (comres)
	{
		printk("%s: i2c error!\n", __func__);
		return comres;
	}
	
	raw[0] = (s16)(data[0] + (data[1] << 8));
	raw[1] = (s16)(data[2] + (data[3] << 8));
	raw[2] = (s16)(data[4] + (data[5] << 8));

	if (mc3xxx->data_trace_enable)
		printk("%s: %d, %d, %d\n",__func__, raw[0], raw[1], raw[2]);

	raw[0] = (raw[0] + offset_data.x / 2) * s_uiGain / gain_data.x;
	raw[1] = (raw[1] + offset_data.y / 2) * s_uiGain / gain_data.y;
	raw[2] = (raw[2] + offset_data.z / 2) * s_uiGain / gain_data.z;
	
	raw[0] = raw[0] * GRAVITY_1G_VALUE / s_uiGain;
	raw[1] = raw[1] * GRAVITY_1G_VALUE / s_uiGain;
	raw[2] = raw[2] * GRAVITY_1G_VALUE / s_uiGain;
	
	REMAP_IF_MC3250_READ(raw[0], raw[1]);
	REMAP_IF_MC34XX_N(raw[0], raw[1]);
	REMAP_IF_MC35XX(raw[0], raw[1]);

	pCvt = &mc3xxx_cvt[mc3xxx->orientation];
	acc->x = pCvt->sign[MC3XXX_AXIS_X] * raw[pCvt->map[MC3XXX_AXIS_X]];
	acc->y = pCvt->sign[MC3XXX_AXIS_Y] * raw[pCvt->map[MC3XXX_AXIS_Y]];
	acc->z = pCvt->sign[MC3XXX_AXIS_Z] * raw[pCvt->map[MC3XXX_AXIS_Z]];

	return comres;
}

static int mc3xxx_read_true_data(struct mc3xxx_data *mc3xxx, struct mc3xxxacc *acc)
{
	int err = 0;

	//MC_PRINT("%s called: %d\n", __func__, IsRbmMode);
	
	if (true == IsRbmMode)
	{
		err = mc3xxx_read_rbm_xyz(mc3xxx, acc);
	}
	else
	{
		err = mc3xxx_read_accel_xyz(mc3xxx, acc);
	}

	if (err)
	{
		printk("%s: read error!\n", __func__);
		return err;
	}

	return err;
}

static int mc3xxx_read_raw_data(struct mc3xxx_data *mc3xxx, char *buf)
{
	int err = 0;
	struct mc3xxxacc acc = { 0 };

	MC_PRINT("%s called\n", __func__);
	
	if (!buf)
	{
		printk("%s: invalid buffer pointer!\n", __func__);
		return -EINVAL;
	}

	err = mc3xxx_read_true_data(mc3xxx, &acc);

	if (err)
	{
		printk("%s: read error!\n", __func__);
		return err;
	}

	sprintf(buf, "%04x %04x %04x", acc.x, acc.y, acc.z);

	return err;
}

static int mc3xxx_read_rbm_data(struct mc3xxx_data *mc3xxx, char *buf)
{
	int err = 0;
	struct mc3xxxacc acc = { 0 };

	MC_PRINT("%s called\n", __func__);
	
	if (!buf)
	{
		printk("%s: invalid buffer pointer!\n", __func__);
		return -EINVAL;
	}

	err = mc3xxx_read_rbm_xyz(mc3xxx, &acc);

	if (err)
		printk("%s: read error!\n", __func__);

	sprintf(buf, "%04x %04x %04x", acc.x, acc.y, acc.z);

	return err;
}

static void mc3xxx_get_offset_gain(u8 *buf, SENSOR_DATA *pOffset, SENSOR_DATA *pGain)
{
	s16 tmp = 0;
    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;

	if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
    }

	// get x/y/z offset
    tmp = ((buf[1] & bMsbFilter) << 8) + buf[0];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    pOffset->x = tmp;
    
    tmp = ((buf[3] & bMsbFilter) << 8) + buf[2];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    pOffset->y = tmp;
    
    tmp = ((buf[5] & bMsbFilter) << 8) + buf[4];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    pOffset->z = tmp;
					
	// get x/y/z gain
	pGain->x = ((buf[1] >> 7) << 8) + buf[6];
	pGain->y = ((buf[3] >> 7) << 8) + buf[7];
	pGain->z = ((buf[5] >> 7) << 8) + buf[8];
}

static int mc3xxx_offset_over_range(int offset)
{
    int dwRangePosLimit  = 0x1FFF;
    int dwRangeNegLimit  = -0x2000;

	if (IS_MC35XX())
    {
        dwRangePosLimit  = 0x3FFF;
        dwRangeNegLimit  = -0x4000;
    }

	if (offset > dwRangePosLimit) 
	{
		offset = dwRangePosLimit;
	}
	else if (offset < dwRangeNegLimit)
	{
		offset = dwRangeNegLimit;
	}
	
	return offset;
}

static int mc3xxx_write_calibration( struct mc3xxx_data *mc3xxx, const SENSOR_DATA *pSensorData)
{
	int err = 0;
	u8 buf[9] = { 0 };
	//int tmp = 0;
	int raw[3] = { 0 };
	SENSOR_DATA offset = { 0 }, gain = { 0 };
	struct mc3xxxacc acc = { 0 };
	const struct mc3xxx_hwmsen_convert *pCvt = NULL;
    u8  bMsbFilter       = 0x3F;

	if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
    }
	
	MC_PRINT("%s called: %d, %d, %d\n", __func__, pSensorData->x, pSensorData->y, pSensorData->z);

	err = mc3xxx_smbus_read_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOFFL, 9, buf);
	if (err)
	{
		printk("%s: read error!\n", __func__);
		return err;
	}

	acc.x = pSensorData->x;
	acc.y = pSensorData->y;
	acc.z = pSensorData->z;

	// get raw from dat
	pCvt = &mc3xxx_cvt[mc3xxx->orientation];
	raw[pCvt->map[MC3XXX_AXIS_X]] = pCvt->sign[MC3XXX_AXIS_X] * acc.x;
	raw[pCvt->map[MC3XXX_AXIS_Y]] = pCvt->sign[MC3XXX_AXIS_Y] * acc.y;
	raw[pCvt->map[MC3XXX_AXIS_Z]] = pCvt->sign[MC3XXX_AXIS_Z] * acc.z;
	raw[MC3XXX_AXIS_X] = raw[MC3XXX_AXIS_X] * s_uiGain / GRAVITY_1G_VALUE;
	raw[MC3XXX_AXIS_Y] = raw[MC3XXX_AXIS_Y] * s_uiGain / GRAVITY_1G_VALUE;
	raw[MC3XXX_AXIS_Z] = raw[MC3XXX_AXIS_Z] * s_uiGain / GRAVITY_1G_VALUE;

	REMAP_IF_MC3250_WRITE(raw[0], raw[1]);
	REMAP_IF_MC34XX_N(raw[0], raw[1]);
	REMAP_IF_MC35XX(raw[0], raw[1]);

	// get offset and gain
	mc3xxx_get_offset_gain(buf, &offset, &gain);
	MC_PRINT("mc3xxx_write_calibration og: %x, %x, %x, %x, %x, %x\n",
			offset.x, offset.y, offset.z, gain.x, gain.y, gain.z);
	
	// prepare new offset
	offset.x = offset.x + 16 * raw[MC3XXX_AXIS_X] * 256 * 128 / 3 / s_uiGain / (40 + gain.x);
	offset.y = offset.y + 16 * raw[MC3XXX_AXIS_Y] * 256 * 128 / 3 / s_uiGain / (40 + gain.y);
	offset.z = offset.z + 16 * raw[MC3XXX_AXIS_Z] * 256 * 128 / 3 / s_uiGain / (40 + gain.z);
	
	// over range check
	offset.x = mc3xxx_offset_over_range(offset.x);
	offset.y = mc3xxx_offset_over_range(offset.y);
	offset.z = mc3xxx_offset_over_range(offset.z);

	// write offset registers
	err = mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_STANDBY);
	
	buf[0] = offset.x & 0xff;
	buf[1] = ((offset.x >> 8) & bMsbFilter) | (gain.x & 0x0100 ? 0x80 : 0);
	buf[2] = offset.y & 0xff;
	buf[3] = ((offset.y >> 8) & bMsbFilter) | (gain.y & 0x0100 ? 0x80 : 0);
	buf[4] = offset.z & 0xff;
	buf[5] = ((offset.z >> 8) & bMsbFilter) | (gain.z & 0x0100 ? 0x80 : 0);

	memcpy(offset_curr, buf, 6);
	err += mc3xxx_smbus_write_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOFFL, 6, buf);

	err += mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_WAKE);

	if (err)
		printk("%s: write error!\n", __func__);

	// save offset and gain of DOT format for later use
	offset_data.x = offset.x;
	offset_data.y = offset.y;
	offset_data.z = offset.z;

	gain_data.x = 256 * 8 * 128 / 3 / (40 + gain.x);
	gain_data.y = 256 * 8 * 128 / 3 / (40 + gain.y);
	gain_data.z = 256 * 8 * 128 / 3 / (40 + gain.z);
	
	msleep(50);

	return err;
}

static int mc3xxx_reset_calibration(struct mc3xxx_data *mc3xxx)
{
	int err = 0;

	MC_PRINT("%s called\n", __func__);

	err = mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_STANDBY);
	
	err += mc3xxx_smbus_write_block(mc3xxx->mc3xxx_client, MC3XXX_REG_XOFFL, 6, offset_buf);

	err += mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_WAKE);

	if (err)
		printk("%s: write error!\n", __func__);

	// save offset and gain of DOT format for later use
	mc3xxx_get_offset_gain(offset_buf, &offset_data, &gain_data);

	gain_data.x = 256 * 8 * 128 / 3 / (40 + gain_data.x);
	gain_data.y = 256 * 8 * 128 / 3 / (40 + gain_data.y);
	gain_data.z = 256 * 8 * 128 / 3 / (40 + gain_data.z);
	
	return err;
}

static int mc3xxx_soft_reset(struct i2c_client *client) 
{
	int err = 0;
	unsigned char tmp = 0;

	MC_PRINT("%s called\n", __func__);

	tmp = 0x6d;
	err = mc3xxx_smbus_write_byte(client, 0x1B, tmp);

	tmp = 0x43;
  	err += mc3xxx_smbus_write_byte(client, 0x1B, tmp);
	msleep(5);

	tmp = 0x43;
  	err += mc3xxx_smbus_write_byte(client, 0x07, tmp);

	tmp = 0x80;
  	err += mc3xxx_smbus_write_byte(client, 0x1C, tmp);

	tmp = 0x80;
  	err += mc3xxx_smbus_write_byte(client, 0x17, tmp);
	msleep(5);

	tmp = 0x00;
  	err += mc3xxx_smbus_write_byte(client, 0x1C, tmp);
	
	tmp = 0x00;
  	err += mc3xxx_smbus_write_byte(client, 0x17, tmp);
	msleep(5);

	memset(offset_buf, 0, sizeof(offset_buf));
	err += mc3xxx_smbus_read_block(client, MC3XXX_REG_XOFFL, 9, &offset_buf[0]);
	if (err)
	{
		printk("%s: read error!\n", __func__);
		return err;
	}

	// for early resume
	memcpy(offset_curr, offset_buf, 6);

	// save offset and gain of DOT format for later use
	mc3xxx_get_offset_gain(offset_buf, &offset_data, &gain_data);

	gain_data.x = 256 * 8 * 128 / 3 / (40 + gain_data.x);
	gain_data.y = 256 * 8 * 128 / 3 / (40 + gain_data.y);
	gain_data.z = 256 * 8 * 128 / 3 / (40 + gain_data.z);

	// initial calibration data
	mc3xxx_cali_data.x = 0;
	mc3xxx_cali_data.y = 0;
	mc3xxx_cali_data.z = 0;

	return err;
}

static void mc3xxx_read_calibration(struct mc3xxx_data *mc3xxx, SENSOR_DATA *pSensorData)
{
	MC_PRINT("%s called\n", __func__);

	pSensorData->x = mc3xxx_cali_data.x;
	pSensorData->y = mc3xxx_cali_data.y;
	pSensorData->z = mc3xxx_cali_data.z;
}

static int mc3xxx_rbm_mode(struct mc3xxx_data *mc3xxx, bool enable)
{
	int rc = 0; 
	unsigned char data = 0;

	MC_PRINT("%s called: %d\n", __func__, enable);

	rc = mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_STANDBY);
	rc += mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, 0x04, &data);
	
	if (0x00 == (data & 0x40))
	{
		data = 0x6D;
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x1B, data);
	    
		data = 0x43;
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x1B, data);
	}

	if (true == enable)
	{
		data = 0; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x3B, data);

		data = 0x02; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x14, data);

		IsRbmMode = 1;
	}
	else
	{
		data = 0; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x14, data);

		data = s_bPCODE; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x3B, data);

		IsRbmMode = 0;
	}

	rc += mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, 0x04, &data);

	if (data & 0x40)
	{
		data = 0x6D; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x1B, data);

		data = 0x43; 
		rc += mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client, 0x1B, data);
	}

	rc += mc3xxx_set_mode(mc3xxx->mc3xxx_client, MC3XXX_MODE_WAKE);

	msleep(220);
	
	return rc;
}

static void mc3xxx_read_cali_file(struct mc3xxx_data *mc3xxx)
{
	SENSOR_DATA cali_data = { 0 };
	int err = 0;
	char buf[64] = { 0 };

	//MC_PRINT("%s called\n", __func__);

	if (load_cali_cnt > 0)
	{
		load_cali_cnt --;
	}
	else
	{
		return;
	}
	
	initKernelEnv();
	//fd_file = openFile(CALIB_PATH, O_RDONLY, 0);
	fd_file = openFile(file_path, O_RDONLY, 0); 
	if (fd_file == NULL) 
	{
		printk("%s: fail to open!\n", __func__);
		cali_data.x = 0;
		cali_data.y = 0;
		cali_data.z = 0;
		return;
	}
	else
	{
		memset(buf, 0, sizeof(buf)); 
		if ((err = readFile(fd_file,buf,64)) <= 0) 
		{
			printk("%s: read file error %d!\n", __func__, err);
		}
		else
		{
			MC_PRINT("%s: %s\n", __func__, buf);
		}
		
		set_fs(oldfs); 
		closeFile(fd_file); 

		sscanf(buf, "%d %d %d", &cali_data.x, &cali_data.y, &cali_data.z);
				
		mc3xxx_write_calibration(mc3xxx, &cali_data);

		load_cali_cnt = 0;

		return;
	}
}

static int mc3xxx_write_log_data(const unsigned char data[64])
{
	#define _WRT_LOG_DATA_BUFFER_SIZE    (66 * 50)

	s16 rbm_data[3] = { 0 }, raw_data[3] = { 0 };
	int err = 0;
	char *_pszBuffer = NULL;
	int n = 0, i = 0;

	MC_PRINT("%s called\n", __func__);

	initKernelEnv();
	fd_file = openFile(DATA_PATH, O_RDWR | O_CREAT, 0); 
	if (fd_file == NULL) 
	{
		printk("%s: can't create file!\n", __func__);
		return -1;
	}
	else
	{
		rbm_data[0] = (s16)((data[0x0d]) | (data[0x0e] << 8));
		rbm_data[1] = (s16)((data[0x0f]) | (data[0x10] << 8));
		rbm_data[2] = (s16)((data[0x11]) | (data[0x12] << 8));

		raw_data[0] = (rbm_data[0] + offset_data.x/2) * s_uiGain / gain_data.x;
		raw_data[1] = (rbm_data[1] + offset_data.y/2) * s_uiGain / gain_data.y;
		raw_data[2] = (rbm_data[2] + offset_data.z/2) * s_uiGain / gain_data.z;

		_pszBuffer = kzalloc(_WRT_LOG_DATA_BUFFER_SIZE, GFP_KERNEL);		
		if (NULL == _pszBuffer)
		{
			printk("%s: fail to allocate memory for buffer!\n", __func__);
			return -1;
		}
		memset(_pszBuffer, 0, _WRT_LOG_DATA_BUFFER_SIZE); 

		n += sprintf(_pszBuffer+n, "G-sensor RAW X = %d  Y = %d  Z = %d\n", raw_data[0] ,raw_data[1] ,raw_data[2]);
		n += sprintf(_pszBuffer+n, "G-sensor RBM X = %d  Y = %d  Z = %d\n", rbm_data[0] ,rbm_data[1] ,rbm_data[2]);
		for(i=0; i<64; i++)
		{
			n += sprintf(_pszBuffer+n, "mCube register map Register[%x] = 0x%x\n",i,data[i]);
		}
		msleep(50);		
		if ((err = writeFile(fd_file,_pszBuffer,n)) <= 0) 
		{
 			printk("%s: write file error %d!\n", __func__, err); 
 		}

		kfree(_pszBuffer);

		set_fs(oldfs); 
		closeFile(fd_file); 
	}
	
	return 0;
}

static int mc3xxx_read_reg_map(struct mc3xxx_data *mc3xxx)
{
	u8 data[64] = { 0 };
	int err = 0;

	MC_PRINT("%s called\n", __func__);

	err = mc3xxx_smbus_read_block(mc3xxx->mc3xxx_client, 0, 64, data);

	if (err)
	{
		printk("%s: read reg fail!\n", __func__); 
		return err;
	}
	
	msleep(50);	
	
	mc3xxx_write_log_data(data);

	msleep(50);
     	
	return err;
}
#endif

static void mc3xxx_work_func(struct work_struct *work)
{
	struct mc3xxx_data *mc3xxx = container_of((struct delayed_work *)work,
						  struct mc3xxx_data, work);
	static struct mc3xxxacc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&mc3xxx->delay));

	//MC_PRINT("%s called\n", __func__);

#ifdef MCUBE_DOT_CALIBRATION
	mc3xxx_read_cali_file(mc3xxx);
	mc3xxx_read_true_data(mc3xxx, &acc);
#else
	mc3xxx_read_accel_xyz(mc3xxx, &acc);
#endif	
	input_report_abs(mc3xxx->input, ABS_X, acc.x);
	input_report_abs(mc3xxx->input, ABS_Y, acc.y);
	input_report_abs(mc3xxx->input, ABS_Z, acc.z);

	input_sync(mc3xxx->input);
	mutex_lock(&mc3xxx->value_mutex);
	mc3xxx->value = acc;
	mutex_unlock(&mc3xxx->value_mutex);
	schedule_delayed_work(&mc3xxx->work, delay);
}

static ssize_t mc3xxx_register_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int address, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	sscanf(buf, "%d%d", &address, &value);

	if (mc3xxx_smbus_write_byte(mc3xxx->mc3xxx_client,
			(unsigned char)address, (unsigned char)value) < 0)
		return -EINVAL;

	return count;
}
static ssize_t mc3xxx_register_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	size_t count = 0;
	u8 reg[0x3f];
	int i;

	for (i = 0; i < 0x3f; i++) {
		mc3xxx_smbus_read_byte(mc3xxx->mc3xxx_client, i, reg + i);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;

}
static ssize_t mc3xxx_range_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	if (mc3xxx_get_range(mc3xxx->mc3xxx_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t mc3xxx_range_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (mc3xxx_set_range(mc3xxx->mc3xxx_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mc3xxx_bandwidth_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	if (mc3xxx_get_bandwidth(mc3xxx->mc3xxx_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t mc3xxx_bandwidth_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (mc3xxx_set_bandwidth(mc3xxx->mc3xxx_client,
				 (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mc3xxx_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	if (mc3xxx_get_mode(mc3xxx->mc3xxx_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t mc3xxx_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (mc3xxx_set_mode(mc3xxx->mc3xxx_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mc3xxx_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct mc3xxx_data *mc3xxx = input_get_drvdata(input);
	struct mc3xxxacc acc_value;

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&mc3xxx->value_mutex);
	acc_value = mc3xxx->value;
	mutex_unlock(&mc3xxx->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
		       acc_value.z);
}

static ssize_t mc3xxx_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	return sprintf(buf, "%d\n", atomic_read(&mc3xxx->delay));

}

static ssize_t mc3xxx_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > MC3XXX_MAX_DELAY)
		data = MC3XXX_MAX_DELAY;
	atomic_set(&mc3xxx->delay, (unsigned int)data);

	return count;
}

static ssize_t mc3xxx_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	return sprintf(buf, "%d\n", atomic_read(&mc3xxx->enable));

}

static void mc3xxx_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&mc3xxx->enable);

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&mc3xxx->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			if (mc3xxx_power_ctl(mc3xxx, true)) {
				dev_err(dev, "power failed\n");
				goto mutex_exit;
			}
			
#ifdef MCUBE_DOT_CALIBRATION
			mc3xxx_soft_reset(mc3xxx->mc3xxx_client);
#endif

			if (mc3xxx_chip_init(mc3xxx->mc3xxx_client) < 0) {
				dev_err(dev, "set init failed\n");
				goto mutex_exit;
			}
			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_WAKE);
			schedule_delayed_work(&mc3xxx->work,
					      msecs_to_jiffies(atomic_read
							       (&mc3xxx->
								delay)));
			atomic_set(&mc3xxx->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			mc3xxx_set_mode(mc3xxx->mc3xxx_client,
					MC3XXX_MODE_STANDBY);
			cancel_delayed_work_sync(&mc3xxx->work);
			atomic_set(&mc3xxx->enable, 0);
			if (mc3xxx_power_ctl(mc3xxx, false)) {
				dev_err(dev, "power failed\n");
				goto mutex_exit;
			}
		}
	}
	
mutex_exit:
	mutex_unlock(&mc3xxx->enable_mutex);

}

static ssize_t mc3xxx_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long data;
	int error;

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		mc3xxx_set_enable(dev, data);

	return count;
}

static int mc3xxx_cdev_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct mc3xxx_data *data = container_of(sensors_cdev,
					struct mc3xxx_data, cdev);

	mc3xxx_set_enable(&data->mc3xxx_client->dev, enable);
	return 0;
}

static int mc3xxx_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct mc3xxx_data *data = container_of(sensors_cdev,
					struct mc3xxx_data, cdev);

	if (delay_ms < MC3XXX_MIN_DELAY)
		delay_ms = MC3XXX_MIN_DELAY;
	if (delay_ms > MC3XXX_MAX_DELAY)
		delay_ms = MC3XXX_MAX_DELAY;
	atomic_set(&data->delay, (unsigned int) delay_ms);

	return 0;
}

static ssize_t mc3xxx_orientation_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	return sprintf(buf, "%d\n", mc3xxx->orientation);
}

static ssize_t mc3xxx_orientation_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (8 > data)
		mc3xxx->orientation = data;

	return count;
}

static ssize_t mc3xxx_dte_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);

	MC_PRINT("%s called\n", __func__);
	return sprintf(buf, "%d\n", mc3xxx->data_trace_enable);
}

static ssize_t mc3xxx_dte_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	unsigned long data;
	int error;

	MC_PRINT("%s called\n", __func__);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	mc3xxx->data_trace_enable = data;

	return count;
}

static DEVICE_ATTR(range, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_range_show, mc3xxx_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_bandwidth_show, mc3xxx_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_mode_show, mc3xxx_mode_store);
static DEVICE_ATTR(value, S_IRUGO, mc3xxx_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_delay_show, mc3xxx_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_enable_show, mc3xxx_enable_store);
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_register_show, mc3xxx_register_store);
static DEVICE_ATTR(orientation, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_orientation_show, mc3xxx_orientation_store);
static DEVICE_ATTR(dte, S_IRUGO | S_IWUSR | S_IWGRP,
		   mc3xxx_dte_show, mc3xxx_dte_store);

static struct attribute *mc3xxx_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_reg.attr,
	&dev_attr_orientation.attr,
	&dev_attr_dte.attr,
	NULL
};

static struct attribute_group mc3xxx_attribute_group = {
	.attrs = mc3xxx_attributes
};

#if defined(MC3XXX_ENABLE_INT)
static void mc3xxx_irq_work_func(struct work_struct *work)
{
}

static irqreturn_t mc3xxx_irq_handler(int irq, void *handle)
{

	struct mc3xxx_data *data = handle;

	if (data == NULL)
		return IRQ_HANDLED;
	if (data->mc3xxx_client == NULL)
		return IRQ_HANDLED;

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;

}
#endif /* defined(MC3XXX_ENABLE_INT) */

static long mc3xxx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;

#ifdef MCUBE_DOT_CALIBRATION
    struct i2c_client *client = container_of(mc3xxx_device.parent, struct i2c_client, dev);
	struct mc3xxx_data *mc3xxx = i2c_get_clientdata(client);
	int temp = 0;
	char strbuf[256] = { 0 };
	SENSOR_DATA sensor_data = { 0 };
#endif

	switch (cmd) {
    #ifdef MCUBE_DOT_CALIBRATION
	case GSENSOR_IOCTL_READ_SENSORDATA:	
	case GSENSOR_IOCTL_READ_RAW_DATA:
		MC_PRINT("%s: GSENSOR_IOCTL_READ_SENSORDATA\n", __func__);
		mc3xxx_read_raw_data(mc3xxx, strbuf);
		if (copy_to_user(argp, strbuf, strlen(strbuf)+1))
		{
			printk("%s: read rawdata fail to copy!\n", __func__);
			return -EFAULT;
		}
		break;
	
	case GSENSOR_MCUBE_IOCTL_SET_CALI:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_SET_CALI\n", __func__);
		if (copy_from_user(&sensor_data, argp, sizeof(sensor_data)))
		{
			printk("%s: set cali fail to copy!\n", __func__);
			return -EFAULT;
		}
		else
		{
			mutex_lock(&mc3xxx->value_mutex);
			err = mc3xxx_write_calibration(mc3xxx, &sensor_data);			 
			mutex_unlock(&mc3xxx->value_mutex);
		}
		break;
		
	case GSENSOR_IOCTL_CLR_CALI:
		MC_PRINT("%s: GSENSOR_IOCTL_CLR_CALI\n", __func__);
		mutex_lock(&mc3xxx->value_mutex);
		err = mc3xxx_reset_calibration(mc3xxx);
		mutex_unlock(&mc3xxx->value_mutex);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		MC_PRINT("%s: GSENSOR_IOCTL_GET_CALI\n", __func__);
		mc3xxx_read_calibration(mc3xxx, &sensor_data);
		
		if (copy_to_user(argp, &sensor_data, sizeof(sensor_data)))
		{
			printk("%s: get cali fail to copy!\n", __func__);
			return -EFAULT;
		}		
		break;	

	case GSENSOR_IOCTL_SET_CALI_MODE:
		MC_PRINT("%s: GSENSOR_IOCTL_SET_CALI_MODE\n", __func__);
		break;

	case GSENSOR_MCUBE_IOCTL_READ_RBM_DATA:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_READ_RBM_DATA\n", __func__);
		mc3xxx_read_rbm_data(mc3xxx, strbuf);
		if (copy_to_user(argp, &strbuf, strlen(strbuf)+1)) {
			printk("%s: read rawdata fail to copy!\n", __func__);
			return -EFAULT;
		}
		break;

	case GSENSOR_MCUBE_IOCTL_SET_RBM_MODE:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_SET_RBM_MODE\n", __func__);
		mutex_lock(&mc3xxx->value_mutex);
		err = mc3xxx_rbm_mode(mc3xxx, 1);
		mutex_unlock(&mc3xxx->value_mutex);
		break;

	case GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE\n", __func__);
		mutex_lock(&mc3xxx->value_mutex);
		err = mc3xxx_rbm_mode(mc3xxx, 0);
		mutex_unlock(&mc3xxx->value_mutex);
		break;

	case GSENSOR_MCUBE_IOCTL_REGISTER_MAP:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_REGISTER_MAP\n", __func__);
		err = mc3xxx_read_reg_map(mc3xxx);
		break;

	case GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID\n", __func__);
		if(argp == NULL)
		{
			printk("%s: read product id null pointer!\n", __func__);
			return -EINVAL;
			break;	  
		}

		temp = mc3xxx_validate_pcode(s_bPCODE);

		if(copy_to_user(argp, &temp, sizeof(temp)))
		{
			printk("%s: read pcode fail to copy!\n", __func__);
			return -EFAULT;
		}
		break;

	case GSENSOR_MCUBE_IOCTL_READ_FILEPATH:
		MC_PRINT("%s: GSENSOR_MCUBE_IOCTL_READ_FILEPATH\n", __func__);
		if(copy_to_user(argp, file_path, (strlen(file_path)+1)))
		{
			printk("%s: read file path fail to copy!\n", __func__);
			return -EFAULT;
		}
		break;
#endif
		
	default:
		MC_PRINT("%s: can't recognize the cmd!\n", __func__);
		argp = argp; // disable compiling error
		return -EINVAL;
	}

	return err;
}

static int mc3xxx_open(struct inode *inode, struct file *filp)
{
	return nonseekable_open(inode, filp);
}

static int mc3xxx_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations mc3xxx_fops =
{
	.owner          = THIS_MODULE,
	.open       	= mc3xxx_open,
	.release    	= mc3xxx_release,
	.unlocked_ioctl = mc3xxx_ioctl,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mc3xxx_early_suspend(struct early_suspend *h)
{
	struct mc3xxx_data *data =
	    container_of(h, struct mc3xxx_data, early_suspend);

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
		cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->enable_mutex);
}

static void mc3xxx_early_resume(struct early_suspend *h)
{
	struct mc3xxx_data *data =
	    container_of(h, struct mc3xxx_data, early_suspend);

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_WAKE);
		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(atomic_read
						       (&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static struct miscdevice mc3xxx_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name  = MC3XXX_DEV_NAME,
	.fops  = &mc3xxx_fops,
};

/*****************************************
 *** gsensor_fetch_sysconfig_para
 *****************************************/
static int gsensor_fetch_sysconfig_para(void)
{
	u_i2c_addr.dirty_addr_buf[0] = MC3XXX_I2C_ADDR;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;

	return 0;
}

static bool mc3xxx_i2c_auto_probe(struct i2c_client *client)
{
	unsigned char    _baDataBuf[2] = {0};

	MC_PRINT("%s called.\n", __func__);

	s_bPCODE = 0x00;


		MC_PRINT("%s: probing addr is 0x%X.\n", __func__, client->addr);

		if (mc3xxx_smbus_read_byte(client, 0x3b, _baDataBuf))
		{
			printk("%s: 0x%X fail to communicate!\n", __func__, client->addr);
			return false;
		}

		MC_PRINT("%s: addr 0x%X ok to read REG(0x3B): 0x%X.\n", __func__, client->addr, _baDataBuf[0]);
		
#ifdef MCUBE_DOT_CALIBRATION
		if (0 == _baDataBuf[0]) {
			mc3xxx_soft_reset(client);
			if (mc3xxx_smbus_read_byte(client, 0x3b, _baDataBuf))
			{
				printk("%s: 0x%X fail to communicate after reset!\n", __func__, client->addr);
				return false;
			}
		}
#endif

		if (true == mc3xxx_validate_pcode(_baDataBuf[0]))
		{
			MC_PRINT("%s: addr 0x%X confirmed ok to use.\n", __func__, client->addr);

			s_bPCODE = _baDataBuf[0];

			return true;
		}

	return false;
}

static int mc3xxx_chip_init(struct i2c_client *client)
{
	unsigned char  _baDataBuf[2] = { 0 };

	_baDataBuf[0] = MC3XXX_REG_MODE;
	_baDataBuf[1] = 0x43;
	mc3xxx_smbus_write_byte(client, _baDataBuf[0], _baDataBuf[1]);
	
	_baDataBuf[0] = MC3XXX_REG_SRFA;
	_baDataBuf[1] = 0x00;
	if (IS_MC35XX())
		_baDataBuf[1] = 0x0A;
	mc3xxx_smbus_write_byte(client, _baDataBuf[0], _baDataBuf[1]);
	
	_baDataBuf[0] = MC3XXX_REG_INTEN;
	_baDataBuf[1] = 0x00;
	mc3xxx_smbus_write_byte(client, _baDataBuf[0], _baDataBuf[1]);

	mc3xxx_set_bandwidth(client, MC3XXX_BW_SET);
	mc3xxx_set_range(client, MC3XXX_RANGE_SET);

	_baDataBuf[0] = MC3XXX_REG_MODE;
	_baDataBuf[1] = 0x41;
	mc3xxx_smbus_write_byte(client, _baDataBuf[0], _baDataBuf[1]);

	return 0;
}


static int mc3xxx_power_ctl(struct mc3xxx_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		data->power_enabled = on;
	} else {
		dev_info(&data->mc3xxx_client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int mc3xxx_power_init(struct mc3xxx_data *data)
{
	int ret;

	data->vdd = regulator_get(&data->mc3xxx_client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->mc3xxx_client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				1700000,
				3400000);
		if (ret) {
			dev_err(&data->mc3xxx_client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int mc3xxx_power_deinit(struct mc3xxx_data *data)
{
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd,
				0, 3400000);

	regulator_put(data->vdd);

	return 0;
}

#ifdef CONFIG_OF
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_property_read_u32(np, "mc3xxx,delay", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read mc3xxx,delay\n");
		atomic_set(&pdata->delay, MC3XXX_MAX_DELAY);
	} else {
		atomic_set(&pdata->delay, temp_val);
	}

	printk(" * * * XCB * * * %s mc3xxx,delay = %d\n", __func__, atomic_read(&pdata->delay));
	
	rc = of_property_read_u32(np, "mc3xxx,orientation", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read sensor orientation paramater\n");
		return rc;
	}
	
	if (temp_val > 7 || temp_val < 0) {
		dev_err(dev, "Invalid orientation parameter, use default value 0\n");
		pdata->orientation = 0;
	} else {
		pdata->orientation = temp_val;
	}

	printk(" * * * XCB * * * %s mc3xxx,orientation = %u\n", __func__, pdata->orientation);

	pdata->vpp = of_get_named_gpio(dev->of_node, "mc3xxx,vpp", 0);
	
	pdata->IRQ = of_get_named_gpio_flags(dev->of_node,
				"mc3xxx,IRQ", 0, NULL);

	return 0;
}
#else
static int mc3xxx_parse_dt(struct device *dev,
			struct mc3xxx_data *pdata)
{
	return -EINVAL;
}
#endif

static int mc3xxx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct mc3xxx_data *data;
	struct input_dev *dev;
	//struct device_node *np;

	printk(" * * * XCB * * * %s: probing addr is 0x%X.\n", __func__, client->addr);
	
	MC_PRINT("%s called.\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		printk("%s: i2c_check_functionality error!\n", __func__);



	data = kzalloc(sizeof(struct mc3xxx_data), GFP_KERNEL);
	if (!data) {
		printk("%s: alloc memory error!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	if (client->dev.of_node) {
		err = mc3xxx_parse_dt(&client->dev, data);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto kfree_exit;
		}
	}
	
	gpio_request(data->vpp, "mc3xxx,vpp");
	
	gpio_direction_output(data->vpp, 0 );

	gpio_set_value(data->vpp, 0);


	data->mc3xxx_client = client;	
	i2c_set_clientdata(client, data);


	err = mc3xxx_power_init(data);
	if (err) {
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto kfree_exit;
	}
	err = mc3xxx_power_ctl(data, true);
	if (err) {
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto deinit_power_exit;
	}
	
	if (true != mc3xxx_i2c_auto_probe(client))
	{
		printk("%s: fail to probe mCube g-sensor!\n", __func__);
		err=-ENXIO;
		goto disable_power_exit;
	}

#ifdef MCUBE_DOT_CALIBRATION
	mc3xxx_soft_reset(client);
	if (true != mc3xxx_i2c_auto_probe(client))
	{
		printk("%s: fail to confirm mCube g-sensor!\n", __func__);\
             err=-ENXIO;
		goto disable_power_exit;
	}
#endif

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	data->data_trace_enable = 0;

	mc3xxx_chip_init(client);

#if defined(MC3XXX_ENABLE_INT)
#endif

	INIT_DELAYED_WORK(&data->work, mc3xxx_work_func);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev){
		printk("%s: fail to allocate device!\n", __func__);
		err = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}
		
	set_bit(EV_ABS, dev->evbit);

	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);
	//set_bit(EV_REL, dev->evbit);
	//set_bit(ABS_X, dev->relbit);
	//set_bit(ABS_Y, dev->relbit);
	//set_bit(ABS_Z, dev->relbit);

	//dev->name = MC3XXX_INPUT_NAME;
	dev->name = MC3XXX_DEV_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		goto kfree_exit;
	}
 
	data->input = dev;
	data->input->dev.parent = &data->mc3xxx_client->dev;

	mc3xxx_device.parent = &client->dev;
	
	err = misc_register(&mc3xxx_device);
	if (err) {
		printk("%s: fail to misc register!\n", __func__);
		goto exit_misc_device_register_failed;
	}

	//err = sysfs_create_group(&client->dev.kobj, &mc3xxx_attribute_group);
	err = sysfs_create_group(&data->input->dev.kobj, &mc3xxx_attribute_group);
	if (err < 0){
		printk("%s: fail to create group!\n", __func__);
		goto error_sysfs;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mc3xxx_early_suspend;
	data->early_suspend.resume = mc3xxx_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	
	data->cdev = sensors_cdev;
	data->cdev.min_delay = POLL_INTERVAL_MIN_MS * 1000;
	data->cdev.delay_msec = atomic_read(&data->delay);;
	data->cdev.sensors_enable = mc3xxx_cdev_enable;
	data->cdev.sensors_poll_delay = mc3xxx_cdev_poll_delay;
	//data->cdev.sensors_self_test = bma2x2_self_calibration_xyz;

	err = sensors_classdev_register(&client->dev, &data->cdev);
	if (err) {
		dev_err(&client->dev, "create class device file failed!\n");
		err = -EINVAL;
		goto exit_misc_device_register_failed;
	}
	
	MC_PRINT("%s probe ok!\n", __func__);

	mc3xxx_power_ctl(data, false);
	return 0;

exit_misc_device_register_failed:
error_sysfs:
	input_unregister_device(data->input);

exit_input_dev_alloc_failed:
	
disable_power_exit:
	mc3xxx_power_ctl(data, false);
deinit_power_exit:
	mc3xxx_power_deinit(data);
	
kfree_exit:
       i2c_set_clientdata(client, NULL);
	kfree(data);
exit:
	pr_err("error ret=%d\n",err);
	return err;
}

static int /*__devexit*/ mc3xxx_remove(struct i2c_client *client)
{
	struct mc3xxx_data *data = i2c_get_clientdata(client);

	MC_PRINT("%s called.\n", __func__);

	sensors_classdev_unregister(&data->cdev);

	mc3xxx_set_enable(&client->dev, 0);
	mc3xxx_power_ctl(data, false);
	mc3xxx_power_deinit(data);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	sysfs_remove_group(&data->input->dev.kobj, &mc3xxx_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int mc3xxx_suspend(struct device *dev)
{
       struct mc3xxx_data *data =  dev_get_drvdata(dev);

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_STANDBY);
		cancel_delayed_work_sync(&data->work);
	}
	mutex_unlock(&data->enable_mutex);

	return 0;
}
static int mc3xxx_resume(struct device *dev)
{
       struct mc3xxx_data *data =  dev_get_drvdata(dev);

	MC_PRINT("%s called\n", __func__);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		mc3xxx_set_mode(data->mc3xxx_client, MC3XXX_MODE_WAKE);
		schedule_delayed_work(&data->work,
				      msecs_to_jiffies(atomic_read
						       (&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);

	return 0;
}
static const struct dev_pm_ops mc3xxx_pm_ops = {
        .suspend = mc3xxx_suspend,
       .resume  = mc3xxx_resume,
};
static const struct i2c_device_id mc3xxx_id[] = {
	{MC3XXX_DEV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mc3xxx_id);

static const struct of_device_id mc3xxx_of_match[] = {
	{.compatible = "mc3,mc3xxx",},
	{},
}

MODULE_DEVICE_TABLE(of, mc3xxx_of_match);

static struct i2c_driver mc3xxx_driver = {
 //   .class = I2C_CLASS_HWMON,
	.driver = {
                   .owner = THIS_MODULE,
                   .name = MC3XXX_DEV_NAME,
                   .pm = &mc3xxx_pm_ops,
                   //  .of_match_table = mc3xxx_of_match,
		   },
       //.suspend = mc3xxx_suspend,
       //.resume = mc3xxx_resume,
	.id_table = mc3xxx_id,
	.probe = mc3xxx_probe,
	.remove = mc3xxx_remove,
    //.shutdown	  = mc3xxx_shutdown,
 //   .address_list  = u_i2c_addr.normal_i2c,
};

static int __init mc3xxx_init(void)
{
	MC_PRINT("%s called.\n", __func__);
	printk("%s:  * * * XCB * * * \n", __func__);
	if (gsensor_fetch_sysconfig_para())
	{
		printk("%s: gsensor_fetch_sysconfig_para err.\n", __func__);
		return -1;
	}
      return i2c_add_driver(&mc3xxx_driver);
}
static void __exit mc3xxx_exit(void)
{
	MC_PRINT("%s called.\n", __func__);

	i2c_del_driver(&mc3xxx_driver);
}


module_init(mc3xxx_init);
module_exit(mc3xxx_exit);

MODULE_DESCRIPTION("mc3xxx accelerometer driver");
MODULE_AUTHOR("mCube-inc");
MODULE_LICENSE("GPL");
MODULE_VERSION(MC3XXX_DRIVER_VERSION);

