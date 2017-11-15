#include <linux/sensors.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/delay.h>


#define MSG21XX_I2C_ADDR     0X26

extern int msg2138_ts_resume(struct device *dev);
extern void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size);
 extern int HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size);
extern  struct i2c_client  *msg21xx_i2c_client;
extern  u8 tpd_proximity_flag2;
#define VPS_NAME "virtual-proximity"

struct virtualpsensor2 *vps2;

struct virtualpsensor2 {
	char const *name;
//	struct i2c_client *client;
	struct input_dev *virtualdevice;
	bool vps_enabled;
	struct sensors_classdev vps_cdev;
	bool virtual_proximity_data;
	/* [FEATURE]-MOD-BEGIN by TCTNB.ZXZ, PR-893958 ,2015/01/05, sensor HAL mod by TCTNB, flash black screen when making first call */
	int active_ps_first;
	/* [FEATURE]-MOD-END by TCTNB.ZXZ*/
};


 struct sensors_classdev virtual_sensors_proximity_cdev2 = {
	.name = VPS_NAME,
	.vendor = "NULL",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

void set_psensor_register(int status)
{
	unsigned char tx_data[4];
	
	tx_data[0] = 0x52;
	tx_data[1] = 0x00;
	tx_data[2] = 0x4a;
	if(1==status)
		tx_data[3] = 0xa0;
	else
		tx_data[3] = 0xa1;

	HalTscrCDevWriteI2CSeq (MSG21XX_I2C_ADDR, tx_data, 4);
	msleep(150);
}


u8 get_psensor_register(void)
{
	
	unsigned char dbbus_rx_data[2];
	unsigned char tx_data[3];
	u8 status;
	
	tx_data[0] = 0x53;
	tx_data[1] = 0x00;
	tx_data[2] = 0x4a;
	
	HalTscrCDevWriteI2CSeq (MSG21XX_I2C_ADDR, tx_data, 3);

 	HalTscrCReadI2CSeq(MSG21XX_I2C_ADDR, &dbbus_rx_data[0], 2);
 
 	//open=0xa0,close=0xa1
	if(0xa0==dbbus_rx_data[0])
 		status=1;
	 else
  		status=0;

 	return status;
}



 int vps_set_enable2(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	u8 read_state;
	u8 set_state;
	//int ret = -1;
	int i;

	//if want to enable P-sensor should resume TP first
	if(enable)
	msg2138_ts_resume(&msg21xx_i2c_client->dev);

	set_state=enable;
	
	for(i=0;i<3;i++)
	{
		set_psensor_register(set_state);
		read_state=get_psensor_register();
		if(read_state==set_state)
		{
			vps2->vps_enabled = read_state;
			tpd_proximity_flag2 = vps2->vps_enabled;
			break;
		}

	}

/*[BUGFIX]-Begin Add by TCTNB.ZXZ, PR983680,2015/04/21, add for virtual Psensor can't wake up lcd when make call*/
	if ( 0 == enable )
	{
		device_init_wakeup(&msg21xx_i2c_client->dev, 0);
	}
	else if ( 1 == enable )
	{
		device_init_wakeup(&msg21xx_i2c_client->dev, 1);
	}
	else
	{
		device_init_wakeup(&msg21xx_i2c_client->dev, 0);
	}
/*[BUGFIX]-End Add by TCTNB.ZXZ*/


	/* [FEATURE]-MOD-BEGIN by TCTNB.ZXZ, PR-893958 ,2015/01/05, sensor HAL mod by TCTNB, flash black screen when making first call */
	if(1==enable)
	{
		if(0==vps2->active_ps_first)
		{
			input_report_abs(vps2->virtualdevice, ABS_DISTANCE, 1);
		       input_sync(vps2->virtualdevice);
			vps2->active_ps_first = 1;
		}
	}
	/* [FEATURE]-MOD-END by TCTNB.ZXZ*/

	return 0;

}

 int virtual_psensor_input_register2(struct i2c_client *pClient)
{
	s32 nRetVal = 0;

	pr_err("*** %s() ***\n", __func__);

	vps2 = kzalloc(sizeof(struct virtualpsensor2), GFP_KERNEL);
//	vps->client = pClient;
//	i2c_set_clientdata(pClient, vps);

	vps2->virtualdevice= input_allocate_device();
	if (vps2->virtualdevice == NULL)
	{
		pr_err("*** input device allocation failed ***\n");
		return -ENOMEM;
	}

	vps2->virtualdevice->name = "proximity";
	vps2->virtualdevice->id.bustype = BUS_I2C;

	/* set the supported event type for input device */
	set_bit(EV_ABS, vps2->virtualdevice->evbit);
	set_bit(ABS_DISTANCE, vps2->virtualdevice->absbit);
	input_set_abs_params(vps2->virtualdevice, ABS_DISTANCE, 0, 1, 0, 0);

	nRetVal = input_register_device(vps2->virtualdevice);
	if (nRetVal < 0)
	{
		pr_err("*** Unable to register virtual P-sensor input device ***\n");
		return nRetVal;
	}

	vps2->vps_cdev = virtual_sensors_proximity_cdev2;
	vps2->vps_cdev.sensors_enable = vps_set_enable2;
	vps2->vps_cdev.sensors_poll_delay = NULL;

	nRetVal = sensors_classdev_register(&pClient->dev, &vps2->vps_cdev);
	if (nRetVal) {
		pr_err("%s: Unable to register to sensors class: %d\n",__func__, nRetVal);
	return nRetVal;
	}

	return 0;
}
