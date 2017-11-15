/* This file is added for bayer sensor OV5670 modules. */

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"

struct ov5670_otp_struct {
    uint16_t module_integrator_id;
    uint16_t lens_id;
	uint16_t vcm_id;
	uint16_t driver_ic;
    uint16_t production_year;
    uint16_t production_month;
    uint16_t production_day;
    uint16_t RG_gain, BG_gain, G_gain;
    uint16_t golden_RG, golden_BG, golden_G;
    uint16_t typical_RG, typical_BG;
};

static int32_t otp_i2c_write(struct msm_sensor_ctrl_t *s_ctrl, 
    uint32_t addr, uint16_t data)
{
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 
		    addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

static uint16_t otp_i2c_read(struct msm_sensor_ctrl_t *s_ctrl, 
	uint32_t addr)
{
	uint16_t data = 0;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 
		addr, &data, MSM_CAMERA_I2C_BYTE_DATA);
	return data;
}

static uint32_t otp_get_modinfo_group(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint16_t flag = 0, temp = 0;

    otp_i2c_write(s_ctrl, 0x0100, 0x01);
	//set 0x5002[1] to 0
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    //partial mode OTP write start address
    otp_i2c_write(s_ctrl, 0x3d88, 0x70);
    otp_i2c_write(s_ctrl, 0x3d89, 0x10);
    // partial mode OTP write end address
    otp_i2c_write(s_ctrl, 0x3d8A, 0x70);
    otp_i2c_write(s_ctrl, 0x3d8B, 0x10);
    // read otp into buffer
    otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    flag = otp_i2c_read(s_ctrl, 0x7010);
    // clear otp buffer
    otp_i2c_write(s_ctrl, 0x7010, 0x00);
	//set 0x5002[1] to 1
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
    if ((flag & 0xC0) == 0x40) {
		return 1;
    } else if ((flag & 0x30) == 0x10) {
        return 2;
    } else if ((flag & 0x0C) == 0x04) {
        return 3;
    } else {
        return 0;
    }
}

static uint32_t otp_get_awb_group(struct msm_sensor_ctrl_t *s_ctrl)
{
    uint16_t flag = 0, temp = 0;
	
    otp_i2c_write(s_ctrl, 0x0100, 0x01);
    // set 0x5002[1] to 0
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    // partial mode OTP write start address
    otp_i2c_write(s_ctrl, 0x3d88, 0x70);
    otp_i2c_write(s_ctrl, 0x3d89, 0x26);
    // partial mode OTP write end address
    otp_i2c_write(s_ctrl, 0x3d8A, 0x70);
    otp_i2c_write(s_ctrl, 0x3d8B, 0x26);
    // read otp into buffer
    otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    flag = otp_i2c_read(s_ctrl, 0x7026);
    // clear otp buffer
    otp_i2c_write(s_ctrl, 0x7026, 0x00);
    //set 0x5002[1] to 1
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
	if ((flag & 0xC0) == 0x40) {
		return 1;
    } else if ((flag & 0x30) == 0x10) {
        return 2;
    } else if ((flag & 0x0C) == 0x04) {
        return 3;
    } else {
        return 0;
    }
}

static int32_t otp_read_modinfo(struct msm_sensor_ctrl_t *s_ctrl, 
    uint32_t index, struct ov5670_otp_struct *otp_ptr)
{
    uint16_t temp = 0;
    uint32_t i = 0, start_addr = 0, end_addr = 0;

    if (index == 1) {
    	start_addr = 0x7011;
    	end_addr = 0x7017;
    } else if (index == 2) {
    	start_addr = 0x7018;
    	end_addr = 0x701E;
    } else if (index == 3) {
    	start_addr = 0x701F;
    	end_addr = 0x7025;
    } else {
		return -1;
    }
    //set 0x5002[1] to 0
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    //partial mode OTP write start address
    otp_i2c_write(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff);
    otp_i2c_write(s_ctrl, 0x3d89, (start_addr & 0xff));
    // partial mode OTP write end address
    otp_i2c_write(s_ctrl, 0x3d8A, (end_addr >> 8) & 0xff);
    otp_i2c_write(s_ctrl, 0x3d8B, (end_addr & 0xff));
    // read otp into buffer
    otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    otp_ptr->module_integrator_id = otp_i2c_read(s_ctrl, start_addr);
    otp_ptr->lens_id = otp_i2c_read(s_ctrl, (start_addr + 1));
	otp_ptr->vcm_id = otp_i2c_read(s_ctrl, (start_addr + 2));
	otp_ptr->driver_ic = otp_i2c_read(s_ctrl, (start_addr + 3));
    otp_ptr->production_year = otp_i2c_read(s_ctrl, (start_addr + 4));
    otp_ptr->production_month = otp_i2c_read(s_ctrl, (start_addr + 5));
    otp_ptr->production_day = otp_i2c_read(s_ctrl, (start_addr + 6));
    // clear otp buffer
    for (i = start_addr; i <= end_addr; i++) {
    	otp_i2c_write(s_ctrl, i, 0x00);
    }
    //set 0x5002[1] to 1
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
    return 0;
}

static int32_t otp_read_awb(struct msm_sensor_ctrl_t *s_ctrl,
    uint32_t index, struct ov5670_otp_struct *otp_ptr)
{
    uint32_t i = 0, start_addr = 0, end_addr = 0;
    uint16_t temp = 0;
	
    if (index == 1) {
    	start_addr = 0x7028;
    	end_addr = 0x702F;
    } else if (index == 2) {
    	start_addr = 0x7031;
    	end_addr = 0x7038;
    } else if (index == 3) {
    	start_addr = 0x703A;
    	end_addr = 0x7041;
    } else {
		return -1;
    }
	// set 0x5002[1] to 0
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp & (~0x02)));
    otp_i2c_write(s_ctrl, 0x3d84, 0xC0);
    // partial mode OTP write start address
    otp_i2c_write(s_ctrl, 0x3d88, ((start_addr >> 8) & 0xff));
    otp_i2c_write(s_ctrl, 0x3d89, (start_addr & 0xff));
    // partial mode OTP write end address
    otp_i2c_write(s_ctrl, 0x3d8A, ((end_addr >> 8) & 0xff));
    otp_i2c_write(s_ctrl, 0x3d8B, (end_addr & 0xff));
    // read otp into buffer
    otp_i2c_write(s_ctrl, 0x3d81, 0x01);
    mdelay(5);
    temp = otp_i2c_read(s_ctrl, (start_addr + 3));
    otp_ptr->RG_gain = 
		(otp_i2c_read(s_ctrl, start_addr) << 2) + ((temp >> 6) & 0x03);
    otp_ptr->BG_gain = 
		(otp_i2c_read(s_ctrl, (start_addr + 1)) << 2) + ((temp >> 4) & 0x03);
	otp_ptr->G_gain = otp_i2c_read(s_ctrl, (start_addr + 2));
    temp = otp_i2c_read(s_ctrl, (start_addr + 7));
    otp_ptr->golden_RG = 
		(otp_i2c_read(s_ctrl, (start_addr + 4)) << 2) + ((temp >> 6) & 0x03);
    otp_ptr->golden_BG = 
		(otp_i2c_read(s_ctrl, (start_addr + 5)) << 2) + ((temp >> 4) & 0x03);
	otp_ptr->golden_G = otp_i2c_read(s_ctrl, (start_addr + 6));
    // clear otp buffer
    for (i = start_addr; i <= end_addr; i++) {
    	otp_i2c_write(s_ctrl, i, 0x00);
    }
    //set 0x5002[1] to 1
    temp = otp_i2c_read(s_ctrl, 0x5002);
    otp_i2c_write(s_ctrl, 0x5002, (temp | 0x02));
    return 0;
}

static int32_t otp_update_awb(struct msm_sensor_ctrl_t *s_ctrl,
    struct ov5670_otp_struct *otp_ptr)
{
	uint32_t nR_G_gain = 0, nB_G_gain = 0, nG_gain = 0, nBase_gain = 0, 
			R_gain = 0, B_gain = 0, G_gain = 0;
	
    nR_G_gain = (otp_ptr->typical_RG * 1000) / otp_ptr->RG_gain;
    nB_G_gain = (otp_ptr->typical_BG * 1000) / otp_ptr->BG_gain;
    nG_gain = 1000;
    if (nR_G_gain < 1000 || nB_G_gain < 1000) {
        if (nR_G_gain < nB_G_gain) {
        	nBase_gain = nR_G_gain;
        } else {
        	nBase_gain = nB_G_gain;
        }
    } else {
    	nBase_gain = nG_gain;
    }
    R_gain = 0x400 * nR_G_gain / (nBase_gain);
    B_gain = 0x400 * nB_G_gain / (nBase_gain);
    G_gain = 0x400 * nG_gain / (nBase_gain);
    if (R_gain > 0x400) {
    	otp_i2c_write(s_ctrl, 0x5032, (R_gain >> 8));
    	otp_i2c_write(s_ctrl, 0x5033, (R_gain & 0xff));
    }
    if (G_gain > 0x400) {
    	otp_i2c_write(s_ctrl, 0x5034, (G_gain >> 8));
    	otp_i2c_write(s_ctrl, 0x5035, (G_gain & 0xff));
    }
    if (B_gain > 0x400) {
    	otp_i2c_write(s_ctrl, 0x5036, (B_gain >> 8));
    	otp_i2c_write(s_ctrl, 0x5037, (B_gain & 0xff));
    }
    return 0;
}

int32_t ov5670_otp_config(struct msm_sensor_ctrl_t *s_ctrl)
{
    struct ov5670_otp_struct current_otp = {0};
    uint32_t otp_index = 0;

    otp_index = otp_get_awb_group(s_ctrl);
    if (otp_index == 1 || otp_index == 2 || otp_index == 3) {
        printk("%s, awb valid, group=%u\n", __func__, otp_index);
        otp_read_awb(s_ctrl, otp_index, &current_otp);
		printk("%s, RG_gain=%u, BG_gain=%u, G_gain=%u\n"
			"%s, golden_RG=%u, golden_BG=%u, golden_G=%u\n",
			__func__, current_otp.RG_gain, current_otp.BG_gain, current_otp.G_gain, 
			__func__, current_otp.golden_RG, current_otp.golden_BG, current_otp.golden_G);
        if(current_otp.RG_gain != 0 && current_otp.BG_gain != 0) {
			/* different modules have different typical RG/BG values */
			if (!strcmp(s_ctrl->sensordata->sensor_name, "ov5670_alto5tf")) {
				current_otp.typical_RG = 301;
				current_otp.typical_BG = 291;
			} else if (!strcmp(s_ctrl->sensordata->sensor_name, "ov5670_alto4evdo")) {
				current_otp.typical_RG = 628;
				current_otp.typical_BG = 550;
			} else {
				current_otp.typical_RG = 304;
				current_otp.typical_BG = 300;
			}
			printk("%s, sensor_name=%s\n"
				"%s, typical_RG=%u, typical_BG=%u\n", 
				__func__, s_ctrl->sensordata->sensor_name,
				__func__, current_otp.typical_RG, current_otp.typical_BG);
            otp_update_awb(s_ctrl, &current_otp);
        }
    } else {
        printk("%s, awb empty/invalid\n", __func__);
    }

	return 0;
}

uint16_t ov5670_otp_get_vcm_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct ov5670_otp_struct current_otp = {0};
	uint32_t otp_index = 0;

	otp_index = otp_get_modinfo_group(s_ctrl);
    if (otp_index == 1 || otp_index == 2 || otp_index == 3) {
        printk("%s, modinfo group=%u\n", __func__, otp_index);
    	otp_read_modinfo(s_ctrl, otp_index, &current_otp);
		printk("%s, module_integrator_id=%u, lens_id=%u\n"
			"%s, vcm_id=%u, driver_ic=%u\n"
			"%s, production_year=%u, production_month=%u, production_day=%u\n",
			__func__, current_otp.module_integrator_id, current_otp.lens_id,
			__func__, current_otp.vcm_id, current_otp.driver_ic,
			__func__, current_otp.production_year, current_otp.production_month, 
			current_otp.production_day);
    } else {
        printk("%s, modinfo empty/invalid\n", __func__);
    }

	return (current_otp.vcm_id);
}
