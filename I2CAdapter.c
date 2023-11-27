/*
 *
 * Copyright (C) 201x IMAGO Technologies GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

/* The I2C adapter can be regisered for a VSPV device when a BreakoutBox is connected.
This bridges the SunSystem interface and makes it possible for a standard rtc driver to be used */

#include "imago_fpga.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/fs.h>

struct adapterData
{
	PDEVICE_DATA pDevData;
	unsigned short previousHost;
	u8 deviceID;
};

static void configHost(unsigned short host, struct adapterData *adapt)
{
	unsigned int wrmsg[3];
	wrmsg[0] = 0x51800000;						   /* hdr0 */
	wrmsg[1] = 0x80000001 | adapt->deviceID << 20; /* hdr1 */
	wrmsg[2] = (1 << 28) | host;				   /* data */
	imago_write_internal(adapt->pDevData, (char *)wrmsg, 12);
	adapt->previousHost = host;
}

static long writeBytes(struct i2c_msg *msg, struct adapterData *adapt, struct device *dev)
{
	unsigned int wrmsg[3];
	unsigned int bytes;
	unsigned int i, j;
	long retVal;
	i = 0;
	if (adapt->previousHost != msg->addr)
	{
		configHost(msg->addr, adapt);
	}
	dev_dbg(dev, "Write msg len: %i\n", msg->len);

	for (i = 0; i < msg->len; i += bytes)
	{
		// write two bytes at a time
		bytes = (msg->len - i) >= 3 ? 3 : (msg->len - i);
		wrmsg[0] = 0x51800000;						   /* hdr0 */
		wrmsg[1] = 0x80000001 | adapt->deviceID << 20; /* hdr1 */
		wrmsg[2] = (1 << 31) | (bytes << 24);		   /* data */
		if (bytes < 3 || (i + bytes) == msg->len)
			wrmsg[2] |= 1 << 29;
		for (j = 0; j < bytes; j++)
		{
			((unsigned char *)&wrmsg[2])[j] = msg->buf[j + i];
		}
		retVal = imago_write_internal(adapt->pDevData, (unsigned char *)wrmsg, 12);
		if (retVal < 0)
			return retVal;
		usleep_range(800, 850);
	}
	return 0;
}

static long readBytes(struct i2c_msg *msg, struct adapterData *adapt, struct device *dev)
{
	unsigned int bytes;
	unsigned int i, j, k, l;
	unsigned int wrmsg[3];
	unsigned int rdmsg[6];
	long retVal;
	
	i = 0;
	if (adapt->previousHost != msg->addr)
	{
		configHost(msg->addr, adapt);
	}
	dev_dbg(dev, "Read msg len: %i\n", msg->len);
	for (i = 0; i < msg->len; i += bytes)
	{
		// read up to 16 bytes at a time
		bytes = (msg->len - i) >= 16 ? 16 : (msg->len - i);

		wrmsg[0] = 0x51800000;						   /* hdr0 */
		wrmsg[1] = 0x80000001 | adapt->deviceID << 20; /* hdr1 */
		wrmsg[2] = ((bytes - 1) << 24);				   /* 0 ^= 1 byte read */

		// kernel_read(adapt->spiDrvFp, wrmsg, 24, &(adapt->spiDrvFp->f_pos));
		retVal = imago_write_internal(adapt->pDevData, (unsigned char *)wrmsg, 12);
		for (k = 0; k < bytes; k += 4)
		{
			rdmsg[0] = adapt->deviceID;					   /* devId */
			rdmsg[1] = 12;								   /* len */
			rdmsg[2] = 100;								   /* timeout */
			rdmsg[3] = 0x51900000;						   /* hdr0 */
			rdmsg[4] = 0x80000001 | adapt->deviceID << 20; /* hdr1 device id is important*/
			rdmsg[5] = 0;
			usleep_range(500, 550);
			retVal = imago_read_internal(adapt->pDevData, (char *)rdmsg, 24);
			if (retVal < 0)
			{
				return retVal;
			}
			l = bytes - k >= 4 ? 4 : bytes - k;
			for (j = 0; j < l; j++)
			{
				msg->buf[i + j + k] = (rdmsg[2] >> ((l - j - 1) * 8)) & 0xFF;
			}
		}
	}
	return 0;
}

static int i2cAdapter_xfer(struct i2c_adapter *adapt, struct i2c_msg msgs[], int num)
{
	long retVal;
	int i;
	struct adapterData *data;
	data = (struct adapterData *)adapt->algo_data;
	
	for (i = 0; i < num; i++)
	{
		if ((msgs[i].flags & I2C_M_RD) == 0)
		{
			retVal = writeBytes(&msgs[i], data, &adapt->dev);
		}
		else
		{
			retVal = readBytes(&msgs[i], data, &adapt->dev);
		}
		if (retVal < 0)
			return -1;
	}
	return i;
}

static unsigned int i2cAdapter_functionality(struct i2c_adapter *adapt)
{
	return I2C_FUNC_I2C;
}

struct i2c_algorithm imago_i2cAlgo = {
	.master_xfer = i2cAdapter_xfer,
	.functionality = i2cAdapter_functionality,
};

struct i2c_adapter imago_i2cAdapter = {
	.owner = THIS_MODULE,
	.algo = &imago_i2cAlgo,
	.name = "IMAGO i2c adapter",
	.algo_data = NULL,
};

long imago_init_i2cAdapter(PDEVICE_DATA pDevData)
{
	struct adapterData *data;
	
	if (imago_i2cAdapter.algo_data != NULL)
		return imago_i2cAdapter.nr;
	
	data = kzalloc(sizeof(struct adapterData), GFP_KERNEL);
	data->previousHost = 0;
	data->pDevData = pDevData;
	if (imago_create_deviceid(pDevData, &data->deviceID) < 0)
	{
		dev_warn(pDevData->dev, "No device id available\n");
		return -EINVAL;
	}
	imago_i2cAdapter.algo_data = data;
	if (i2c_add_adapter(&imago_i2cAdapter) < 0)
		return -1;
	else
		return imago_i2cAdapter.nr;
}

void imago_remove_i2cAdapter(void)
{
	if (imago_i2cAdapter.algo_data != NULL)
	{
		struct adapterData *tempData = (struct adapterData *)imago_i2cAdapter.algo_data;
		imago_abort_longterm_read(tempData->pDevData, tempData->deviceID);
		imago_release_deviceid(tempData->pDevData, tempData->deviceID);
		kfree(tempData);
		i2c_del_adapter(&imago_i2cAdapter);
	}
}
