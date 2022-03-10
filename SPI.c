/*
 * SPI.c
 *
 * SPI code
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

#include "imago_fpga.h"
#include <linux/spi/spi.h>


static const struct of_device_id imago_spi_of_match[] = {
	{
		.compatible	= "imago,fpga-spi-daytona",
		.data		= (void *)DeviceType_DAYTONA,
	},
	{
		.compatible	= "imago,fpga-spi-vspv3",
		.data		= (void *)DeviceType_VSPV3,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imago_spi_of_match);

static const struct spi_device_id imago_spi_id[] = {
        {"fpga-spi-daytona", DeviceType_DAYTONA},
        {"fpga-spi-vspv3", DeviceType_VSPV3},
        {}
};
MODULE_DEVICE_TABLE(spi, imago_spi_id);


// writes FPGA packet
static long fpga_write(struct _DEVICE_DATA *pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	struct spi_transfer transfer;
	struct spi_message message;
	struct spi_device *spi = to_spi_device(pDevData->dev);
	// u8 packet[3 * 4];	// SUN packet:  header0, header1, data
	u8 deviceID;
	int result;
	u8 txbuf[1 + 3 * 4];		// SPI Header + SUN Paket

	txbuf[0] = 0; // SPI Header: in CPU Source-FIFO schreiben

	// User Data -> Kernel
	if (BytesToWrite > (3 * 4)) {
		dev_warn(pDevData->dev, "fpga_write(): too many bytes\n");
		return -EFBIG;
	}
	if (copy_from_user(&txbuf[1], pToUserMem, BytesToWrite) != 0) {
		dev_warn(pDevData->dev, "fpga_write(): copy_from_user() failed\n");
		return -EFAULT;
	}

	// insert serialID to Header1:
	deviceID = (((u32*)&txbuf[1])[1] >> 20) & (MAX_IRQDEVICECOUNT - 1);
	if (deviceID != 0) {
		((u32*)&txbuf[1])[1] |= pDevData->SunDeviceData[deviceID].serialID << 26;
	}

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = txbuf;
	transfer.len = sizeof(txbuf);
	spi_message_add_tail(&transfer, &message);

	result = spi_sync(spi, &message);
	if (result < 0) {
		dev_err(&spi->dev, "fpga_write(): SPI error %d\n", result);
		return -EFAULT;
	}

	return BytesToWrite;
}

// IRQ thread
static irqreturn_t spi_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	struct spi_transfer transfer;
	struct spi_message message;
	struct spi_device *spi;
	u8 txbuf[1+3*4] = {1 /* message type: read from CPU Target-FIFO */};
	u8 rxbuf[1+3*4];
	int result;

	BUG_ON(pDevData->device_type == DeviceType_Invalid);

	spi = to_spi_device(pDevData->dev);

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = txbuf;
	transfer.rx_buf = rxbuf;
	transfer.len = sizeof(txbuf);
	spi_message_add_tail(&transfer, &message);

	result = spi_sync(spi, &message);
	if (result < 0)
	{
		dev_err(pDevData->dev, "AGEXDrv_tasklet_SPI() > SPI error %d\n", result);
		return IRQ_HANDLED;
	}

	memcpy(sun_packet, &rxbuf[1], 3 * 4);

	// Process SUN packet
	imago_sun_interrupt(pDevData, sun_packet);

	return IRQ_HANDLED;
}


static int imago_spi_probe(struct spi_device *spi)
{
	PDEVICE_DATA pDevData = NULL;
	u8 dev_type;
	const struct of_device_id *of_id;

	dev_dbg(&spi->dev, "probe device\n");

	of_id = of_match_device(imago_spi_of_match, &spi->dev);
	if (of_id)
		dev_type = (unsigned long)of_id->data;
	else
		dev_type = spi_get_device_id(spi)->driver_data;

	if (dev_type == DeviceType_Invalid) {
		dev_err(&spi->dev, "invalid device identifier (%u)\n", dev_type);
		return -EINVAL;
	}

	if (!spi->irq) {
		dev_err(&spi->dev, "missing interrupt configuration\n");
		return -EINVAL;
	}

	pDevData = imago_alloc_dev_data(&spi->dev, dev_type);
	if (pDevData == NULL)
		return -EINVAL;

	pDevData->write = fpga_write;
	spi_set_drvdata(spi, pDevData);

	dev_dbg(&spi->dev, "using major/minor (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));


	// setup interrupt
	if (request_threaded_irq(spi->irq, NULL, spi_thread,
				IRQF_ONESHOT, MODMODULENAME, pDevData) != 0) {
		dev_err(&spi->dev, "request_threaded_irq failed\n");
		imago_free_dev_data(pDevData);
		return -EIO;
	}

	dev_dbg(&spi->dev, "IRQ: %d \n", spi->irq);
	pDevData->boIsIRQOpen = true;

	// create char device
	imago_create_device(pDevData);

	dev_info(&spi->dev, "probe done\n");

	return 0;
}


static int imago_spi_remove(struct spi_device *spi)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)spi_get_drvdata(spi);

	dev_dbg(&spi->dev, "imago_spi_remove\n");

	if (pDevData == NULL) {
		dev_warn(&spi->dev, "imago_spi_remove: device data is invalid\n");
		return -ENODEV;
	}

	//IRQ zuückgeben
	if (pDevData->boIsIRQOpen)
		free_irq(spi->irq, pDevData);
	
	imago_free_dev_data(pDevData);

	return 0;
}

struct spi_driver imago_spi_driver = {
	.driver = {
		.name	= "imago-fpga-spi",
		.of_match_table = of_match_ptr(imago_spi_of_match),
	},
	.id_table	= imago_spi_id,
	.probe		= imago_spi_probe,
	.remove		= imago_spi_remove,
};
