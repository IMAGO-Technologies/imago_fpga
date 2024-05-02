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
static int fpga_write(struct _DEVICE_DATA *pDevData, u32* packet, unsigned int packet_size)
{
	struct spi_transfer transfer;
	struct spi_message message;
	struct spi_device *spi = to_spi_device(pDevData->dev);
	int result;
	u8 txbuf[1 + 3 * 4];		// SPI Header + SUN Paket
	u8 rxBuf[1 + 3 * 4];
	u8 maxWriteRepeats;

	txbuf[0] = 0; // SPI Header: in CPU Source-FIFO schreiben

	dev_dbg(pDevData->dev, "fpga_write: H0:0x%08x, H1:0x%08x, D:0x%08x\n", packet[0], packet[1], packet[2]);

	// User Data -> Kernel
	if (packet_size != 3) {
		dev_warn(pDevData->dev, "fpga_write(): invalid packet size\n");
		return -EFBIG;
	}

	memcpy(&txbuf[1], packet, 4 * packet_size);

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = txbuf;
	transfer.len = sizeof(txbuf);
	transfer.rx_buf = rxBuf;
	spi_message_add_tail(&transfer, &message);

	result = spi_sync(spi, &message);
	if (result < 0) {
		dev_err(&spi->dev, "fpga_write(): SPI error %d\n", result);
		return -EFAULT;
	}
	
	if((rxBuf[0] & 1u) != 0){
		transfer.len = 1;
		dev_dbg(pDevData->dev, "Wait for spi buffer\n");
		maxWriteRepeats = 250;
		do
		{
			result = spi_sync(spi, &message);
			if (result < 0) {
				dev_err(&spi->dev, "fpga_write(): SPI error %d\n", result);
				return -EFAULT;
			}
			maxWriteRepeats--;
		} while (((rxBuf[0] & 1u) != 0) && (maxWriteRepeats > 0));
		if(maxWriteRepeats == 0){
			dev_warn(&spi->dev, "fpga_write(): SPI buffer error\n");
			return -EFAULT;
		}
		dev_dbg(pDevData->dev, "Wait for spi buffer done\n");
	}

	return 4 * packet_size;
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
	int res;

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

	dev_dbg(&spi->dev, "using IRQ: %d \n", spi->irq);

	// create char device
	res = imago_create_device(pDevData);
	if (res < 0)
		return res;

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

	//IRQ zu�ckgeben
	free_irq(spi->irq, pDevData);
	
	imago_dev_close(pDevData);

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
