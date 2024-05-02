/*
 * HID.c
 *
 * USB-HID driver
 *
 * Copyright (C) IMAGO Technologies GmbH
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
#include <linux/hid.h>
#include <linux/usb.h>


struct imago_hid_uart {
	PDEVICE_DATA pDevData;
	u8 rx_buf[3 * 4];
	u8 rx_valid;
	u8 tx_buf[2 + 3 * 4];
};


static int raw_event(struct hid_device *hid, struct hid_report *report,
		u8 *data, int size)
{
	hid_dbg(hid, "raw_event: report id=0x%02x\n", data[0]);
	if (data[0] == 0xb1) {
		// UART interrupt report (not used, but fires once after first access)
		hid_dbg(hid, "raw_event: interrupt=0x%02x\n", data[1]);
		hid_dbg(hid, "raw_event: DCD / RI =0x%02x\n", data[2]);
	}
	else if (data[0] >= 0xf0 && data[0] <= 0xfe) {
		// UART Input Report
		struct imago_hid_uart *hid_uart = hid_get_drvdata(hid);
		unsigned int i;
		unsigned int bytes = data[1];

		for (i = 0; i < bytes; i++) {
			// hid_dbg(hid, "data[]: 0x%02x\n", data[2+i]);

			hid_uart->rx_buf[hid_uart->rx_valid++] = data[2+i];

			if (hid_uart->rx_valid == (3 * 4)) {
				// process SUN packet
				u32 *sun_packet = (u32 *)hid_uart->rx_buf;
				hid_dbg(hid, "SUN packet: h0=0x%08x, h1=0x%08x, data=0x%08x\n", sun_packet[0], sun_packet[1], sun_packet[2]);
				imago_sun_interrupt(hid_uart->pDevData, sun_packet);
				hid_uart->rx_valid = 0;
			}
		}
	}

	return 0;
}

static int fpga_write(struct _DEVICE_DATA *pDevData, u32* packet, unsigned int packet_size)
{
	struct hid_device *hid = to_hid_device(pDevData->dev);
	struct imago_hid_uart *hid_uart = hid_get_drvdata(hid);
	u8 report_id = 0xd0 + (4 * packet_size - 1) / 4;
	int res;

	if (packet_size != 3) {
		dev_warn(pDevData->dev, "fpga_write(): invalid packet size\n");
		return -EFBIG;
	}

	hid_uart->tx_buf[0] = report_id;
	hid_uart->tx_buf[1] = 4 * packet_size;	// actual payload size
	memcpy(&hid_uart->tx_buf[2], packet, 4 * packet_size);

	hid_dbg(hid, "fpga_write: h0=0x%08x, h1=0x%08x, data=0x%08x\n", ((u32 *)&hid_uart->tx_buf[2])[0], ((u32 *)&hid_uart->tx_buf[2])[1], ((u32 *)&hid_uart->tx_buf[2])[2]);

	res = hid_hw_output_report(hid, hid_uart->tx_buf, 2 + 4 * packet_size);

	if (res != 2 + 4 * packet_size) {
		hid_err(hid, "fpga_write(): hid_hw_output_report() failed\n");
		return -EIO;
	}

	return 4 * packet_size;
}

static int imago_hid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int res;
	u8 dev_type;
	PDEVICE_DATA pDevData = NULL;
	u8 *msg = NULL;
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct imago_hid_uart *hid_uart;

	dev_type = id->driver_data;
	if (dev_type == DeviceType_Invalid) {
		hid_err(hdev, "invalid device type (%u)\n", dev_type);
		return -EINVAL;
	}

	hid_dbg(hdev, "FT260 interface: %u\n", intf->cur_altsetting->desc.bInterfaceNumber);

	res = hid_parse(hdev);
	if (res) {
		hid_err(hdev, "parse failed\n");
		goto exit;
	}

	res = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (res) {
		hid_err(hdev, "hw start failed\n");
		goto exit;
	}

	if (intf->cur_altsetting->desc.bInterfaceNumber == 0)
		return 0;

	msg = kmalloc(64, GFP_KERNEL);

	res = hid_hw_raw_request(hdev, 0xa1, msg,
				 26, HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (res < 0) {
		hid_err(hdev, "hid_hw_raw_request failed\n");
		kfree(msg);
		goto exit_stop;
	}
	hid_dbg(hdev, "FT260 chip_mode: %d\n", msg[1]);
	if (msg[1] != 0) {
		res = -ENODEV;
		kfree(msg);
		goto exit_stop;
	}

	res = hid_hw_open(hdev);
	if (res < 0) {
		hid_err(hdev, "hid_hw_open() failed\n");
		kfree(msg);
		goto exit_stop;
	}

	// set system clock
	msg[0] = 0xa1;	// Report ID
	msg[1] = 0x01;	// Set Clock
	msg[2] = 2;		// 48 MHz
	res = hid_hw_raw_request(hdev, 0xa1, msg,
				 3, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (res < 0) {
		hid_err(hdev, "hid_hw_raw_request() set system clock failed\n");
		goto exit_stop;
	}

	// configure UART
	msg[0] = 0xa1;		// Report ID
	msg[1] = 0x41;		// Configure UART
	msg[2] = 4;			// flow_ctrl: off
	//*(unsigned int*)&msg[3] = 4156250;	// 4.15625 Mbps
	//*(unsigned int*)&msg[3] = 2078125;	// 2.078 Mbps
	*(unsigned int*)&msg[3] = 2083333;	// 2.083 Mbps
	msg[7] = 0x08;		// data_bit: 8
	msg[8] = 0;			// parity: no parity
	msg[9] = 0;			// one stop bit
	msg[10] = 0;		// breaking: not TXD spacing
	res = hid_hw_raw_request(hdev, 0xa1, msg, 11, HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (res < 0) {
		hid_err(hdev, "hid_hw_raw_request() Configure UART failed\n");
		goto exit_stop;
	}

	hid_uart = kzalloc(sizeof(*hid_uart), GFP_KERNEL);
	if (!hid_uart) {
		res = -ENOMEM;
		goto exit_stop;
	}

	pDevData = imago_alloc_dev_data(&hdev->dev, dev_type);
	if (pDevData == NULL)
		return -EINVAL;

	pDevData->write = fpga_write;
	hid_uart->pDevData = pDevData;
	hid_set_drvdata(hdev, hid_uart);

	hid_dbg(hdev, "using major/minor (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	// create char device
	res = imago_create_device(pDevData);
	if (res < 0)
		return res;

	hid_info(hdev, "probe done\n");

	res = 0;
	goto exit;

exit_stop:
	hid_hw_stop(hdev);
	if (pDevData != NULL)
		imago_free_dev_data(pDevData);
exit:
	if (msg != NULL)
		kfree(msg);

	return res;
}

static void imago_hid_remove(struct hid_device *hdev)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);

	hid_dbg(hdev, "imago_hid_remove()\n");

	if (intf->cur_altsetting->desc.bInterfaceNumber == 1) {
		struct imago_hid_uart *hid_uart = hid_get_drvdata(hdev);
		PDEVICE_DATA pDevData = hid_uart->pDevData;

		if (pDevData == NULL) {
			hid_warn(hdev, "imago_hid_remove(): device data is invalid\n");
			return;
		}

		imago_dev_close(pDevData);

		kfree(hid_uart);
	}

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id imago_devices[] = {
	{ HID_USB_DEVICE(0x0403, 0x6B88),
		.driver_data = DeviceType_MVM2 },
	{ }
};

MODULE_DEVICE_TABLE(hid, imago_devices);

struct hid_driver imago_hid_driver = {
		.name = "imago-fpga-hid",
		.id_table = imago_devices,
		.probe = imago_hid_probe,
		.remove = imago_hid_remove,
		.raw_event = raw_event,
};

