/*
 * The entry point to the kernel module
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
 * -> module (un)load, read & write & IO, PCI probe/remove, IRQ(ISR)
 *
 */

#include "imago_fpga.h"

struct imago_device_info {
	char *name;
	u8 flags;
};

static struct imago_device_info device_info[] = {
	[DeviceType_AGEX] = {
		.name = "VisionBox AGE-X",
		.flags = IMAGO_DEV_FLAG_PCI},
	[DeviceType_AGEX2] = {
		.name = "VisionBox AGE-X2",
		.flags = IMAGO_DEV_FLAG_PCIE},
	[DeviceType_MVC0] = {
		.name = "Machine Vision Controller",
		.flags = IMAGO_DEV_FLAG_PCIE},
	[DeviceType_AGEX2_CL] = {
		.name = "VisionBox AGE-X2 CL",
		.flags = IMAGO_DEV_FLAG_PCIE | IMAGO_DEV_FLAG_DMA2HOST | IMAGO_DEV_FLAG_PCI64BIT},
	[DeviceType_VCXM] = {
		.name = "VisionCam XM",
		.flags = IMAGO_DEV_FLAG_PCIE | IMAGO_DEV_FLAG_DMA2HOST},
	[DeviceType_LEMANS] = {
		.name = "VisionBox LE MANS",
		.flags = IMAGO_DEV_FLAG_PCIE},
	[DeviceType_PCIE_CL] = {
		.name = "PCIe CL",
		.flags = IMAGO_DEV_FLAG_PCIE | IMAGO_DEV_FLAG_DMA2HOST | IMAGO_DEV_FLAG_PCI64BIT},
	[DeviceType_AGEX5] = {
		.name = "VisionBox AGE-X5",
		.flags = IMAGO_DEV_FLAG_PCIE},
	[DeviceType_AGEX5_CL] = {
		.name = "VisionBox AGE-X5 CL",
		.flags = IMAGO_DEV_FLAG_PCIE | IMAGO_DEV_FLAG_DMA2HOST | IMAGO_DEV_FLAG_PCI64BIT},
	[DeviceType_DAYTONA] = {
		.name = "VisionBox DAYTONA",
		.flags = IMAGO_DEV_FLAG_SPI},
	[DeviceType_VSPV3] = {
		.name = "VisionSensor PV3",
		.flags = IMAGO_DEV_FLAG_SPI},
};

//setzt alle Felder auf definierte Werte
DEVICE_DATA *imago_alloc_dev_data(struct device *dev, u8 dev_type)
{
	int i, minor, iChannel, iTC;
	DEVICE_DATA *pDevData = NULL;

	dev_dbg(dev, "found '%s' device\n", device_info[dev_type].name);

	for (minor = 0; minor < MAX_DEVICE_COUNT; minor++) {
		if (_ModuleData.dev_data[minor] == NULL) {
			pDevData = kzalloc(sizeof(DEVICE_DATA), GFP_KERNEL);
			if (pDevData == NULL) {
				dev_err(dev, "imago_alloc_dev_data: kzalloc() failed\n");
				return NULL;
			}
			_ModuleData.dev_data[minor] = pDevData;
			pDevData->DeviceNumber = MKDEV(MAJOR(_ModuleData.FirstDeviceNumber), minor);
			break;
		}
	}

	if (pDevData == NULL) {
		dev_err(dev, "no free minor number found\n");
		return NULL;
	}

	/* Module */
	pDevData->boIsDeviceOpen	= false;
	pDevData->dev				= dev;

	pDevData->device_type		= dev_type;
	if (dev_type != DeviceType_Invalid)
		pDevData->flags				= device_info[dev_type].flags;
	else
		pDevData->flags				= 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
		sema_init(&pDevData->DeviceSem,1);		//1<>frei
#else
		init_MUTEX( &pDevData->DeviceSem);
#endif

	/* SUN */
	raw_spin_lock_init(&pDevData->lock);

	for (i = 0; i<MAX_IRQDEVICECOUNT; i++) {
		pDevData->SunDeviceData[i].requestState = SUN_REQ_STATE_FREE;
		pDevData->SunDeviceData[i].serialID = 0;
		init_completion(&pDevData->SunDeviceData[i].result_complete);
	}

	pDevData->boIsBAR0Requested = false;
	pDevData->pVABAR0			= NULL;
	pDevData->pVACommonBuffer 	= NULL;
	pDevData->pBACommonBuffer	= 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
	// VC-XM / DRA7x PCIe workaround for IRQ race in old kernels:
	// "dra7xx: PCIe IRQ handling rework" https://lkml.org/lkml/2018/2/9/208
	// https://github.com/IMAGO-Technologies/linux-visioncam/commit/87263c04ec46afb9c664ba9bde166c40cd32e5cb#diff-3aa30d57b2d8270de9ab785bccd77846
	// Wenn der naechste FPGA Interrupt ankommt bevor der HWI verlassen wird, so kann
	// er verloren gehen => FPGA Interrupt erst im IRQ Thread freigeben
	if (dev_type == DeviceType_VCXM)
		pDevData->irqEnableInHWI	= false;
	else
		pDevData->irqEnableInHWI	= true;
#else
	pDevData->irqEnableInHWI	= true;
#endif

	// DMA
	if (_ModuleData.dma_update_in_hwi == 0)
		pDevData->setupTcInHWI		= 0;
	else
		pDevData->setupTcInHWI		= 1;
	pDevData->DMARead_channels	= 0;
	pDevData->DMARead_TCs		= 0;
	pDevData->DMARead_SGs		= 0;
	raw_spin_lock_init(&pDevData->DMARead_SpinLock);
	for (iChannel = 0; iChannel < MAX_DMA_CHANNELS; iChannel++) {
		PDMA_READ_CHANNEL pChannel = &pDevData->DMARead_Channel[iChannel];
		pChannel->jobBuffers = NULL;
		init_completion(&pChannel->job_complete);
		pChannel->dmaWaitCount = 0;
		pChannel->abortWait = 0;

		for (iTC = 0; iTC < MAX_DMA_READ_CHANNELTCS; iTC++) {
			PDMA_READ_TC pTC = pChannel->TCs+iTC;
			pTC->boIsUsed = false;
		}
	}
	
	return pDevData;
}

static void imago_dev_release(struct device *dev)
{
	PDEVICE_DATA pDevData = dev_get_drvdata(dev);
	dev_dbg(dev, "imago_dev_release, dev: %p, pDevData: %p\n", dev, pDevData);
	
	if (pDevData != NULL)
		imago_free_dev_data(pDevData);
}

extern struct file_operations fpga_ops;

// Helper function for creating a char device
int imago_create_device(PDEVICE_DATA pDevData)
{
	int res;

	cdev_init(&pDevData->DeviceCDev, &fpga_ops);
	pDevData->DeviceCDev.owner = THIS_MODULE;
	pDevData->DeviceCDev.ops 	= &fpga_ops;	//notwendig in den quellen wird fops gesetzt?

	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
	res = cdev_add(&pDevData->DeviceCDev, pDevData->DeviceNumber, 1/*wie viele ab startNum*/);
	if (res < 0) {
		dev_err(pDevData->dev, "cdev_add() failed\n");
		return res;
	}

	// create a device and registers it with sysfs
	// struct device *dev = device_create(
	pDevData->sub_dev = device_create(
			_ModuleData.pModuleClass,
			NULL, 						// parent
			pDevData->DeviceNumber,
			NULL,						// drvdata
			MODMODULENAME"%d", MINOR(pDevData->DeviceNumber)
			);
	if (IS_ERR(pDevData->sub_dev)) {
		dev_err(pDevData->sub_dev, "error creating sysfs device (%ld)\n", PTR_ERR(pDevData->sub_dev));
		cdev_del(&pDevData->DeviceCDev);
		return PTR_ERR(pDevData->sub_dev);
	}

	dev_set_drvdata(pDevData->sub_dev, pDevData);
	pDevData->sub_dev->release = imago_dev_release;
	pDevData->boIsDeviceOpen = true;

	dev_dbg(pDevData->sub_dev, "device created\n");

	return 0;
}

void imago_dev_close(DEVICE_DATA *pDevData)
{
	int minor;

	dev_dbg(pDevData->dev, "imago_dev_close()\n");

	for (minor = 0; minor < MAX_DEVICE_COUNT; minor++) {
		if (_ModuleData.dev_data[minor] == pDevData && pDevData->boIsDeviceOpen) {
			u8 device_id;
			unsigned long flags;

			raw_spin_lock_irqsave(&pDevData->lock, flags);
			pDevData->boIsDeviceOpen = false;
			// wakeup threads waiting in read():
			for (device_id = 0; device_id < MAX_IRQDEVICECOUNT; device_id++)
			{
				struct SUN_DEVICE_DATA *pSunDevice = &pDevData->SunDeviceData[device_id];
				
				if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA) {
					pSunDevice->requestState = SUN_REQ_STATE_ABORT;
					complete(&pSunDevice->result_complete);
				}
			}
			raw_spin_unlock_irqrestore(&pDevData->lock, flags);

			// remove device from sysfs
			device_destroy(_ModuleData.pModuleClass, pDevData->DeviceNumber);

			// delete the device
			cdev_del(&pDevData->DeviceCDev);

			return;
		}
	}
	
	dev_warn(pDevData->dev, "imago_dev_close(): invalid device data\n");
}

void imago_free_dev_data(DEVICE_DATA *pDevData)
{
	int minor;

	pr_devel(MODMODULENAME": imago_dev_free()\n");

	for (minor = 0; minor < MAX_DEVICE_COUNT; minor++) {
		if (_ModuleData.dev_data[minor] == pDevData) {
			kfree(pDevData);
			_ModuleData.dev_data[minor] = NULL;
			return;
		}
	}
	
	pr_warn(MODMODULENAME": imago_dev_free: invalid device data\n");
}
