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
#include <linux/pci.h>
#include <linux/spi/spi.h>
#include <linux/hid.h>

//module defines "sudo modinfo agexpcidrv.ko"
MODULE_VERSION(MODVERSION);
MODULE_LICENSE(MODLICENSE);
MODULE_DESCRIPTION(MODDESCRIPTION);
MODULE_AUTHOR(MODAUTHOR);


static unsigned int max_dma_buffers = 32;
module_param(max_dma_buffers, uint, 0644);
MODULE_PARM_DESC(max_dma_buffers, "Maximum number of DMA buffers supported for each DMA channel (default: 32, must be power of 2).");

static int dma_update_in_hwi = -1;
module_param(dma_update_in_hwi, int, 0644);
MODULE_PARM_DESC(dma_update_in_hwi, "Update DMA transfer in irq (0: no, use threaded irq; 1: yes; -1: auto/default).");


//static member (die gleichen für alle devices)
MODULE_DATA _ModuleData;


// "Called when a device is added, removed from this class, or a few other things that generate uevents to add the environment variables."
static int dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	// do not use dev_dbg(), because of "i2c: prevent endless uevent loop with CONFIG_I2C_DEBUG_CORE":
	// https://patchwork.ozlabs.org/project/linux-i2c/patch/1458748247-9219-1-git-send-email-jglauber@cavium.com/
	pr_devel("uevent: device: %s\n", dev_name(dev));

	return add_uevent_var(env, "DEVMODE=%#o", 0666);
}

	 
//<====================================>
//	Module
//<====================================>

//wird aufgerufen wenn das Modul geladen wird
static int __init imago_module_init(void)
{
	int res;

	pr_devel(MODMODULENAME": imago_module_init()\n");

	//im sicher zu sein das bei einem 64Bit Build auch dieses Define gesetzt war...
#ifdef CONFIG_64BIT 
	if (sizeof(void*) != sizeof(u64)) {
		pr_err(MODMODULENAME": invalid 64 bit build!\n");
		return -EINVAL;
	}
#else	
	if (sizeof(void*) != sizeof(u32)) {
		pr_err(MODMODULENAME": invalid 64 bit build!\n");
		return -EINVAL;
	}
#endif

	if (!is_power_of_2(max_dma_buffers)) {
		pr_err(MODMODULENAME": module parameter 'max_dma_buffers' must be power of 2\n");
		return -EINVAL;
	}

	/* init member */
	/**********************************************************************/
	memset(&_ModuleData, 0, sizeof(_ModuleData));
	_ModuleData.pModuleClass = ERR_PTR(-EFAULT);
	_ModuleData.max_dma_buffers = max_dma_buffers;
	_ModuleData.dma_update_in_hwi = dma_update_in_hwi;

	// Test for existing module using the old name "agexpcidrv".
	// find_module() was removed in 5.12, but the old module doesn't support this kernel anyway,
	// so no check is required.
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,0)
	{
		struct module *old_mod;
		mutex_lock(&module_mutex);
		old_mod = find_module("agexpcidrv");
		mutex_unlock(&module_mutex);
		if (old_mod) {
			pr_err(MODMODULENAME": Error: the old driver 'agexpcidrv' is already loaded, aborting...");
			return -EBUSY;
		}
	}
#endif

	/* sich n device nummer holen */
	/**********************************************************************/
	// steht dann in /proc/devices
	res = alloc_chrdev_region(
			&_ModuleData.FirstDeviceNumber,	/* out die 1. nummer */
			0,					/* 1. minor am besten 0 */
			MAX_DEVICE_COUNT,	/* wie viele */
			MODMODULENAME);		/* name vom driver */
	if (res < 0) {
		pr_err(MODMODULENAME": can't get major number\n");
		return res;
	}
	pr_devel(MODMODULENAME": major: %u, minor: %u, count: %u\n", MAJOR(_ModuleData.FirstDeviceNumber), MINOR(_ModuleData.FirstDeviceNumber), MAX_DEVICE_COUNT);

	//sicher ist sicher (wir nutzen den Minor als Index für _ModuleData_Devs[])
	if (MINOR(_ModuleData.FirstDeviceNumber) != 0) {
		pr_err(MODMODULENAME": first minor number must be zero\n");
		return -EINVAL;
	}


	/* erzeugt eine Sysfs class */
	/**********************************************************************/
	_ModuleData.pModuleClass = class_create(THIS_MODULE, "agexdrv");
	if (IS_ERR(_ModuleData.pModuleClass)) {
		pr_err(MODMODULENAME": error creating sysfs class\n");
		return PTR_ERR(_ModuleData.pModuleClass);
	}

	// add the uevent handler
	_ModuleData.pModuleClass->dev_uevent = dev_uevent;	//send uevents to udev, so it'll create the /dev/node


	// register drivers
#if IS_ENABLED(CONFIG_PCI)
	if (pci_register_driver(&imago_pci_driver) != 0)
		pr_warn(MODMODULENAME": pci_register_driver failed!\n");
#endif

#ifdef __aarch64__
#if IS_ENABLED(CONFIG_SPI_MASTER)
	if (spi_register_driver(&imago_spi_driver) != 0)
		pr_warn(MODMODULENAME": spi_register_driver failed!\n");
#endif
#endif

#if IS_ENABLED(CONFIG_USB_HID)
	if (hid_register_driver(&imago_hid_driver) != 0)
		pr_warn(MODMODULENAME": usb_register_driver failed!\n");
#endif
	
	pr_info(MODMODULENAME": init done (%s [%s])\n", MODDATECODE, MODVERSION);

	return 0;
}

//wird aufgerufen wenn das Modul entladen wird
//Note: kann nicht entladen werden wenn es noch genutzt wird
static void __exit imago_module_exit(void)
{
	pr_devel(MODMODULENAME": imago_module_exit()\n");

#if IS_ENABLED(CONFIG_USB_HID)
	hid_unregister_driver(&imago_hid_driver);
#endif

#ifdef __aarch64__
#if IS_ENABLED(CONFIG_SPI_MASTER)
	spi_unregister_driver(&imago_spi_driver);
	imago_remove_i2cAdapter();
#endif
#endif

#if IS_ENABLED(CONFIG_PCI)
	pci_unregister_driver(&imago_pci_driver);
#endif

	class_destroy(_ModuleData.pModuleClass);

	unregister_chrdev_region(_ModuleData.FirstDeviceNumber, MAX_DEVICE_COUNT);
}


module_init(imago_module_init);
module_exit(imago_module_exit);
