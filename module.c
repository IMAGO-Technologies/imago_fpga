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

MODULE_VERSION(MODVERSION);
MODULE_LICENSE(MODLICENSE);
MODULE_DESCRIPTION(MODDESCRIPTION);
MODULE_AUTHOR(MODAUTHOR);


static int dma_update_in_hwi = -1;
module_param(dma_update_in_hwi, int, 0644);
MODULE_PARM_DESC(dma_update_in_hwi, "Update DMA transfer in irq (0: no, use threaded irq; 1: yes; -1: auto/default).");

MODULE_DATA _ModuleData;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static char *imago_devnode(const struct device *dev, umode_t *mode)
#else
static char *imago_devnode(struct device *dev, umode_t *mode)
#endif
{
	pr_devel("devnode: device: %s\n", dev_name(dev));

	if (mode)
		*mode = 0660;

	return NULL;
}


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

	/* init member */
	/**********************************************************************/
	memset(&_ModuleData, 0, sizeof(_ModuleData));
	_ModuleData.pModuleClass = ERR_PTR(-EFAULT);
	_ModuleData.dma_update_in_hwi = dma_update_in_hwi;

	// Test for existing module using the old name "agexpcidrv".
	// find_module() was removed in 5.12 (and 5.10.220 but not 5.11), but the old module doesn't
	// support new kernels anyway, so no check is required then.
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,0) && \
    (LINUX_VERSION_CODE < KERNEL_VERSION(5,10,220) || LINUX_VERSION_CODE >= KERNEL_VERSION(5,11,0))
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
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	_ModuleData.pModuleClass = class_create("agexdrv");
#else
	_ModuleData.pModuleClass = class_create(THIS_MODULE, "agexdrv");
#endif
	if (IS_ERR(_ModuleData.pModuleClass)) {
		pr_err(MODMODULENAME": error creating sysfs class\n");
		return PTR_ERR(_ModuleData.pModuleClass);
	}

	// add the uevent handler
	_ModuleData.pModuleClass->devnode = imago_devnode;

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
