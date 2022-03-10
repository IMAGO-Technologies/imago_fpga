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

//module defines "sudo modinfo agexpcidrv.ko"
MODULE_VERSION(MODVERSION);
MODULE_LICENSE(MODLICENSE);
MODULE_DESCRIPTION(MODDESCRIPTION);
MODULE_AUTHOR(MODAUTHOR);


static unsigned int max_dma_buffers = 32;
module_param(max_dma_buffers, uint, 0644);
MODULE_PARM_DESC(max_dma_buffers, "Maximum number of DMA buffers supported for each DMA channel (default: 32, must be power of 2).");


//static member (die gleichen für alle devices)
MODULE_DATA _ModuleData;


// "Called when a device is added, removed from this class, or a few other things that generate uevents to add the environment variables."
static int dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	dev_dbg(dev, "udev event\n");
    if (add_uevent_var(env, "DEVMODE=%#o", 0666) != 0)
		dev_warn(dev, "add_uevent_var() failed\n");
    return 0;
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

	// for (i=0; i<MAX_DEVICE_COUNT; i++) {
		// imago_init_dev_data(&_ModuleData.Devs[i], NULL, DeviceType_Invalid);
		// _ModuleData.boIsMinorUsed[i] = false;
	// }
		

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
	if (IS_ERR(_ModuleData.pModuleClass))
		pr_warning(MODMODULENAME": error creating sysfs class\n");
	
	//add the uevend handler
	if (!IS_ERR(_ModuleData.pModuleClass))
		_ModuleData.pModuleClass->dev_uevent = dev_uevent;	//send uevents to udev, so it'll create the /dev/node


	/* macht dem Kernel den treiber für PCIdevs bekannt */
	/**********************************************************************/
	if (pci_register_driver(&imago_pci_driver) != 0)
		pr_warning(MODMODULENAME": pci_register_driver failed!\n");

#if defined(__aarch64__) && defined(CONFIG_SPI_MASTER)
	if (spi_register_driver(&imago_spi_driver) != 0)
		pr_warning(MODMODULENAME": spi_register_driver failed!\n");
#endif

	pr_info(MODMODULENAME": init done (%s [%s])\n", MODDATECODE, MODVERSION);

	return 0;
}

//wird aufgerufen wenn das Modul entladen wird
//Note: kann nicht entladen werden wenn es noch genutzt wird
static void __exit imago_module_exit(void)
{
	pr_devel(MODMODULENAME": imago_module_exit()\n");

#if defined(__aarch64__) && defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&imago_spi_driver);
#endif

	//wir können uns nicht mehr um PCIdevs kümmern:-)
	pci_unregister_driver(&imago_pci_driver);

	//gibt Sysfs class frei
	if(!IS_ERR(_ModuleData.pModuleClass))
		class_destroy(_ModuleData.pModuleClass);

	//gibt die dev Nnummern frei
	//Note: "cleanup_module is never called if registering failed"
	unregister_chrdev_region(_ModuleData.FirstDeviceNumber, MAX_DEVICE_COUNT);
}


module_init(imago_module_init);
module_exit(imago_module_exit);
