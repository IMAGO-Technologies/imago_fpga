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

#include "AGEXDrv.h"

//module defines "sudo modinfo agexpcidrv.ko"
MODULE_VERSION(MODVERSION);
MODULE_LICENSE(MODLICENSE);
MODULE_DESCRIPTION(MODDESCRIPTION);
MODULE_AUTHOR(MODAUTHOR);

//übergibt/speichert die pointer
module_init(AGEXDrv_init);
module_exit(AGEXDrv_exit);

//static member (die gleichen für alle devices)
MODULE_DATA _ModuleData;

//für welche PCI IDs sind wir zuständi?
static struct pci_device_id AGEXDrv_ids[] = {
	{ PCI_DEVICE(0x1204/*VendorID (Lattice Semi)*/, 0x0200 /*DeviceID*/),	/* AGE-X1 */
		.driver_data = SubType_AGEX },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0004 /*DeviceID*/),			/* AGE-X2 */
		.driver_data = SubType_AGEX2 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xA6E4 /*DeviceID*/),			/* MVC0 */
		.driver_data = SubType_MVC0 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0010 /*DeviceID*/),			/* AGE-X2-CL */
		.driver_data = SubType_AGEX2_CL },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0005 /*DeviceID*/),			/* VCXM */
		.driver_data = SubType_VCXM },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xCA72 /*DeviceID*/),			/* LeMans */
		.driver_data = SubType_LEMANS },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xDECA /*DeviceID*/),			/* PCIe-CL */
		.driver_data = SubType_PCIE_CL },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xA6E5 /*DeviceID*/),			/* AGE-X5 */
		.driver_data = SubType_AGEX5 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xDCA5 /*DeviceID*/),			/* AGE-X5-CL */
		.driver_data = SubType_AGEX5_CL },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, AGEXDrv_ids);		//macht dem kernel bekannt was dieses modul für PCI devs kann

//struct mit den PCICallBacks
static struct pci_driver AGEXDrv_pci_driver = {
	.name = MODMODULENAME,
	.id_table = AGEXDrv_ids,
	.probe = AGEXDrv_PCI_probe,
	.remove = AGEXDrv_PCI_remove,
};


// "Called when a device is added, removed from this class, or a few other things that generate uevents to add the environment variables."
static int AGEXDRV_dev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_PCI_probe\n");
    if( add_uevent_var(env, "DEVMODE=%#o", 0666) != 0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" add_uevent_var() failed\n");
    return 0;
}



//<====================================>
//	Module
//<====================================>

//wird aufgerufen wenn das Modul geladen wird
int AGEXDrv_init(void)
{
	int res,i;

	pr_devel(MODDEBUGOUTTEXT " enter init\n");

	//im sicher zu sein das bei einem 64Bit Build auch dieses Define gesetzt war...
#ifdef CONFIG_64BIT 
	if(	sizeof(void*) != sizeof(u64) ){
		printk(KERN_ERR MODDEBUGOUTTEXT" invalid 64Bit Build!\n");	return -EINVAL;
	}
#else	
	if(	sizeof(void*) != sizeof(u32) ){
		printk(KERN_ERR MODDEBUGOUTTEXT" invalid 64Bit Build!\n");	return -EINVAL;
	}
#endif


	/* init member */
	/**********************************************************************/
	_ModuleData.pModuleClass =  ERR_PTR(-EFAULT);

	for(i=0; i<MAX_DEVICE_COUNT; i++){
		AGEXDrv_InitDrvData(&_ModuleData.Devs[i]);
		_ModuleData.boIsMinorUsed[i] = FALSE;
	}
		

	/* sich n device nummer holen */
	/**********************************************************************/
	// steht dann in /proc/devices
	res = alloc_chrdev_region(
			&_ModuleData.FirstDeviceNumber,	/* out die 1. nummer */
			0,					/* 1. minor am besten 0 */
			MAX_DEVICE_COUNT,	/* wie viele */
			MODMODULENAME);		/* name vom driver */
	if (res < 0) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" can't get major!\n");
		return res;
	}
	else
		pr_devel(MODDEBUGOUTTEXT" major %d, minor %d, anz %d\n",MAJOR(_ModuleData.FirstDeviceNumber),MINOR(_ModuleData.FirstDeviceNumber), MAX_DEVICE_COUNT);
	//sicher ist sicher (wir nutzen den Minor als Index für _ModuleData_Devs[])
	if( MINOR(_ModuleData.FirstDeviceNumber) != 0){
		printk(KERN_WARNING MODDEBUGOUTTEXT" start minor must we zero!\n");
		return -EINVAL;
	}


	/* erzeugt eine Sysfs class */
	/**********************************************************************/
	_ModuleData.pModuleClass = class_create(THIS_MODULE, MODCLASSNAME);
	if( IS_ERR(_ModuleData.pModuleClass))
		printk(KERN_WARNING MODDEBUGOUTTEXT" can't create sysfs class!\n");
	
	//add the uevend handler
	if( !IS_ERR(_ModuleData.pModuleClass) )
		_ModuleData.pModuleClass->dev_uevent = AGEXDRV_dev_uevent;	//send uevents to udev, so it'll create the /dev/node


	/* macht dem Kernel den treiber für PCIdevs bekannt */
	/**********************************************************************/
	if( pci_register_driver(&AGEXDrv_pci_driver) !=0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" pci_register_driver failed!\n");

#ifdef __aarch64__
	if( spi_register_driver(&imago_spi_driver) !=0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" spi_register_driver failed!\n");
#endif
	
	printk(KERN_INFO MODDEBUGOUTTEXT" init done (%s [%s])\n", MODDATECODE, MODVERSION);
	pr_devel(MODDEBUGOUTTEXT" leave init\n");
	return 0;
}

//wird aufgerufen wenn das Modul entladen wird
//Note: kann nicht entladen werden wenn es noch genutzt wird
void AGEXDrv_exit(void)
{
	pr_devel(MODDEBUGOUTTEXT" enter exit\n");

#ifdef __aarch64__
	spi_unregister_driver(&imago_spi_driver);
#endif

	//wir können uns nicht mehr um PCIdevs kümmern:-)
	pci_unregister_driver(&AGEXDrv_pci_driver);
		
	//gibt Sysfs class frei
	if(!IS_ERR(_ModuleData.pModuleClass))
		class_destroy(_ModuleData.pModuleClass);

	//gibt die dev Nnummern frei
	//Note: "cleanup_module is never called if registering failed"
	unregister_chrdev_region(_ModuleData.FirstDeviceNumber, MAX_DEVICE_COUNT);
}

//setzt alle Felder auf definierte Werte
void AGEXDrv_InitDrvData(PDEVICE_DATA pDat)
{
	int i, iChannel, iTC;

	//Note: darf nicht wegen  SG... 
	//memset(pDat, 0, sizeof(DEVICE_DATA));

	/* Module */
	pDat->boIsDeviceOpen	= FALSE;
	pDat->dev				= NULL;
	pDat->DeviceSubType		= SubType_Invalid;
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
		sema_init(&pDat->DeviceSem,1);		//1<>frei
	#else
		init_MUTEX( &pDat->DeviceSem);
	#endif

	/* SUN */
	spin_lock_init(&pDat->lock);

	for (i = 0; i<MAX_IRQDEVICECOUNT; i++) {
		pDat->SunDeviceData[i].requestState = SUN_REQ_STATE_FREE;
		pDat->SunDeviceData[i].serialID = 0;
	}

	pDat->boIsBAR0Requested = FALSE;
	pDat->pVABAR0			= NULL;
	pDat->pVACommonBuffer 	= NULL;
	pDat->pBACommonBuffer	= 0;
	pDat->boIsIRQOpen		= FALSE;


	/* DMA */
	//Note: 
	//	- da wir hier noch nicht wissen ob wie �berhaupt eine DMA haben immer init
	// 	- auch alle m�glichen DMAChannels/TCs initen da wir noch nicht wissen wie viele wir haben werden
	pDat->DMARead_channels	= 0;
	pDat->DMARead_TCs		= 0;
	pDat->DMARead_SGs		= 0;
	spin_lock_init(&pDat->DMARead_SpinLock);
	for (iChannel = 0; iChannel < MAX_DMA_READ_DMACHANNELS; iChannel++)
	{
		PDMA_READ_CHANNEL pChannel = &pDat->DMARead_Channel[iChannel];

		memset(pChannel->jobBuffers, 0, sizeof(pChannel->jobBuffers));
		memset(&pChannel->dummyJob, 0, sizeof(pChannel->dummyJob));
		INIT_KFIFO(pChannel->Jobs_ToDo);
		INIT_KFIFO(pChannel->Jobs_Done);
		#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
			sema_init(&pChannel->WaitSem,0);		//1<>frei
		#else
			init_MUTEX_LOCKED( &pChannel->WaitSem);
		#endif

		for(iTC = 0; iTC < MAX_DMA_READ_CHANNELTCS; iTC++)
		{
			PDMA_READ_TC pTC = pChannel->TCs+iTC;

			pTC->boIsUsed 					= FALSE;
		}//for TCs
	}//for DMAChannesl
}

