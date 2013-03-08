/*
 * AGEXDrv.c
 *
 * The entry point to the kernel module
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
 * -> module (un)load, read & write & IO, PCI probe/remove, IRQ(ISR)
 *
 */

#include "AGEXDrv.h"


MODULE_LICENSE("GPL");

MODULE_DESCRIPTION("Kernel module for the VisionBox AGE-X");
MODULE_AUTHOR("IMAGO Technologies GmbH");


/* prototypes */
static int AGEXDrv_PCI_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void AGEXDrv_PCI_remove(struct pci_dev *dev);

/* Members :-) */
bool _boIsIRQOpen;
bool _boIsBAR0Requested;
void* _PCI_IOMEM_StartAdr;
unsigned long _BAR0_Len;
dev_t 		_DeviceNumber;
struct cdev	_Device;
struct cdev * _pDevice;
struct class *_pClassType;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	DEFINE_SEMAPHORE(_Driver_Sem);		//erzeugt eine mit sem_count = 1 -> frei
#else
	DECLARE_MUTEX(_Driver_Sem);
#endif

bool _boIsDeviceIDUsed[MAX_IRQDEVICECOUNT];
LONGTERM_IOREQUEST  _LongTermRequestList[MAX_LONG_TERM_IO_REQUEST];


//struct mit den file fns
struct file_operations AGEXDrv_fops = {
	.owner = THIS_MODULE,
	.read =  AGEXDrv_read,
	.write = AGEXDrv_write,
	.unlocked_ioctl = AGEXDrv_unlocked_ioctl,
	.llseek = no_llseek,
};

//für welche PCI IDs sind wir zuständi?
static struct pci_device_id AGEXDrv_ids[] = {
	{ PCI_DEVICE(0x1204/*VendorID (Lattice Semi)*/, 0x0200 /*DeviceID*/), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, AGEXDrv_ids);		//macht dem kernel bekannt was dieses modul für PCI devs kann

//struct mit den PCICallBacks
static struct pci_driver AGEXDrv_pci_driver = {
	.name = "agexdrv",
	.id_table = AGEXDrv_ids,
	.probe = AGEXDrv_PCI_probe,
	.remove = AGEXDrv_PCI_remove,
};



//<====================================>
//	PCI fns
//		wegen Hot-Plug gibt es CallBacks
//<====================================>

//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterstützt, 0 ja <0 nein
static int AGEXDrv_PCI_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	unsigned long bar0_start,bar0_len;

	pr_devel("agexdrv: AGEXDrv_PCI_probe\n");

	//hat ein counter
	if (pci_enable_device(dev) < 0) {
		printk(KERN_ERR "agexdrv: pci_enable_device failed\n");
		return -EIO;
	}

	//BAR0>
	//das bar0 prüfen
	bar0_start = pci_resource_start(dev, 0);
	bar0_len = pci_resource_len(dev, 0);
	if ( !(pci_resource_flags(dev, 0) & IORESOURCE_MEM)){
		printk(KERN_WARNING "agexdrv: invalid bar0\n");
		return -ENODEV;
	}
	else
		pr_devel("agexdrv: bar0> 0x%lx, %lubytes\n",bar0_start,bar0_len);

	//bar0 mappen request_region();
	if( request_mem_region(bar0_start, bar0_len,"agexdrv") == NULL){
		_boIsBAR0Requested = FALSE;
		printk(KERN_ERR "agexdrv: request_mem_region failed!\n");
		return -EBUSY;
	}
	else
	{
		_boIsBAR0Requested = TRUE;
		_PCI_IOMEM_StartAdr = ioremap(bar0_start,bar0_len); //das setzen der adr zeigt auch an das wir (R/W) fns aufs device schreiben dürfen
		if(_PCI_IOMEM_StartAdr == NULL)
			printk(KERN_ERR "agexdrv: ioremap failed!\n");
		else
			pr_devel("agexdrv: map bar0> 0x%lx to 0x%p\n",bar0_start,_PCI_IOMEM_StartAdr);
	}
	_BAR0_Len = bar0_len;



	//IRQ>
	//das gibt die (un)gemapped nummer zurück lspci zeigt die mapped an!
	//if(pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &PCIIRQ_Number) ) {
	if( request_irq(dev->irq,	/* die IRQ nummer */
			AGEXDrv_interrupt,	/* die IRQ fn */
			IRQF_SHARED,		/* shared */
			"agexdrv",			/* name wird in /proc/interrupts angezeigt */
			&_boIsIRQOpen		/* unique identifier/ wird auch dem CallBack mit gegeben */
			) != 0)
	{
		printk(KERN_ERR "agexdrv: request_irq failed\n");
		_boIsIRQOpen = FALSE;
	}
	else
	{
		AGEXDrv_SwitchInterruptOn(TRUE);

		pr_devel("agexdrv: IRQ> %d \n",dev->irq);
		_boIsIRQOpen = TRUE;
	}



	return 0;
}

//wird aufgerufen wenn das PCIdev removed wird
static void AGEXDrv_PCI_remove(struct pci_dev *dev)
{
	pr_devel("agexdrv: AGEXDrv_PCI_remove\n");

	//IRQ zuückgeben
	if(_boIsIRQOpen){
		AGEXDrv_SwitchInterruptOn(FALSE);
		free_irq(dev->irq, &_boIsIRQOpen);
	}

	//unmappen
	if(_PCI_IOMEM_StartAdr != NULL)
		iounmap(_PCI_IOMEM_StartAdr);
	_PCI_IOMEM_StartAdr = NULL;

	if(_boIsBAR0Requested)
		release_mem_region( pci_resource_start(dev, 0), pci_resource_len(dev, 0) );
	_boIsBAR0Requested = FALSE;

	pci_disable_device(dev);
}



//<====================================>
//	Module
//<====================================>

//wird aufgerufen wenn das Modul geladen wird
static int AGEXDrv_init(void)
{
	int res,i;
	dev_t devNumber = 0; //major & minor (wie genau das kann sich ändern)

	pr_devel("agexdrv: enter init\n");

	/* init member */
	_boIsIRQOpen		= FALSE;
	_boIsBAR0Requested 	= FALSE;
	_PCI_IOMEM_StartAdr = NULL;
	_pDevice = NULL;	//damit wir erkennen ob wir das dev öffnen konnten
	_pClassType =  ERR_PTR(-EFAULT);

	for(i=0;i<MAX_IRQDEVICECOUNT;i++)
		_boIsDeviceIDUsed[i] = FALSE;

	for(i=0;i<MAX_LONG_TERM_IO_REQUEST;i++)
	{
		_LongTermRequestList[i].boIsInFPGA = FALSE;
		_LongTermRequestList[i].boIsInProcessUse = FALSE;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
		sema_init(&_LongTermRequestList[i].WaitSem,0);
#else
		init_MUTEX_LOCKED( &_LongTermRequestList[i].WaitSem);
#endif
		memset(_LongTermRequestList[i].IRQBuffer,0, MAX_SUNPACKETSIZE); //nicht notwendig
		_LongTermRequestList[i].IRQBuffer_anzBytes = 0;
	}


	/* sich eine device nummer holen */
	// steht dann in /proc/devices
	res = alloc_chrdev_region(
			&devNumber,		/* out die 1. nummer */
			0,				/* 1. minor am besten 0 */
			1,				/* wie viele */
			"agexdrv");		/* name vom driver */
	if (res < 0) {
		printk(KERN_WARNING "agexdrv: can't get major!\n");
		return res;
	}
	else
		pr_devel("agexdrv: major %d, minor %d\n",MAJOR(devNumber),MINOR(devNumber));
	_DeviceNumber = devNumber;



	/* init & fügt das device hinzu */
	cdev_init(&_Device, &AGEXDrv_fops);
	_Device.owner 	= THIS_MODULE;
	_Device.ops 	= &AGEXDrv_fops;	//notwendig in den quellen wird fops gesetzt?


	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
	res = cdev_add(&_Device, _DeviceNumber, 1);
	if(res < 0)
		printk(KERN_WARNING "agexdrv: can't add device!\n");
	else
		_pDevice = &_Device;


	/* erzeugt eine Sysfs class & eintrag*/
	_pClassType = class_create(THIS_MODULE, "agexdrv");
	if( IS_ERR(_pClassType))
		printk(KERN_WARNING "agexdrv: can't create sysfs class!\n");

	//seend uevents to udev, so it'll create the /dev node*/
	//war mal class_device_create
	if( !IS_ERR(_pClassType) )
	{
		struct device * temp = device_create(
				_pClassType, 	/* die Type classe */
				NULL, 			/* pointer zum Eltern, dann wird das dev ein Kind vom parten*/
				_DeviceNumber,	/* die nummer zum device */
				NULL,
				"agexdrv0"		/*string for the class device's name */
				);				/* ...*/

		if( IS_ERR(temp))
			printk(KERN_WARNING "agexdrv: can't create sysfs device!\n");
	}


	/* macht dem Kernel den treiber für PCIdevs bekannt */
	if( pci_register_driver(&AGEXDrv_pci_driver) !=0)
		printk(KERN_WARNING "agexdrv: pci_register_driver failed!\n");

	printk(KERN_INFO"agexdrv: init done (%s [%s])\n", pBuildTime, pVersion);
	pr_devel("agexdrv: leave init\n");
	return 0;
}

//wird aufgerufen wenn das Modul entladen wird
//Note: kann nicht entladen werden wenn es noch genutzt wird
static void AGEXDrv_exit(void)
{
	pr_devel("agexdrv: enter exit\n");

	//"... this function busy-waits until the tasklet exits..."
	tasklet_disable(&_AGEXDrv_tasklet);

	//wir können uns nicht mehr um PCIdevs kümmern:-)
	pci_unregister_driver(&AGEXDrv_pci_driver);

	//Achtung! wenn wir es nicht adden konnten
	if(_pDevice != NULL)
		cdev_del(_pDevice);

	//gibt Sysfs device frei
	if(!IS_ERR(_pClassType))
	{
		device_destroy(_pClassType,_DeviceNumber);

		class_destroy(_pClassType);
	}

	//gibt die dev nummer frei
	//Note: "cleanup_module is never called if registering failed"
	unregister_chrdev_region(_DeviceNumber, 1);
}

//übergibt/speichert die pointer
module_init(AGEXDrv_init);
module_exit(AGEXDrv_exit);

