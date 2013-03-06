/*
 * AGEXDrv.c
 *
 * Hier sind die "fns" drin welche vom user Mode aufgerufen werden können
 *
 * -> module (un)load, read, write & IO
 *
 *  Created on: 20.12.2011
 *      Author: imago
 */

#include "AGEXDrv.h"


MODULE_LICENSE("GPL");

/* prototypes */
ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos);
ssize_t AGEXDrv_write (struct file *filp, const char __user *buf, size_t count,loff_t *pos);
long AGEXDrv_unlocked_ioctl (struct file *filp, unsigned int cmd,unsigned long arg);
static int AGEXDrv_PCI_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void AGEXDrv_PCI_remove(struct pci_dev *dev);
void AGEXDrv_tasklet (unsigned long unused);

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

DECLARE_TASKLET(_AGEXDrv_tasklet,AGEXDrv_tasklet,0);


//<====================================>
//	IRQ fns
//
//<====================================>

//Schaltet im PCI-Gerät die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(const bool boTurnOn)
{
	u32 regVal;

	//alles gut?
	if(_PCI_IOMEM_StartAdr == NULL)
		return;

	//was schreiben
	if(boTurnOn)
		regVal = 0x1;
	else
		regVal = 0x0;

	iowrite32(regVal, _PCI_IOMEM_StartAdr + ISR_ONOFF_OFFSET);
}



//< HWIRQ > nur prüfen ob von uns, ja tasklet starten
irqreturn_t AGEXDrv_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	u32 regVal;

	/* war der int von uns? reg lesen  */
	if(_PCI_IOMEM_StartAdr == NULL)
		return IRQ_NONE;

	regVal = ioread32(_PCI_IOMEM_StartAdr + ISR_AVAILABLE_OFFSET);
	if(regVal & 0x1)
	{
		//INTs abschalten
		AGEXDrv_SwitchInterruptOn(FALSE);

		// trigger the tasklet
		tasklet_schedule(&_AGEXDrv_tasklet);

		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;
}


//SWI bzw. DPC Achtung hat keinen process context
void AGEXDrv_tasklet (unsigned long unused)
{
	u32 regVal, fifoLevel, header0, header1;
	u16 deviceID, wordCount, index;
	bool boPacketDataDone = FALSE;

	printk(KERN_DEBUG "agexdrv: AGEXDrv_tasklet\n");


	//Alles gut?
	if(_PCI_IOMEM_StartAdr == NULL)
		return;


	/* vom FPGA den PaketKopf einlesen/auswerten */
	// FIFO-Fuellstand einlesen
	regVal = ioread32(_PCI_IOMEM_StartAdr + ISR_AVAILABLE_OFFSET);

	// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
	fifoLevel = (regVal >> 1) & 0x1FF;

	if (fifoLevel < 3){
		// sollte nicht auftreten
		printk(KERN_ERR "agexdrv: SUN Error: FIFO level to small: %d\n", fifoLevel);
		return;
	}
	printk(KERN_DEBUG "agexdrv: SUN FIFO level: %d\n", fifoLevel);

	// Header0 und Header1 einlesen
	header0 = ioread32(_PCI_IOMEM_StartAdr);
	header1 = ioread32(_PCI_IOMEM_StartAdr);
	wordCount = header1 & 0xFF;
	deviceID = (header1>>20) & (MAX_IRQDEVICECOUNT-1);
	printk(KERN_DEBUG "agexdrv: SUN WordCount: %d\n", wordCount);




	/* läuft noch eine Anfrage für die ID? */
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		// DeviceID auswerten (warten wir auf das SUNPaket?)
		// 	-> DeviceID muss passen und wir haben die daten noch nicht reingefüllt
		if(		( _LongTermRequestList[index].boIsInFPGA == TRUE )
			&&	(_LongTermRequestList[index].DeviceID == deviceID) )
		{
			//Wartet (noch) ein Request aufs Paket?
			if(_LongTermRequestList[index].boIsInProcessUse == TRUE)
			{
				printk(KERN_DEBUG "agexdrv: Completing request %d, id %d\n",index,deviceID);

				//wenns in den Buffer passt, Daten einlesen
				if(MAX_SUNPACKETSIZE >= 4*wordCount+8) // Paket mit Header0/1
				{
					_LongTermRequestList[index].IRQBuffer[0] = header0;
					_LongTermRequestList[index].IRQBuffer[1] = header1;

					ioread32_rep(_PCI_IOMEM_StartAdr,_LongTermRequestList[index].IRQBuffer+2,wordCount);

					boPacketDataDone	= TRUE;
					_LongTermRequestList[index].IRQBuffer_anzBytes = 4*wordCount+8;
				}
				else
				{
					_LongTermRequestList[index].IRQBuffer_anzBytes = 0;
					printk(KERN_WARNING "agexdrv: LongTerm buffer to small\n");
				}

				//den process aufwecken
				up(&_LongTermRequestList[index].WaitSem);

			}//end if boIsInProcessUse == true


			//Eintrag freigeben
			//	-> egal ob die DLL noch auf eine Anfrage wartet oder nicht
			//	das Paket für diese ID ist gekommen
			_LongTermRequestList[index].boIsInFPGA = FALSE;

		}//end if DeviceID == [index].DeviceID
	}//end for MAX_LONG_TERM_IO_REQUEST



	/* Immer! egal ob sie einer will oder nicht die daten weglesen */
	if (!boPacketDataDone)
	{
		u16 i;
		volatile u32 dummy;
		for (i=0; i<wordCount; i++)
			dummy = ioread32(_PCI_IOMEM_StartAdr);
	}


	/* Re-enable the interrupt */
	AGEXDrv_SwitchInterruptOn(TRUE);

	return;
}


//<====================================>
//	PCI fns
//		wegen Hot-Plug gibt es CallBacks
//<====================================>

//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterstützt, 0 ja <0 nein
static int AGEXDrv_PCI_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	unsigned long bar0_start,bar0_len;

	printk(KERN_DEBUG "agexdrv: AGEXDrv_PCI_probe\n");

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
		printk(KERN_DEBUG "agexdrv: bar0> 0x%lx, %lubytes\n",bar0_start,bar0_len);

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
			printk(KERN_DEBUG "agexdrv: map bar0> 0x%lx to 0x%p\n",bar0_start,_PCI_IOMEM_StartAdr);
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

		printk(KERN_DEBUG "agexdrv: IRQ> %d \n",dev->irq);
		_boIsIRQOpen = TRUE;
	}



	return 0;
}

//wird aufgerufen wenn das PCIdev removed wird
static void AGEXDrv_PCI_remove(struct pci_dev *dev)
{
	printk(KERN_DEBUG "agexdrv: AGEXDrv_PCI_remove\n");

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
//	File fns
//<====================================>

//Note: alte ioctl war unter "big kernel lock"
//http://lwn.net/Articles/119652/
long AGEXDrv_unlocked_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	long ret = 0;

	//Note: beim 1. write bekommen wir ein CMD:1, Type 'T', Size: 0 <-> FIONBIO (linux/include/asm-i386/ioctls.h, line 41)
	printk(KERN_DEBUG "agexdrv: ioctl (CMD %d, MAGIC %c, size %d)\n",_IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd));

	/* Alles gut */
	//ist ist das CMD eins für uns?
	if (_IOC_TYPE(cmd) != AGEXDRV_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > AGEXDRV_IOC_MAXNR) return -ENOTTY;

	//bei uns ist arg ein Pointer, und testen ob wir ihn nutzen dürfen (richtung aus UserSicht)
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;


	/* jetzt die CMDs auswerten */
	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&_Driver_Sem) != 0)
		return -EINTR;

//----------------------------->
	//hier wird das CMD ausgeführt
	ret = Locked_ioctl(cmd, (u8 __user *) arg, _IOC_SIZE(cmd) );
//<-----------------------------
	up(&_Driver_Sem);


	return ret;
}

ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	u32 BytesToWrite, DeviceID;
	long res;
	unsigned long jiffiesTimeOut;
	s8 Index=-1;

	/*Wie es geht:
	 * 1. (locked)
	 * 	- gibt es noch einen freien Platz in der LongTerm Liste, ja -> beide Flags setzen, sem auf unfrei setzen
	 * 	--> boIsInProcessUse <> es ist eine Anfrage im FPGA
	 * 	--> process wartet <> ein process is waiting for the sem
	 * 2. (locked)
	 * 	- ein write machen (2 DWORD im buffer gibt die anz BytesToWrite an)
	 * 3.
	 *  - auf die sem warten, wenn sem fehler, process flag löschen aber eintrag bleibt gültig
	 * (4.) async
	 *  - IRQ, von uns -> tasklet starten
	 *  - tasklet,
	 *  --> paket auslesen(immer),
	 *  --> gibt es einen gültigen LongTerm Eintrag ja, daten dort einfüllen, flag löschen
	 *  --> wenn auf die sem gewarted wird, aufwecken
	 * 5.
	 *	- Antwort in UserBuf füllen
	 *	- beide flags löschen
	 */

	printk(KERN_DEBUG "agexdrv: read (%d Bytes)\n", (int)count);

	/* Alles gut? */
	//mem ok?
	if( count > _BAR0_Len)
		return -EFBIG;
	if(_PCI_IOMEM_StartAdr == NULL)
		return -EFBIG;
	if(_boIsIRQOpen == FALSE)
		return -EFAULT;
	if(count < (2*4))
		return -EFAULT;

	//dürfen wir den mem nutzen?
	if( !access_ok(VERIFY_WRITE, buf, count) )
		return -EFAULT;


	/* freien eintrag suchen */
	//args lesen
	if( get_user(DeviceID, buf) != 0)
		return -EFAULT;
	if( get_user(BytesToWrite, buf+4) != 0)
		return -EFAULT;
	if(BytesToWrite > MAX_SUNPACKETSIZE)
		return -EFBIG;
	if( (BytesToWrite+2*4) > count)
		return -EFBIG;

	printk(KERN_DEBUG "agexdrv: read, devID %d, BytesToWrite %d\n",DeviceID,BytesToWrite );

	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&_Driver_Sem) != 0)
		return -EINTR;
//----------------------------->
	//freien eintrag suchen
	Index = -1;
	res = Locked_startlongtermread(DeviceID);

	/* wenn ok jetzt ins FPGA schreiben */
	if(res >= 0){
		Index = (s8)res;
		res =  Locked_write(buf+2*4, BytesToWrite);
	}
//<-----------------------------
	up(&_Driver_Sem);

	if(Index <0 || Index > MAX_LONG_TERM_IO_REQUEST)	//nur um ganz sicher zu sein
		return -EFAULT;
	if(res < 0)
		goto EXIT_READ;

	/* warten auf Antwort */
	//if( down_killable(&_LongTermRequestList[Index].WaitSem) != 0){
	jiffiesTimeOut = msecs_to_jiffies(1*1000);
	if( down_timeout(&_LongTermRequestList[Index].WaitSem,jiffiesTimeOut) != 0){
		res = -EINTR;
		goto EXIT_READ;
	}


	/* daten copy */
	//zu viel für den user buf bzw. gültig?
	if( 	(_LongTermRequestList[Index].IRQBuffer_anzBytes > count)
		||	(_LongTermRequestList[Index].IRQBuffer_anzBytes > MAX_SUNPACKETSIZE)
		||	(_LongTermRequestList[Index].IRQBuffer_anzBytes == 0) ){
		res = -EFBIG;
		goto EXIT_READ;
	}
	//copy
	if( copy_to_user(buf, _LongTermRequestList[Index].IRQBuffer,_LongTermRequestList[Index].IRQBuffer_anzBytes ) != 0 ){
		res = -EFAULT;
		goto EXIT_READ;
	}
	else
		res = _LongTermRequestList[Index].IRQBuffer_anzBytes;



	//gibt den eintrag frei
EXIT_READ:
	_LongTermRequestList[Index].boIsInProcessUse = FALSE;
	return res;
}

ssize_t AGEXDrv_write (struct file *filp, const char __user *buf, size_t count,loff_t *pos)
{
	long res;

	printk(KERN_DEBUG "agexdrv: write (%d Bytes)\n", (int)count);

	/* Alles gut? */
	//mem ok?
	if( (count > _BAR0_Len) || (count > MAX_SUNPACKETSIZE))
		return -EFBIG;
	if(_PCI_IOMEM_StartAdr == NULL)
		return -EFBIG;

	//dürfen wir den mem nutzen?
	if( !access_ok(VERIFY_READ, buf, count) )
		return -EFAULT;

	/* jetzt kommt das schreiben */
	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&_Driver_Sem) != 0)
		return -EINTR;
//----------------------------->
	res =  Locked_write(buf, count);
//<-----------------------------
		up(&_Driver_Sem);

	return res;
}




//<====================================>
//	Module
//<====================================>

//wird aufgerufen wenn das Modul geladen wird
static int AGEXDrv_init(void)
{
	int res,i;
	dev_t devNumber = 0; //major & minor (wie genau das kann sich ändern)

	printk(KERN_DEBUG "agexdrv: enter init\n");

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
		init_MUTEX_LOCKED( &_LongTermRequestList[i].WaitSem);
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
		printk(KERN_INFO "agexdrv: major %d, minor %d\n",MAJOR(devNumber),MINOR(devNumber));
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

	printk(KERN_DEBUG "agexdrv: leave init\n");
	return 0;
}

//wird aufgerufen wenn das Modul entladen wird
//Note: kann nicht entladen werden wenn es noch genutzt wird
static void AGEXDrv_exit(void)
{
	printk(KERN_DEBUG "agexdrv: enter exit\n");

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

