/*
 * PCI.c
 *
 * PCI(e) Probe code
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
 *
 */

#include "AGEXDrv.h"


//struct mit den file fns
struct file_operations AGEXDrv_fops = {
	.owner	= THIS_MODULE,
	.open	= AGEXDrv_open,
	.read	= AGEXDrv_read,
	.write	= AGEXDrv_write,
	.unlocked_ioctl = AGEXDrv_unlocked_ioctl,
	.llseek = no_llseek,
};

struct sAGEXDrv_device_info AGEXDrv_device_info[] = {
	[SubType_AGEX] = {
		.name = "VisionBox AGE-X",
		.flags = AGEXDRV_FLAG_PCI},
	[SubType_AGEX2] = {
		.name = "VisionBox AGE-X2",
		.flags = AGEXDRV_FLAG_COMMONBUFFER},
	[SubType_MVC0] = {
		.name = "Machine Vision Controller",
		.flags = AGEXDRV_FLAG_COMMONBUFFER},
	[SubType_AGEX2_CL] = {
		.name = "VisionBox AGE-X2 CL",
		.flags = AGEXDRV_FLAG_COMMONBUFFER | AGEXDRV_FLAG_DMA2HOST | AGEXDRV_FLAG_PCI64BIT},
	[SubType_VCXM] = {
		.name = "VisionCam XM",
		.flags = AGEXDRV_FLAG_COMMONBUFFER | AGEXDRV_FLAG_DMA2HOST},
	[SubType_LEMANS] = {
		.name = "VisionBox LE MANS",
		.flags = AGEXDRV_FLAG_COMMONBUFFER},
	[SubType_PCIE_CL] = {
		.name = "PCIe CL",
		.flags = AGEXDRV_FLAG_COMMONBUFFER | AGEXDRV_FLAG_DMA2HOST | AGEXDRV_FLAG_PCI64BIT},
	[SubType_AGEX5] = {
		.name = "VisionBox AGE-X5",
		.flags = AGEXDRV_FLAG_COMMONBUFFER},
	[SubType_AGEX5_CL] = {
		.name = "VisionBox AGE-X5 CL",
		.flags = AGEXDRV_FLAG_COMMONBUFFER | AGEXDRV_FLAG_DMA2HOST | AGEXDRV_FLAG_PCI64BIT},
	[SubType_DAYTONA] = {
		.name = "VisionBox DAYTONA",
		.flags = AGEXDRV_FLAG_SPI},
};

//<====================================>
//	PCI fns
//		wegen Hot-Plug gibt es CallBacks
//<====================================>
//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterst�tzt, 0 ja <0 nein
int AGEXDrv_PCI_probe(struct pci_dev *pcidev, const struct pci_device_id *id)
{
	u64 bar0_start,bar0_len;
	int res,i, DevIndex;
	u8 tempDevSubType;
	pci_set_drvdata(pcidev, NULL);	

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_PCI_probe\n");

	tempDevSubType = id->driver_data;
	if (tempDevSubType == SubType_Invalid || tempDevSubType > ARRAY_SIZE(AGEXDrv_device_info))
	{
		printk(KERN_WARNING MODDEBUGOUTTEXT" unknown device identifier (%u)\n", tempDevSubType);
		return -EINVAL;
	}

	pr_devel(MODDEBUGOUTTEXT" found '%s' device\n", AGEXDrv_device_info[tempDevSubType].name);


	//>freie Minor Nummer?
	/**********************************************************************/
	DevIndex =-1;
	for(i=0; i<MAX_DEVICE_COUNT; i++)
	{
		if(!_ModuleData.boIsMinorUsed[i])
		{
			DevIndex = i;
			AGEXDrv_InitDrvData(&_ModuleData.Devs[DevIndex]);			
			_ModuleData.Devs[DevIndex].DeviceSubType= tempDevSubType;
			_ModuleData.Devs[DevIndex].DeviceNumber = MKDEV(MAJOR(_ModuleData.FirstDeviceNumber), DevIndex);
			_ModuleData.Devs[DevIndex].pDeviceDevice = &pcidev->dev;
			_ModuleData.Devs[DevIndex].flags =  AGEXDrv_device_info[tempDevSubType].flags;
			pci_set_drvdata(pcidev, &_ModuleData.Devs[DevIndex]);				//damit wir im AGEXDrv_PCI_remove() wissen welches def freigebene werden soll
			break;
		}
	}
	if(DevIndex==-1){
		printk(KERN_WARNING MODDEBUGOUTTEXT" no free Minor-Number found!\n"); return -EINVAL;}
	else
		pr_devel(MODDEBUGOUTTEXT" use major/minor (%d:%d)\n", MAJOR(_ModuleData.Devs[DevIndex].DeviceNumber), MINOR(_ModuleData.Devs[DevIndex].DeviceNumber));

	//>PCI device on(setzt Bits im PCI Config Mem)
	/**********************************************************************/
	if (pci_enable_device(pcidev) < 0) {
		printk(KERN_ERR MODDEBUGOUTTEXT" pci_enable_device failed\n"); return -EIO;}

	//Enable PCI Bus Master
	pci_set_master(pcidev);
	

	//>BAR0
	/**********************************************************************/
	//das bar0 pr�fen
	bar0_start = pci_resource_start(pcidev, 0);
	bar0_len = pci_resource_len(pcidev, 0);
	if ( !(pci_resource_flags(pcidev, 0) & IORESOURCE_MEM)){
		printk(KERN_WARNING MODDEBUGOUTTEXT" invalid bar0\n");
		return -ENODEV;
	}
	else
		pr_devel(MODDEBUGOUTTEXT" bar0> 0x%llx, %llu [Bytes]\n",bar0_start,bar0_len);

	//bar0 mappen request_region(); request_mem_region() macht kein mapping sondern 'nur' eine 'reservation'
	if( request_mem_region(bar0_start, bar0_len,MODMODULENAME) == NULL){
		_ModuleData.Devs[DevIndex].boIsBAR0Requested = FALSE;
		printk(KERN_ERR MODDEBUGOUTTEXT" request_mem_region failed!\n");
		return -EBUSY;
	}
	else
	{
		_ModuleData.Devs[DevIndex].boIsBAR0Requested = TRUE;
		_ModuleData.Devs[DevIndex].pVABAR0 = ioremap(bar0_start,bar0_len); //das setzen der adr zeigt auch an das wir (R/W) fns aufs device schreiben d�rfen
		if(_ModuleData.Devs[DevIndex].pVABAR0 == NULL)
			printk(KERN_ERR MODDEBUGOUTTEXT" ioremap failed!\n");
		else
			pr_devel(MODDEBUGOUTTEXT" map bar0> 0x%llx to 0x%p\n",bar0_start,_ModuleData.Devs[DevIndex].pVABAR0);
	}


	//>DMA(Coherent) Buffer (nicht AGEX1)
	/**********************************************************************/
	if( IS_TYPEWITH_COMMONBUFFER(&_ModuleData.Devs[DevIndex]) )
	{
		u8 MaxDAMAddressSize = 32;
		if( IS_TYPEWITH_PCI64BIT(&_ModuleData.Devs[DevIndex]) )
			MaxDAMAddressSize = 64;

		//sagt das wir xxBit k�nnen
		//https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
		// "...By default, the kernel assumes that your device can address the full 32-bits...
		//  ... It is good style to do this even if your device holds the default setting ..."
		if( dma_set_mask(&pcidev->dev, DMA_BIT_MASK(MaxDAMAddressSize) ) != 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_mask failed!\n"); return -EIO;}
		//ab 2.6.34
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
		if( dma_set_coherent_mask(&pcidev->dev, DMA_BIT_MASK(MaxDAMAddressSize) ) != 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_coherent_mask failed!\n"); return -EIO;}
#endif


		//gibt speicher zur�ck ohne/mit cache off und auf Page ausgerichtet
		// https://www.kernel.org/doc/Documentation/DMA-API.txt
		// "... Consistent memory is memory for which a write by either the device or
		//	the processor can immediately be read by the processor or device
		//	without having to worry about caching effects..."
		_ModuleData.Devs[DevIndex].pVACommonBuffer = dma_alloc_coherent(	&pcidev->dev,	/* f�r welches device */
																			PAGE_SIZE,		/* gr��e in Bytes (wird eh min zu einer Page)*/
																			&_ModuleData.Devs[DevIndex].pBACommonBuffer,
																			GFP_KERNEL);	/* zone wird �ber die maske eingestellt, sonst nur noch ob GFP_ATOMIC)*/
		if(_ModuleData.Devs[DevIndex].pVACommonBuffer == 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT" dma_alloc_coherent failed!\n"); return -ENOMEM;}
		else
		{
			if( sizeof(dma_addr_t) == sizeof(u32) )
				pr_devel(MODDEBUGOUTTEXT" DMABuffer> VA: 0x%p, BA: 0x%x, %ld [Bytes]\n", 
					_ModuleData.Devs[DevIndex].pVACommonBuffer, (u32)_ModuleData.Devs[DevIndex].pBACommonBuffer, PAGE_SIZE);
			else
				pr_devel(MODDEBUGOUTTEXT" DMABuffer> VA: 0x%p, BA: 0x%llx, %ld [Bytes]\n",
					_ModuleData.Devs[DevIndex].pVACommonBuffer, (u64)_ModuleData.Devs[DevIndex].pBACommonBuffer, PAGE_SIZE);
		}
		memset(_ModuleData.Devs[DevIndex].pVACommonBuffer, 0 ,PAGE_SIZE);	//es gibt ab 3.2 dma_zalloc_coherent()
	}


	//>DMA2Host Buffer (also CL, VCXM ...)
	/**********************************************************************/
	if( IS_TYPEWITH_DMA2HOST(&_ModuleData.Devs[DevIndex]) )
	{
		//damit z.b dma_map_sg() (mit einer IOMMU) nicht zuviel zusammengefasst
		// aber >nicht< f�r sg_alloc_table_from_pages()!
		if( dma_set_max_seg_size(&pcidev->dev, DMA_READ_TC_SG_MAX_BYTECOUNT) != 0 )		
			{printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_max_seg_size failed!\n"); return -EIO;}
	}

	
	
	//>IRQ & tasklet
	/**********************************************************************/
	//tasklet
	tasklet_init(&_ModuleData.Devs[DevIndex].IRQTasklet, AGEXDrv_tasklet, (unsigned long)&_ModuleData.Devs[DevIndex]);

	//msi einschalten
	//http://www.mjmwired.net/kernel/Documentation/MSI-HOWTO.txt
	// "... to call this API before calling request_irq()..."
	if( IS_TYPEWITH_COMMONBUFFER(&_ModuleData.Devs[DevIndex]) ) 
	{
		if( pci_enable_msi(pcidev) != 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT"pci_enable_msi failed!\n"); return -EIO;}
	}

	//das gibt die (un)gemapped nummer zur�ck lspci zeigt die mapped an!
	//if(pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &PCIIRQ_Number) ) {
	if( request_irq(pcidev->irq,/* die IRQ nummer */
			AGEXDrv_interrupt,	/* die IRQ fn */
			IRQF_SHARED,		/* shared */
			MODMODULENAME,		/* name wird in /proc/interrupts angezeigt */
			&_ModuleData.Devs[DevIndex]	/* unique identifier/ wird auch dem CallBack mit gegeben */
			) != 0)
	{
		printk(KERN_ERR MODDEBUGOUTTEXT" request_irq failed\n");
		if( IS_TYPEWITH_COMMONBUFFER(&_ModuleData.Devs[DevIndex]) )
			pci_disable_msi(pcidev);

		_ModuleData.Devs[DevIndex].boIsIRQOpen = FALSE;
		return -EIO;
	}

	AGEXDrv_SwitchInterruptOn(&_ModuleData.Devs[DevIndex], TRUE);

	pr_devel(MODDEBUGOUTTEXT" IRQ> %d \n",pcidev->irq);
	_ModuleData.Devs[DevIndex].boIsIRQOpen = TRUE;


	//>dev init & f�gt das es hinzu
	/**********************************************************************/
	cdev_init(&_ModuleData.Devs[DevIndex].DeviceCDev, &AGEXDrv_fops);
	_ModuleData.Devs[DevIndex].DeviceCDev.owner = THIS_MODULE;
	_ModuleData.Devs[DevIndex].DeviceCDev.ops 	= &AGEXDrv_fops;	//notwendig in den quellen wird fops gesetzt?

	//f�gt ein device hinzu, nach der fn k�nnen FileFns genutzt werden
	res = cdev_add(&_ModuleData.Devs[DevIndex].DeviceCDev, _ModuleData.Devs[DevIndex].DeviceNumber, 1/*wie viele ab startNum*/);
	if(res < 0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" can't add device!\n");
	else
		_ModuleData.Devs[DevIndex].boIsDeviceOpen = TRUE;


	//> in Sysfs class eintragen
	/**********************************************************************/			
	//war mal class_device_create
	if( !IS_ERR(_ModuleData.pModuleClass) )
	{		
		char devName[128];
		struct device *temp;

		sprintf(devName, "%s%d", MODMODULENAME, MINOR(_ModuleData.Devs[DevIndex].DeviceNumber));
		temp = device_create(
				_ModuleData.pModuleClass, 	/* die Type classe */
				NULL, 			/* pointer zum Eltern, dann wird das dev ein Kind vom parten*/
				_ModuleData.Devs[DevIndex].DeviceNumber, /* die nummer zum device */
				NULL,
				devName			/*string for the device's name */
				);

		if( IS_ERR(temp))
			printk(KERN_WARNING MODDEBUGOUTTEXT" can't create sysfs device!\n");
	}


	// init von allem ist durch
	printk(KERN_INFO MODDEBUGOUTTEXT" pci probe done (0x%04x:0x%04x <> %d:%d)\n",
		id->vendor, id->device, MAJOR(_ModuleData.Devs[DevIndex].DeviceNumber), MINOR(_ModuleData.Devs[DevIndex].DeviceNumber));	
	_ModuleData.boIsMinorUsed[DevIndex] = TRUE;

	return 0;
}


//wird aufgerufen wenn das PCIdev removed wird
void AGEXDrv_PCI_remove(struct pci_dev *pcidev)
{
	//Note: wenn PCI_remove() aufgerufen wird, 
	// darf kein UserThread mehr im Teiber sein bzw. noch reinspringen weil sonst... bum 	
	u32 i;
	PDEVICE_DATA pDevData = (PDEVICE_DATA)pci_get_drvdata(pcidev);
	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_PCI_remove (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	if(pDevData == NULL){
		printk(KERN_WARNING MODDEBUGOUTTEXT" device pointer is zero!\n"); return;}

	//DMA (versuchen) aufzur�umen
	// - devs die keine DMA haben, da ist DMARead_anzXXX=0 
	for(i = 0; i < pDevData->DMARead_anzChannels; i++)
	{
		AGEXDrv_DMARead_Abort_DMAChannel(pDevData, i);
		AGEXDrv_DMARead_Abort_DMAWaiter(pDevData, i);
	}

	//IRQ zu�ckgeben
	if(pDevData->boIsIRQOpen)
	{
		AGEXDrv_SwitchInterruptOn(pDevData, FALSE);
		free_irq(pcidev->irq, pDevData);
		if( IS_TYPEWITH_COMMONBUFFER(pDevData) )
			pci_disable_msi(pcidev);
	}
	
	//"... this function busy-waits until the tasklet exits..."
	tasklet_disable(&pDevData->IRQTasklet);

	//unmappen
	if(pDevData->pVABAR0 != NULL)
		iounmap(pDevData->pVABAR0);
	pDevData->pVABAR0 = NULL;

	if(pDevData->boIsBAR0Requested)
		release_mem_region( pci_resource_start(pcidev, 0), pci_resource_len(pcidev, 0) );
	pDevData->boIsBAR0Requested = FALSE;

	if( 	( IS_TYPEWITH_COMMONBUFFER(pDevData) )
		&& 	(pDevData->pVACommonBuffer != NULL) )
		dma_free_coherent(&pcidev->dev, PAGE_SIZE, pDevData->pVACommonBuffer, pDevData->pBACommonBuffer);
	pDevData->pVACommonBuffer = NULL;

	//das pci_dev nicht mehr f�r PCI nutzen (bzw. setzt Bits im PCIConfigMem)??
	pci_disable_device(pcidev);

	//device in der sysfs class l�schen
	if(!IS_ERR(_ModuleData.pModuleClass))	
		device_destroy(_ModuleData.pModuleClass, pDevData->DeviceNumber);

	//device l�schen
	if(pDevData->boIsDeviceOpen)
		cdev_del(&pDevData->DeviceCDev);
	pDevData->boIsDeviceOpen = FALSE;

}


