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



//<====================================>
//	PCI fns
//		wegen Hot-Plug gibt es CallBacks
//<====================================>
//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterstützt, 0 ja <0 nein
int AGEXDrv_PCI_probe(struct pci_dev *pcidev, const struct pci_device_id *id)
{
	u64 bar0_start,bar0_len;
	int res,i, DevIndex;
	u8 tempDevSubType;
	pci_set_drvdata(pcidev, NULL);	

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_PCI_probe\n");


	//>was sind wir?
	/**********************************************************************/
	if( (id->vendor == 0x1204) &&  (id->device == 0x0200) )		//Lattice (AGE-X1)
	{
		tempDevSubType = SubType_AGEX;
		pr_devel(MODDEBUGOUTTEXT" found AGE-X device\n");
	}
	else if((id->vendor == 0x1172) &&  (id->device == 0x0004) )	//Altera (AGE-X2)
	{
		tempDevSubType = SubType_AGEX2;
		pr_devel(MODDEBUGOUTTEXT" found AGE-X2 device\n");
	}
	else if((id->vendor == 0x1172) &&  (id->device == 0xA6E4) )	//Altera (MVC0)
	{
		tempDevSubType = SubType_MVC0;
		pr_devel(MODDEBUGOUTTEXT" found MVC0 device\n");
	}
	else
	{
		printk(KERN_WARNING MODDEBUGOUTTEXT" unknowen device identifier (ven: 0x%x,  dev: 0x%x)\n", id->vendor, id->device);
		return -EINVAL;
	}


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


	//>BAR0
	/**********************************************************************/
	//das bar0 prüfen
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
		_ModuleData.Devs[DevIndex].pVABAR0 = ioremap(bar0_start,bar0_len); //das setzen der adr zeigt auch an das wir (R/W) fns aufs device schreiben dürfen
		if(_ModuleData.Devs[DevIndex].pVABAR0 == NULL)
			printk(KERN_ERR MODDEBUGOUTTEXT" ioremap failed!\n");
		else
			pr_devel(MODDEBUGOUTTEXT" map bar0> 0x%llx to 0x%p\n",bar0_start,_ModuleData.Devs[DevIndex].pVABAR0);
	}
	_ModuleData.Devs[DevIndex].BAR0SizeBytes = bar0_len;


	//>DMA(Coherent) Buffer (nicht AGEX1)
	/**********************************************************************/
	if( (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_AGEX2) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_AGEX2_CL) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_MVC0))
	{
		u8 MaxDAMAddressSize = 32;
		if(_ModuleData.Devs[DevIndex].DeviceSubType==SubType_AGEX2_CL)
			MaxDAMAddressSize = 64;

		//sagt das wir xxBit können
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


		//gibt speicher zurück ohne/mit cache off und auf Page ausgerichtet
		// https://www.kernel.org/doc/Documentation/DMA-API.txt
		// "... Consistent memory is memory for which a write by either the device or
		//	the processor can immediately be read by the processor or device
		//	without having to worry about caching effects..."
		_ModuleData.Devs[DevIndex].pVACommonBuffer = dma_alloc_coherent(	&pcidev->dev,	/* für welches device */
																			PAGE_SIZE,		/* größe in Bytes (wird eh min zu einer Page)*/
																			&_ModuleData.Devs[DevIndex].pBACommonBuffer,
																			GFP_KERNEL);	/* zone wird über die maske eingestellt, sonst nur noch ob GFP_ATOMIC)*/
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

	
	//>IRQ & tasklet
	/**********************************************************************/
	//tasklet
	tasklet_init(&_ModuleData.Devs[DevIndex].IRQTasklet, AGEXDrv_tasklet, DevIndex);

	//msi einschalten
	//http://www.mjmwired.net/kernel/Documentation/MSI-HOWTO.txt
	// "... to call this API before calling request_irq()..."
	if( (_ModuleData.Devs[DevIndex].DeviceSubType == SubType_AGEX2) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_AGEX2_CL) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_MVC0) ) 
	{
		if( pci_enable_msi(pcidev) != 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT"pci_enable_msi failed!\n"); return -EIO;}
	}

	//das gibt die (un)gemapped nummer zurück lspci zeigt die mapped an!
	//if(pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &PCIIRQ_Number) ) {
	if( request_irq(pcidev->irq,/* die IRQ nummer */
			AGEXDrv_interrupt,	/* die IRQ fn */
			IRQF_SHARED,		/* shared */
			MODMODULENAME,		/* name wird in /proc/interrupts angezeigt */
			&_ModuleData.Devs[DevIndex]	/* unique identifier/ wird auch dem CallBack mit gegeben */
			) != 0)
	{
		printk(KERN_ERR MODDEBUGOUTTEXT" request_irq failed\n");
		if( (_ModuleData.Devs[DevIndex].DeviceSubType == SubType_AGEX2) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_AGEX2_CL) || (_ModuleData.Devs[DevIndex].DeviceSubType==SubType_MVC0) )
			pci_disable_msi(pcidev);

		_ModuleData.Devs[DevIndex].boIsIRQOpen = FALSE;
	}
	else
	{
		AGEXDrv_SwitchInterruptOn(&_ModuleData.Devs[DevIndex], TRUE);

		pr_devel(MODDEBUGOUTTEXT" IRQ> %d \n",pcidev->irq);
		_ModuleData.Devs[DevIndex].boIsIRQOpen = TRUE;
	}


	//>dev init & fügt das es hinzu
	/**********************************************************************/
	cdev_init(&_ModuleData.Devs[DevIndex].DeviceCDev, &AGEXDrv_fops);
	_ModuleData.Devs[DevIndex].DeviceCDev.owner = THIS_MODULE;
	_ModuleData.Devs[DevIndex].DeviceCDev.ops 	= &AGEXDrv_fops;	//notwendig in den quellen wird fops gesetzt?

	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
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

	//IRQ zuückgeben
	if(pDevData->boIsIRQOpen)
	{
		AGEXDrv_SwitchInterruptOn(pDevData, FALSE);
		free_irq(pcidev->irq, pDevData);
		if( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_AGEX2_CL) || (pDevData->DeviceSubType==SubType_MVC0) )
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

	if( 	( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_AGEX2_CL) || (pDevData->DeviceSubType==SubType_MVC0) )
		&& 	(pDevData->pVACommonBuffer != NULL) )
		dma_free_coherent(&pcidev->dev, PAGE_SIZE, pDevData->pVACommonBuffer, pDevData->pBACommonBuffer);
	pDevData->pVACommonBuffer = NULL;

	//das pci_dev nicht mehr für PCI nutzen (bzw. setzt Bits im PCIConfigMem)??
	pci_disable_device(pcidev);

	//device in der sysfs class löschen
	if(!IS_ERR(_ModuleData.pModuleClass))	
		device_destroy(_ModuleData.pModuleClass, pDevData->DeviceNumber);

	//device löschen
	if(pDevData->boIsDeviceOpen)
		cdev_del(&pDevData->DeviceCDev);
	pDevData->boIsDeviceOpen = FALSE;

}


