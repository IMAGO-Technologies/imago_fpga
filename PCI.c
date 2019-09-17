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
//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterstützt, 0 ja <0 nein
int AGEXDrv_PCI_probe(struct pci_dev *pcidev, const struct pci_device_id *id)
{
	u64 bar0_start,bar0_len;
	int res;
	u8 tempDevSubType;
	PDEVICE_DATA pDevData = NULL;
	int minor = -1;

	pci_set_drvdata(pcidev, NULL);	

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_PCI_probe\n");

	tempDevSubType = id->driver_data;
	if (tempDevSubType == SubType_Invalid || tempDevSubType > ARRAY_SIZE(AGEXDrv_device_info)) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" unknown device identifier (%u)\n", tempDevSubType);
		return -EINVAL;
	}

	pr_devel(MODDEBUGOUTTEXT" found '%s' device\n", AGEXDrv_device_info[tempDevSubType].name);


	//>freie Minor Nummer?
	/**********************************************************************/
	for (minor=0; minor<MAX_DEVICE_COUNT; minor++) {
		if (!_ModuleData.boIsMinorUsed[minor]) {
			pDevData = &_ModuleData.Devs[minor];
			AGEXDrv_InitDrvData(pDevData);
			pDevData->DeviceSubType= tempDevSubType;
			pDevData->DeviceNumber = MKDEV(MAJOR(_ModuleData.FirstDeviceNumber), minor);
			pDevData->dev = &pcidev->dev;
			pDevData->flags =  AGEXDrv_device_info[tempDevSubType].flags;
			pci_set_drvdata(pcidev, pDevData);				//damit wir im AGEXDrv_PCI_remove() wissen welches def freigebene werden soll
			break;
		}
	}
	if (pDevData == NULL) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" no free Minor-Number found!\n");
		return -EINVAL;
	}

	pr_devel(MODDEBUGOUTTEXT" use major/minor (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	//>PCI device on(setzt Bits im PCI Config Mem)
	/**********************************************************************/
	if (pci_enable_device(pcidev) < 0) {
		printk(KERN_ERR MODDEBUGOUTTEXT" pci_enable_device failed\n"); return -EIO;}

	//Enable PCI Bus Master
	pci_set_master(pcidev);
	

	//>BAR0
	/**********************************************************************/
	//das bar0 prüfen
	bar0_start = pci_resource_start(pcidev, 0);
	bar0_len = pci_resource_len(pcidev, 0);
	if (!(pci_resource_flags(pcidev, 0) & IORESOURCE_MEM)) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" invalid bar0\n");
		return -ENODEV;
	}
	else
		pr_devel(MODDEBUGOUTTEXT" bar0> 0x%llx, %llu [Bytes]\n", bar0_start, bar0_len);

	//bar0 mappen request_region(); request_mem_region() macht kein mapping sondern 'nur' eine 'reservation'
	if (request_mem_region(bar0_start, bar0_len,MODMODULENAME) == NULL) {
		pDevData->boIsBAR0Requested = FALSE;
		printk(KERN_ERR MODDEBUGOUTTEXT" request_mem_region failed!\n");
		return -EBUSY;
	}
	else {
		pDevData->boIsBAR0Requested = TRUE;
		pDevData->pVABAR0 = ioremap(bar0_start,bar0_len); //das setzen der adr zeigt auch an das wir (R/W) fns aufs device schreiben dürfen
		if (pDevData->pVABAR0 == NULL)
			printk(KERN_ERR MODDEBUGOUTTEXT" ioremap failed!\n");
		else
			pr_devel(MODDEBUGOUTTEXT" map bar0> 0x%llx to 0x%p\n", bar0_start, pDevData->pVABAR0);
	}


	//>DMA(Coherent) Buffer (nicht AGEX1)
	/**********************************************************************/
	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		u8 MaxDAMAddressSize = 32;
		if (IS_TYPEWITH_PCI64BIT(pDevData))
			MaxDAMAddressSize = 64;

		//sagt das wir xxBit können
		//https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
		// "...By default, the kernel assumes that your device can address the full 32-bits...
		//  ... It is good style to do this even if your device holds the default setting ..."
		if (dma_set_mask(&pcidev->dev, DMA_BIT_MASK(MaxDAMAddressSize) ) != 0) {
			printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_mask failed!\n");
			return -EIO;
		}
		//ab 2.6.34
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
		if (dma_set_coherent_mask(&pcidev->dev, DMA_BIT_MASK(MaxDAMAddressSize) ) != 0) {
			printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_coherent_mask failed!\n");
			return -EIO;
		}
#endif


		//gibt speicher zurück ohne/mit cache off und auf Page ausgerichtet
		// https://www.kernel.org/doc/Documentation/DMA-API.txt
		// "... Consistent memory is memory for which a write by either the device or
		//	the processor can immediately be read by the processor or device
		//	without having to worry about caching effects..."
		pDevData->pVACommonBuffer = dma_alloc_coherent(	&pcidev->dev,	/* für welches device */
														PAGE_SIZE,		/* größe in Bytes (wird eh min zu einer Page)*/
														&pDevData->pBACommonBuffer,
														GFP_KERNEL);	/* zone wird über die maske eingestellt, sonst nur noch ob GFP_ATOMIC)*/
		if (pDevData->pVACommonBuffer == 0) {
			printk(KERN_ERR MODDEBUGOUTTEXT" dma_alloc_coherent failed!\n");
			return -ENOMEM;
		}

		if (sizeof(dma_addr_t) == sizeof(u32))
			pr_devel(MODDEBUGOUTTEXT" DMABuffer> VA: 0x%p, BA: 0x%x, %ld [Bytes]\n", 
				pDevData->pVACommonBuffer, (u32)pDevData->pBACommonBuffer, PAGE_SIZE);
		else
			pr_devel(MODDEBUGOUTTEXT" DMABuffer> VA: 0x%p, BA: 0x%llx, %ld [Bytes]\n",
				pDevData->pVACommonBuffer, (u64)pDevData->pBACommonBuffer, PAGE_SIZE);
		memset(pDevData->pVACommonBuffer, 0 ,PAGE_SIZE);	//es gibt ab 3.2 dma_zalloc_coherent()
	}


	//>DMA2Host Buffer (also CL, VCXM ...)
	/**********************************************************************/
	if (IS_TYPEWITH_DMA2HOST(pDevData)) {
		//damit z.b dma_map_sg() (mit einer IOMMU) nicht zuviel zusammengefasst
		// aber >nicht< für sg_alloc_table_from_pages()!
		if( dma_set_max_seg_size(&pcidev->dev, DMA_READ_TC_SG_MAX_BYTECOUNT) != 0 )		
			{printk(KERN_ERR MODDEBUGOUTTEXT" dma_set_max_seg_size failed!\n"); return -EIO;}
	}

	
	//>IRQ & tasklet
	/**********************************************************************/
	//tasklet
	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) 
		tasklet_init(&pDevData->IRQTasklet, AGEXDrv_tasklet_PCIe, (unsigned long)pDevData);
	else
		tasklet_init(&pDevData->IRQTasklet, AGEXDrv_tasklet_PCI, (unsigned long)pDevData);

	//msi einschalten
	//http://www.mjmwired.net/kernel/Documentation/MSI-HOWTO.txt
	// "... to call this API before calling request_irq()..."
	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		if( pci_enable_msi(pcidev) != 0)
			{printk(KERN_ERR MODDEBUGOUTTEXT"pci_enable_msi failed!\n"); return -EIO;}
	}

	//das gibt die (un)gemapped nummer zurück lspci zeigt die mapped an!
	//if(pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &PCIIRQ_Number) ) {
	if (request_irq(pcidev->irq,/* die IRQ nummer */
			AGEXDrv_interrupt,	/* die IRQ fn */
			IRQF_SHARED,		/* shared */
			MODMODULENAME,		/* name wird in /proc/interrupts angezeigt */
			pDevData			/* unique identifier/ wird auch dem CallBack mit gegeben */
			) != 0) {
		printk(KERN_ERR MODDEBUGOUTTEXT" request_irq failed\n");
		if (IS_TYPEWITH_COMMONBUFFER(pDevData))
			pci_disable_msi(pcidev);

		pDevData->boIsIRQOpen = FALSE;
		return -EIO;
	}

	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		// Adresse fuer Common Buffer im FPGA setzen
		// low 32 bit
		iowrite32((pDevData->pBACommonBuffer & 0xFFFFFFFF), pDevData->pVABAR0 + ISR_COMMONBUFFER_ADR_AGEX2);

#ifdef CONFIG_64BIT		
		// high 32 bit
		if (IS_TYPEWITH_PCI64BIT(pDevData))
			iowrite32((pDevData->pBACommonBuffer >> 32), pDevData->pVABAR0 + ISR_COMMONBUFFER_ADR_AGEX2 + 4);
#endif		
	}
	
	// Interrupts im FPGA einschalten
	AGEXDrv_SwitchInterruptOn(pDevData, TRUE);

	pr_devel(MODDEBUGOUTTEXT" IRQ> %d \n",pcidev->irq);
	pDevData->boIsIRQOpen = TRUE;


	//>dev init & fügt das es hinzu
	/**********************************************************************/
	cdev_init(&pDevData->DeviceCDev, &AGEXDrv_fops);
	pDevData->DeviceCDev.owner = THIS_MODULE;
	pDevData->DeviceCDev.ops 	= &AGEXDrv_fops;	//notwendig in den quellen wird fops gesetzt?

	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
	res = cdev_add(&pDevData->DeviceCDev, pDevData->DeviceNumber, 1/*wie viele ab startNum*/);
	if (res < 0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" can't add device!\n");
	else
		pDevData->boIsDeviceOpen = TRUE;


	//> in Sysfs class eintragen
	/**********************************************************************/			
	//war mal class_device_create
	if (!IS_ERR(_ModuleData.pModuleClass)) {		
		char devName[128];
		struct device *temp;

		sprintf(devName, "%s%d", MODMODULENAME, MINOR(pDevData->DeviceNumber));
		temp = device_create(
				_ModuleData.pModuleClass, 	/* die Type classe */
				NULL, 			/* pointer zum Eltern, dann wird das dev ein Kind vom parten*/
				pDevData->DeviceNumber, /* die nummer zum device */
				NULL,
				devName			/*string for the device's name */
				);

		if (IS_ERR(temp))
			printk(KERN_WARNING MODDEBUGOUTTEXT" can't create sysfs device!\n");
	}


	// init von allem ist durch
	dev_info(pDevData->dev, "probe done (0x%04x:0x%04x <> %d:%d)\n",
		id->vendor, id->device, MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));
	_ModuleData.boIsMinorUsed[minor] = TRUE;

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

	if (pDevData == NULL) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" device pointer is zero!\n"); return;}

	//DMA (versuchen) aufzuräumen
	// - devs die keine DMA haben, da ist DMARead_anzXXX=0 
	for (i = 0; i < pDevData->DMARead_channels; i++) {
		AGEXDrv_DMARead_Abort_DMAChannel(pDevData, i);
		AGEXDrv_DMARead_Abort_DMAWaiter(pDevData, i);
	}

	//IRQ zuückgeben
	if(pDevData->boIsIRQOpen) {
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

