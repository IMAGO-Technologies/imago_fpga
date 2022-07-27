/*
 * PCI(e) code
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
 */

#include "imago_fpga.h"
#include <linux/irq.h>
#include <linux/pci.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
	#include <uapi/linux/sched/types.h>	/* For struct sched_param */
#endif


// register offset for interrupt enable
#define ISR_ONOFF_OFFSET_AGEX (1<<20)
#define ISR_ONOFF_OFFSET_AGEX2 (0x10010)
// register offset for setting the address of interrupt data
#define ISR_COMMONBUFFER_ADR_AGEX2 (0x10000)
// register offset for reading interrupt flag and FIFO level
#define ISR_AVAILABLE_OFFSET (1<<20)

//2xDWORD, 		IRQFlags
//2xDWORD, 		IRQPacket, Header0/Header1
//128xDWORD, 	IRQPacket, data
//DMAs*TCsx8,	DMABuffer counters (UINT16)
#define HOST_BUFFER_SIZE ((4*(2+2+128))+(MAX_DMA_CHANNELS * MAX_DMA_READ_CHANNELTCS*8)) 


//für welche PCI IDs sind wir zuständi?
static struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(0x1204/*VendorID (Lattice Semi)*/, 0x0200 /*DeviceID*/),	/* AGE-X1 */
		.driver_data = DeviceType_AGEX },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0004 /*DeviceID*/),			/* AGE-X2 */
		.driver_data = DeviceType_AGEX2 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xA6E4 /*DeviceID*/),			/* MVC0 */
		.driver_data = DeviceType_MVC0 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0010 /*DeviceID*/),			/* AGE-X2-CL */
		.driver_data = DeviceType_AGEX2_CL },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0x0005 /*DeviceID*/),			/* VCXM */
		.driver_data = DeviceType_VCXM },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xCA72 /*DeviceID*/),			/* LeMans */
		.driver_data = DeviceType_LEMANS },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xDECA /*DeviceID*/),			/* PCIe-CL */
		.driver_data = DeviceType_PCIE_CL },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xA6E5 /*DeviceID*/),			/* AGE-X5 */
		.driver_data = DeviceType_AGEX5 },
	{ PCI_DEVICE(0x1172/*VendorID (Altera)*/, 0xDCA5 /*DeviceID*/),			/* AGE-X5-CL */
		.driver_data = DeviceType_AGEX5_CL },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);		//macht dem kernel bekannt was dieses modul für PCI devs kann


// writes FPGA packet
static long fpga_write(struct _DEVICE_DATA *pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	u8 TempBuffer[4*(3+1)];		// +1 damit bei PCIe immer 64 Bit sind
	u8 deviceID;

	// User Data -> Kernel
	if (BytesToWrite > sizeof(TempBuffer)) {
		dev_warn(pDevData->dev, "fpga_write(): too many bytes\n");
		return -EFBIG;
	}
	if (copy_from_user (TempBuffer, pToUserMem, BytesToWrite) != 0) {
		dev_warn(pDevData->dev, "fpga_write(): copy_from_user() failed\n");
		return -EFAULT;
	}

	// insert serialID to Header1:
	deviceID = (((u32*)TempBuffer)[1] >> 20) & (MAX_IRQDEVICECOUNT - 1);
	if (deviceID != 0) {
		((u32*)TempBuffer)[1] |= pDevData->SunDeviceData[deviceID].serialID << 26;
	}

	/* Kernel -> PCI */
	//Notes:
	// -für die AGEX muss sich die Adr nicht ändern
	// -bei der AGEX2 müssen es 32Bit mit steigender Adr sein
	//
	// aus include/asm-generic/iomap.h für ioread/writeX_rep
	// "...They do _not_ update the port address. If you
	//	want MMIO that copies stuff laid out in MMIO
	//	memory across multiple ports, use "memcpy_toio()..."
	//
	// aber auch memcpy_toio() macht nicht immer 32Bit
	// "http://www.gossamer-threads.com/lists/linux/kernel/650995?do=post_view_threaded#650995"
	if (BytesToWrite % 4) {
		u32 ByteIndex =0;
		for(; ByteIndex < BytesToWrite; ByteIndex++)
			iowrite8(TempBuffer[ByteIndex], pDevData->pVABAR0+ByteIndex);
	}
	else {
		u32 WordsToCopy = BytesToWrite/4;
		u32 WordIndex;
		for (WordIndex = 0; WordIndex < WordsToCopy; WordIndex++)
			iowrite32(((u32*)TempBuffer)[WordIndex], pDevData->pVABAR0 + WordIndex*4 );
	}

	return BytesToWrite;
}


//Schaltet im PCI-Geraet die Ints ab/zu
static void pci_enable_interrupt(PDEVICE_DATA pDevData, bool enable)
{
	if (pDevData == NULL || pDevData->device_type == DeviceType_Invalid || pDevData->pVABAR0 == NULL)
		return;

	//der DPC ist/ muss durch sein
	//(kann aber erst hier gesetzt werden da sonst race cond mit CommonBuffer Clear, IRQEnable und DPCFlag)
	if (enable) {

		if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
			//nur um ganz sicher zu sein
			if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
				return;

			//> das IRQ Flag löschen (2 Word [Flags])
			//ACHTUNG! die 3 Word [Header0/Header1/Data[0]] nicht überschreiben da alte VCXM/CL-PCIe FPGAs während ein DMA IRQ behandelt wurde
			//	schon das SUNPaket in den CommonBuffer geschrieben haben, 
			//	nach dem setzen des "IRQ-Enable Bits" im FPGA hat dieser dann das FLAG gesetzt und den MSI geschickt
			memset(pDevData->pVACommonBuffer, 0, 2 * sizeof(u32));
			smp_mb();	//nop bei x86
		}
	}

	// dev_dbg(pDevData->dev, "pci_enable_interrupt: %u\n", enable ? 1 : 0);

	/* IRQ enable flag */
	iowrite32(enable ? 1 : 0, pDevData->pVABAR0 + (IS_TYPEWITH_COMMONBUFFER(pDevData) ? ISR_ONOFF_OFFSET_AGEX2 : ISR_ONOFF_OFFSET_AGEX));
}

// Interrupts

// PCI interrupt
static irqreturn_t pci_interrupt(int irq, void *dev_id)
{
	u32 regVal;
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;

	BUG_ON(pDevData == NULL || pDevData->pVABAR0 == NULL || pDevData->device_type == DeviceType_Invalid);

	// check the FPGA interrupt flag (cleared whein FIFO is empty or when interrupt is disabled)
	regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);
	if ((regVal & 0x1) == 0)
		return IRQ_NONE;	// not our interrupt

	// disable interrupt in FPGA
	pci_enable_interrupt(pDevData, false);

	// wake up thread
	return IRQ_WAKE_THREAD;
}

// PCIe interrupt
static irqreturn_t pcie_interrupt(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	irqreturn_t result = pDevData->irqEnableInHWI ? IRQ_HANDLED : IRQ_WAKE_THREAD;

	dev_dbg(pDevData->dev, "pcie_interrupt: IRQReg_A = 0x%08x\n", IRQReg_A);

	// check for DMA done flag
	if ((IRQReg_A >> 4) != 0) {
		if (pDevData->setupTcInHWI)
			imago_DMARead_DPC(pDevData);
		else
			result = IRQ_WAKE_THREAD;
	}

	// check for SUN interrupt flag
	if ((IRQReg_A & 0x1) != 0) {
		// get SUN Header0/1
		sun_packet[0] = ((u32*)pDevData->pVACommonBuffer)[2+0];
		sun_packet[1] = ((u32*)pDevData->pVACommonBuffer)[2+1];
		sun_packet[2] = ((u32*)pDevData->pVACommonBuffer)[2+2];

		// process SUN packet
		imago_sun_interrupt(pDevData, sun_packet);
	}

	if (result == IRQ_HANDLED)
		pci_enable_interrupt(pDevData, true);

	return result;
}


// IRQ threads

// PCI
static irqreturn_t pci_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			regVal, fifoLevel;

	BUG_ON(pDevData->device_type == DeviceType_Invalid);
	BUG_ON(pDevData->pVABAR0 == NULL);

	// AGEX hat ein FIFO => FIFO-Fuellstand einlesen
	regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);

	// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
	fifoLevel = (regVal >> 1) & 0x1FF;
	if (fifoLevel < 3) {
		// sollte nicht auftreten
		dev_err(pDevData->dev, "pci_thread() > SUN Error: FIFO level to small: %d\n", fifoLevel);
		return IRQ_HANDLED;
	}

	// SUN Header0/1 einlesen
	sun_packet[0] = ioread32(pDevData->pVABAR0);	// Header0
	sun_packet[1] = ioread32(pDevData->pVABAR0);	// Header1
	sun_packet[2] = ioread32(pDevData->pVABAR0);	// Payload

	// process SUN packet
	imago_sun_interrupt(pDevData, sun_packet);

	/* Re-enable the interrupt */
	/**********************************************************************/
	pci_enable_interrupt(pDevData, true);

	return IRQ_HANDLED;
}

// PCIe: only process DMA interrupts, SUN packets are already handled by the pcie_interrupt
static irqreturn_t pcie_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];

	if ((IRQReg_A >> 4) != 0 && !pDevData->setupTcInHWI)
		imago_DMARead_DPC(pDevData);

	pci_enable_interrupt(pDevData, true);

	return IRQ_HANDLED;
}


//<====================================>
//	PCI fns
//		wegen Hot-Plug gibt es CallBacks
//<====================================>
//wird aufgerufen wenn der kernel denkt das der treiber das PCIDev unterstützt, 0 ja <0 nein
static int imago_pci_probe(struct pci_dev *pcidev, const struct pci_device_id *id)
{
	u64 bar0_start,bar0_len;
	int i, res;
	u8 dev_type;
	PDEVICE_DATA pDevData = NULL;
	struct irq_desc *desc;

	pci_set_drvdata(pcidev, NULL);	

	dev_dbg(&pcidev->dev, "imago_pci_probe\n");

	dev_type = id->driver_data;
	if (dev_type == DeviceType_Invalid) {
		dev_warn(&pcidev->dev, "invalid device identifier (%u)\n", dev_type);
		return -EINVAL;
	}


	pDevData = imago_alloc_dev_data(&pcidev->dev, dev_type);
	if (pDevData == NULL)
		return -EINVAL;

	pDevData->write = fpga_write;
	pci_set_drvdata(pcidev, pDevData);				//damit wir im imago_pci_remove() wissen welches def freigebene werden soll

	dev_dbg(&pcidev->dev, "using major/minor (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	//>PCI device on(setzt Bits im PCI Config Mem)
	/**********************************************************************/
	if (pci_enable_device(pcidev) < 0) {
		dev_err(&pcidev->dev, "pci_enable_device failed\n");
		imago_free_dev_data(pDevData);
		return -EIO;
	}

	//Enable PCI Bus Master
	pci_set_master(pcidev);
	

	//>BAR0
	/**********************************************************************/
	//das bar0 prüfen
	bar0_start = pci_resource_start(pcidev, 0);
	bar0_len = pci_resource_len(pcidev, 0);
	if (!(pci_resource_flags(pcidev, 0) & IORESOURCE_MEM)) {
		dev_warn(&pcidev->dev, "invalid bar0\n");
		imago_free_dev_data(pDevData);
		return -ENODEV;
	}

	dev_dbg(&pcidev->dev, "bar0: 0x%llx, %llu bytes\n", bar0_start, bar0_len);

	//bar0 mappen request_region(); request_mem_region() macht kein mapping sondern 'nur' eine 'reservation'
	if (request_mem_region(bar0_start, bar0_len, MODMODULENAME) == NULL) {
		dev_err(&pcidev->dev, "request_mem_region failed\n");
		imago_free_dev_data(pDevData);
		return -EBUSY;
	}
	else {
		pDevData->boIsBAR0Requested = true;
		pDevData->pVABAR0 = ioremap(bar0_start,bar0_len); //das setzen der adr zeigt auch an das wir (R/W) fns aufs device schreiben dürfen
		if (pDevData->pVABAR0 == NULL)
			dev_err(&pcidev->dev, "ioremap failed\n");
		else
			dev_dbg(&pcidev->dev, "map bar0 0x%llx to 0x%p\n", bar0_start, pDevData->pVABAR0);
	}


	// allocate coherent DMA buffer for storing interrupt data by the FPGA
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
			dev_err(&pcidev->dev, "dma_set_mask failed!\n");
			imago_free_dev_data(pDevData);
			return -EIO;
		}
		//ab 2.6.34
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
		if (dma_set_coherent_mask(&pcidev->dev, DMA_BIT_MASK(MaxDAMAddressSize) ) != 0) {
			dev_err(&pcidev->dev, "dma_set_coherent_mask failed!\n");
			imago_free_dev_data(pDevData);
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
			dev_err(&pcidev->dev, "dma_alloc_coherent failed!\n");
			imago_free_dev_data(pDevData);
			return -ENOMEM;
		}
		dev_dbg(&pcidev->dev, "DMABuffer> VA: 0x%p, BA: 0x%llx, %ld bytes\n",
			pDevData->pVACommonBuffer, (u64)pDevData->pBACommonBuffer, PAGE_SIZE);
		memset(pDevData->pVACommonBuffer, 0 ,PAGE_SIZE);	//es gibt ab 3.2 dma_zalloc_coherent()
	}


	// DMA
	/**********************************************************************/
	if (IS_TYPEWITH_DMA2HOST(pDevData)) {
		for (i = 0; i < MAX_DMA_CHANNELS; i++) {
			PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[i];

			// allocate job storage and KFIFO queues
			pDMAChannel->jobBuffers = kzalloc(_ModuleData.max_dma_buffers * sizeof(DMA_READ_JOB), GFP_KERNEL);
			if (pDMAChannel->jobBuffers == NULL) {
				dev_err(pDevData->dev, "kzalloc() failed\n");
				imago_free_dev_data(pDevData);
				return -ENOMEM;
			}
			res = kfifo_alloc(&pDMAChannel->Jobs_ToDo, _ModuleData.max_dma_buffers * sizeof(DMA_READ_JOB *), GFP_KERNEL);
			if (res) {
				dev_err(pDevData->dev, "kfifo_alloc() failed\n");
				imago_free_dev_data(pDevData);
				return res;
			}
			res = kfifo_alloc(&pDMAChannel->Jobs_Done, _ModuleData.max_dma_buffers * sizeof(DMA_READ_JOB *), GFP_KERNEL);
			if (res) {
				dev_err(pDevData->dev, "kfifo_alloc() failed\n");
				imago_free_dev_data(pDevData);
				return res;
			}
		}

		//damit z.b dma_map_sg() (mit einer IOMMU) nicht zuviel zusammengefasst
		// aber >nicht< für sg_alloc_table_from_pages()!
		if (dma_set_max_seg_size(&pcidev->dev, DMA_READ_TC_SG_MAX_BYTECOUNT) != 0) {
			dev_err(&pcidev->dev, "dma_set_max_seg_size failed!\n");
			imago_free_dev_data(pDevData);
			return -EIO;
		}
	}

	
	// setup interrupt
	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		// PCIe interface
		// enable MSI
		// http://www.mjmwired.net/kernel/Documentation/MSI-HOWTO.txt
		// "... to call this API before calling request_irq()..."
		if (pci_enable_msi(pcidev) != 0) {
			dev_err(&pcidev->dev, "pci_enable_msi failed\n");
			imago_free_dev_data(pDevData);
			return -EIO;
		}

		// PREEMP_RT: use IRQF_NO_THREAD to avoid force-threaded interrupt of the primary handler
		// which would disable bh/preempt and therefore compete with other force-threaded interrupts
		if (request_threaded_irq(pcidev->irq, pcie_interrupt, pcie_thread,
					IRQF_TRIGGER_RISING | IRQF_NO_THREAD, MODMODULENAME, pDevData) != 0) {
			dev_err(&pcidev->dev, "request_threaded_irq failed\n");
			imago_free_dev_data(pDevData);
			return -EIO;
		}
	}
	else {
		// PCI interface
		if (request_threaded_irq(pcidev->irq, pci_interrupt, pci_thread,
					IRQF_SHARED, MODMODULENAME, pDevData) != 0) {
			dev_err(&pcidev->dev, "request_threaded_irq failed\n");
			imago_free_dev_data(pDevData);
			return -EIO;
		}
	}

	// increase priority of threaded interrupt
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,9,0)
	{
		struct sched_param sched_par;
		sched_par.sched_priority = 86;
		desc = irq_to_desc(pcidev->irq);
		sched_setscheduler(desc->action->thread, SCHED_FIFO, &sched_par);
	}
#else
	desc = irq_data_to_desc(irq_get_irq_data(pcidev->irq));
	sched_set_fifo(desc->action->thread);
#endif

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
	
	dev_dbg(&pcidev->dev, "using IRQ %d\n", pcidev->irq);

	// create char device
	res = imago_create_device(pDevData);
	if (res < 0) {
		imago_free_dev_data(pDevData);
		return res;
	}

	// enable PCI interrupts
	pci_enable_interrupt(pDevData, true);

	dev_info(pDevData->dev, "probe done (0x%04x:0x%04x <> %d:%d)\n",
		id->vendor, id->device, MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	return 0;
}


static void imago_pci_remove(struct pci_dev *pcidev)
{
	u32 i;
	PDEVICE_DATA pDevData = (PDEVICE_DATA)pci_get_drvdata(pcidev);

	if (pDevData == NULL) {
		dev_warn(&pcidev->dev, "imago_pci_remove(): device data is invalid\n");
		return;
	}

	dev_dbg(&pcidev->dev, "imago_pci_remove (%d:%d)\n", MAJOR(pDevData->DeviceNumber), MINOR(pDevData->DeviceNumber));

	if (IS_TYPEWITH_DMA2HOST(pDevData)) {
		// Stop DMA transfers and unmap all buffers
		for (i = 0; i < pDevData->DMARead_channels; i++) {
			imago_DMARead_Reset_DMAChannel(pDevData, i);
		}

		for (i = 0; i < MAX_DMA_CHANNELS; i++) {
			PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channel[i];
			kfifo_free(&pDMAChannel->Jobs_ToDo);
			kfifo_free(&pDMAChannel->Jobs_Done);
			kfree(pDMAChannel->jobBuffers);
		}
	}

	pci_enable_interrupt(pDevData, false);
	free_irq(pcidev->irq, pDevData);
	if (IS_TYPEWITH_COMMONBUFFER(pDevData))
		pci_disable_msi(pcidev);

	if(pDevData->pVABAR0 != NULL)
		iounmap(pDevData->pVABAR0);
	pDevData->pVABAR0 = NULL;

	if(pDevData->boIsBAR0Requested)
		release_mem_region( pci_resource_start(pcidev, 0), pci_resource_len(pcidev, 0) );
	pDevData->boIsBAR0Requested = false;

	if( 	( IS_TYPEWITH_COMMONBUFFER(pDevData) )
		&& 	(pDevData->pVACommonBuffer != NULL) )
		dma_free_coherent(&pcidev->dev, PAGE_SIZE, pDevData->pVACommonBuffer, pDevData->pBACommonBuffer);
	pDevData->pVACommonBuffer = NULL;

	pci_disable_device(pcidev);

	imago_dev_close(pDevData);
}

struct pci_driver imago_pci_driver = {
	.name = "imago-fpga-pci",
	.id_table = pci_ids,
	.probe = imago_pci_probe,
	.remove = imago_pci_remove,
};
