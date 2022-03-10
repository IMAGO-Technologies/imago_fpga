/*
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.*
 */

#ifndef AGEXDRV_H_
#define AGEXDRV_H_

// module definitions
/******************************************************************************************/
#define MODVERSION "1.1.16.0"
#define MODDATECODE __DATE__ " - " __TIME__
#define MODLICENSE "GPL"
#define MODDESCRIPTION "IMAGO FPGA / RTCC device driver"
#define MODAUTHOR "IMAGO Technologies GmbH"
#define MODMODULENAME	"imago_fpga"


#include <linux/init.h>		// for module_init(),
#include <linux/module.h>	// for MODULE_LICENSE
#include <linux/version.h>	// for die Version

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	#include <linux/printk.h>	// for printk
#endif

#include <linux/types.h>	// for dev_t
#include <asm/types.h>		// for u8, s8
#include <linux/slab.h>		// for kmalloc ...
#include <linux/sched.h>	// for current (pointer to the current process)
#include <linux/fs_struct.h>
#include <linux/kdev_t.h>	// for MAJOR/MINOR
#include <linux/cdev.h>		// for cdev_*
#include <linux/device.h>	// for class_create
#include <linux/fs.h>		// for alloc_chrdev_region /file_*
#include <linux/semaphore.h>// for up/down ...
#include <linux/completion.h>
#include <linux/kfifo.h>	// for kfifo_*
#include <linux/of_device.h>	// for of*
#include <linux/interrupt.h>// for IRQ*
#include <linux/dma-mapping.h>	// for dma_*
#include <linux/scatterlist.h>	// sg_* ...
#include <linux/spinlock.h>	// spin_* ...
#include <linux/delay.h>	// for usleep_range
#include <linux/uaccess.h>	// for copy_to_user


/*** defines ***/
/******************************************************************************************/

// Hardware type definitions
enum IMAGO_DEVICE_TYPE
{
	DeviceType_Invalid 	= 0,
	DeviceType_AGEX		= 1,
	DeviceType_AGEX2	= 2,
	DeviceType_MVC0		= 3,
	DeviceType_AGEX2_CL	= 4,
	DeviceType_VCXM		= 5,
	DeviceType_LEMANS	= 6,
   	DeviceType_PCIE_CL	= 7,
	DeviceType_AGEX5	= 8,
	DeviceType_AGEX5_CL	= 9,
	DeviceType_DAYTONA	= 10,
	DeviceType_VSPV3	= 11
};

// Device flags used by struct _DEVICE_DATA
#define IMAGO_DEV_FLAG_PCIE			0x01
#define IMAGO_DEV_FLAG_PCI			0x02
#define IMAGO_DEV_FLAG_PCI64BIT		0x04
#define IMAGO_DEV_FLAG_DMA2HOST		0x08
#define IMAGO_DEV_FLAG_SPI			0x10


//> Ioctl definitions, see "ioctl-number.txt"
/******************************************************************************************/
// magic number
#define IMAGO_IOC_MAGIC  '['


/******************************************************************************************/

// maximum number of supported devices (FPGAs)
#define MAX_DEVICE_COUNT 32

// number of SUN device IDs for each FPGA
#define MAX_IRQDEVICECOUNT 64
#define MAX_SUNPACKETSIZE  (4*3) // SUN packet size, 2x 32 bit header + 32 bit payload

// maximum number of supported DMA channels for each device
#define MAX_DMA_CHANNELS 2
// maximum number of transactions for each DMA channel
#define MAX_DMA_READ_CHANNELTCS		4
// maximum number of scatter gather elements per transaction
#define MAX_DMA_READ_TCSGS			2048
#if (MAX_DMA_READ_CHANNELTCS * MAX_DMA_CHANNELS) > 28
 #error Too many DMA transfer channels (> 28)
#endif

#define DMA_READ_TC_SG_OFFSET 		0x40000			// FPGA address offset for SG elements
#define DMA_READ_TC_TC2TC_SETPBYTES 16				// size of SG entry: 4 flags + 4 size + 8 ptr
#define DMA_READ_TC_SG_MAX_BYTECOUNT ( (1<<22)-1 )	// transfer limit in bytes for SG element: FPGA limit is 20 bit word count (32-bit words)


#if PAGE_SIZE != 4096
 #error Page size must be 4096 bytes
#endif


/*** structs ***/
/******************************************************************************************/

// state definitions for SUN device read requests
enum SUN_REQ_STATE {
	SUN_REQ_STATE_FREE		= 0,	// DeviceID is not used by user space
	SUN_REQ_STATE_IDLE		= 1,	// DeviceID is used, but no request is active
	SUN_REQ_STATE_INFPGA	= 2,	// Request is in the FPGA or not handled by ISR yet
	SUN_REQ_STATE_RESULT	= 3,	// Request is handled by ISR waiting to be processed by process
	SUN_REQ_STATE_ABORT		= 4		// Request abort signaled to process
};

// SUN device read request structure
struct SUN_DEVICE_DATA {
	enum SUN_REQ_STATE requestState;
	u8 serialID;
	struct completion result_complete;
	u32	packet[MAX_SUNPACKETSIZE/4];
};


// DMA job structure
typedef struct _DMA_READ_JOB
{
 	uintptr_t 			pVMUser;			// user buffer
	size_t 				bufferSize;

	// job status, valid only if in Jobs_Done FIFO
	bool				boIsOk;				// no errors
	u16 				BufferCounter; 		// buffer counter comming from FPGA

	// pinned user buffer
	bool				boIsPageListValid;	// page list 'ppPageList' is allocated
	bool 				boIsPinned;			// get_user_pages() was called
	struct page **		ppPageList;
	u32					pagesPinned;

	// SG table
	bool 				boIsSGValid;		// SG table 'SGTable' is valid
	bool 				boIsSGMapped;		// SG table is mapped for DMA
	struct sg_table 	SGTable;
	u32					SGcount;			// number of mapped SG elements
	u32					SGItemsLeft;		// number of remaining SG elements for DMA to complete
	struct scatterlist	*pSGNext;			// SG elements to transfer
}  DMA_READ_JOB, *PDMA_READ_JOB;

// transfer channel structure
typedef struct _DMA_READ_TC
{
	bool 			boIsUsed;		// transfer channel is in use
	DMA_READ_JOB	*pJob;			// current job data (comming from Jobs_ToDo FIFO, going to Jobs_Done FIFO)
	u32				*pDesriptorFifo;
}  DMA_READ_TC, *PDMA_READ_TC;

// DMA channel structure
typedef struct _DMA_READ_CHANNEL
{
	DMA_READ_JOB *jobBuffers;						// storage of job buffers

	// job FIFOs store only pointer to jobs:
	DECLARE_KFIFO_PTR(Jobs_ToDo, PDMA_READ_JOB);	// pending transfer jobs
	DECLARE_KFIFO_PTR(Jobs_Done, PDMA_READ_JOB);	// transfer jobs done or aborted

	struct completion job_complete;					// DMA job completion
	u8 dmaWaitCount;								// number of threads waiting for completion
	u8 abortWait;									// signal DMA abort event to waiting threads

	DMA_READ_TC TCs[MAX_DMA_READ_CHANNELTCS];		// transfer channel data
	bool			doManualMap;
}  DMA_READ_CHANNEL, *PDMA_READ_CHANNEL;



// device data structure
typedef struct _DEVICE_DATA
{		
	//> Device	
	//***************************************************************/
	bool					boIsDeviceOpen;	//true <> Device ist valid
	struct cdev				DeviceCDev;		//das KernelObj vom Module	
	struct device*			dev;
	u8 						device_type;	//was sind wir AGEX, AGEX2... <> IMAGO_DEVICE_TYPE	
	struct semaphore		DeviceSem;		//lock für ein Device (diese struct & common buffer)
	dev_t					DeviceNumber;	//Nummer von CHAR device
	u8						flags;
	long					(*write)(struct _DEVICE_DATA *pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite);

	//> IDs/MetaInfos fuer ein read	
	//***************************************************************/
	spinlock_t				lock;
	struct SUN_DEVICE_DATA	SunDeviceData[MAX_IRQDEVICECOUNT];

	//> BAR0
	//***************************************************************/
	bool					boIsBAR0Requested;	//ist die Bar0 gültig
	void*					pVABAR0;			//zeigt auf den Anfang des gemapped mem vom PCIDev (eg 0xffffc90017480000)

	//> ~IRQ
	//***************************************************************/
	bool					boIsIRQOpen;
	
	//> CommonBuffer (AGEX2/4...)
	//***************************************************************/
	void* 					pVACommonBuffer;	//Virtuelleradresse	(eg: 0xffff8800d43dc000)
	dma_addr_t 				pBACommonBuffer;	//(Phy)(PCI)Busadresse (eg. 0xd43dc000)

	//> DMA (CL, VCXM)
	//***************************************************************/
	u8						DMARead_channels;	// actual number of DMA channels
	u8						DMARead_TCs;		// actual number of transfer channels
	u16  					DMARead_SGs;		// actual number of scatter gather elements
	spinlock_t				DMARead_SpinLock;	// DMA spinlock
	bool					setupTcInHWI;		// setup transfer channel in hardware interrupt
	bool					irqEnableInHWI;		// if disabled: DRA7x workaround for IRQ race in old kernels
	DMA_READ_CHANNEL		DMARead_Channel[MAX_DMA_CHANNELS];	// DMA channel data
} DEVICE_DATA, *PDEVICE_DATA;

// module data structure
typedef struct _MODULE_DATA
{
	DEVICE_DATA		*dev_data[MAX_DEVICE_COUNT];	// device data, index is the minor number
	dev_t 			FirstDeviceNumber;				// MAJOR(devNumber),MINOR(devNumber) (eg 240 , 0)
	struct class	*pModuleClass;					// /sys/class/*
	uint			max_dma_buffers;				// maximum number of DMA buffers

} MODULE_DATA, *PMODULE_DATA;


// globals
extern MODULE_DATA _ModuleData;
extern struct pci_driver imago_pci_driver;
extern struct spi_driver imago_spi_driver;

/*** prototypes ***/
/******************************************************************************************/
DEVICE_DATA *imago_alloc_dev_data(struct device *dev, u8 dev_type);
void imago_free_dev_data(DEVICE_DATA *pDevData);
long imago_locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes);
void imago_create_device(PDEVICE_DATA pDevData);
void imago_sun_interrupt(PDEVICE_DATA pDevData, u32 *sun_packet);

/* DMA functions */
int imago_DMARead_AddJob(PDEVICE_DATA pDevData, u32 iDMA, DMA_READ_JOB *pJob);
void imago_DMARead_DPC(PDEVICE_DATA pDevData);
void imago_DMARead_StartNextTransfer_Locked(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC);

int imago_DMARead_MapUserBuffer(PDEVICE_DATA pDevData, DMA_READ_CHANNEL *pDMAChannel, uintptr_t pVMUser, u64 bufferSize, DMA_READ_JOB **ppJob);
void imago_DMARead_UnMapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob);

int imago_DMARead_Abort_DMAChannel(PDEVICE_DATA pDevData, const u32 iDMA);
int imago_DMARead_Abort_DMAWaiter(PDEVICE_DATA pDevData,  const u32 iDMA);
int imago_DMARead_Reset_DMAChannel(PDEVICE_DATA pDevData, unsigned int dma_channel);


// device uses PCIe interface (common buffer + MSI)
static inline bool IS_TYPEWITH_COMMONBUFFER(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCIE) != 0);
}

// device uses PCI interface
static inline bool IS_TYPEWITH_PCI(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCI) != 0);
}

// device supports 64-bit addressing
static inline bool IS_TYPEWITH_PCI64BIT(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCI64BIT) != 0);
}

// device supports DMA
static inline bool IS_TYPEWITH_DMA2HOST(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_DMA2HOST) != 0);
}

static inline unsigned long imago_DMARead_Lock(DEVICE_DATA *pDevData)
{
	unsigned long flags = 0;
	if (pDevData->setupTcInHWI)
		spin_lock_irqsave(&pDevData->DMARead_SpinLock, flags);
	else
		spin_lock(&pDevData->DMARead_SpinLock);
	return flags;
}

static inline void imago_DMARead_Unlock(DEVICE_DATA *pDevData, unsigned long flags)
{
	if (pDevData->setupTcInHWI)
		spin_unlock_irqrestore(&pDevData->DMARead_SpinLock, flags);
	else
		spin_unlock(&pDevData->DMARead_SpinLock);
}

#endif /* AGEXDRV_H_ */

