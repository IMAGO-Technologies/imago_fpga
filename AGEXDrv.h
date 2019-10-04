/*
 * AGEXDrv.h
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.*
 */

#ifndef AGEXDRV_H_
#define AGEXDRV_H_

//> defines about the Module
/******************************************************************************************/
#define MODVERSION "1.1.10.0"
#define MODDATECODE __DATE__ " - " __TIME__
#define MODLICENSE "GPL";
#define MODDESCRIPTION "IMAGO FPGA / RTCC device driver";
#define MODAUTHOR "IMAGO Technologies GmbH";

#define MODCLASSNAME	"agexdrv"
#define MODMODULENAME	"agexpcidrv"
#define MODDEBUGOUTTEXT	"agexpcidrv:"


/*** includes ***/
/******************************************************************************************/
#include <linux/init.h>		// for module_init(),
#include <linux/module.h>	// for MODULE_LICENSE
#include <linux/version.h>	// for die Version

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	#include <linux/printk.h>	// for printk
#endif

#include <linux/types.h>	// for dev_t
#include <asm/types.h>		// for u8, s8
#include <linux/sched.h>	// for current (pointer to the current process)
#include <linux/fs_struct.h>
#include <linux/kdev_t.h>	// for MAJOR/MINOR
#include <linux/cdev.h>		// for cdev_*
#include <linux/device.h>	// for class_create
#include <linux/fs.h>		// for alloc_chrdev_region /file_*
#include <linux/semaphore.h>// for up/down ...
#include <linux/kfifo.h>	// for kfifo_*
#include <linux/of_device.h>	// for of*
#include <linux/pci.h>		// for pci*
#include <linux/spi/spi.h>		// for spi*
#include <linux/ioport.h>	// for _regio*
#include <linux/interrupt.h>// for IRQ*
#include <linux/dma-mapping.h>	// for dma_*
#include <linux/scatterlist.h>	// sg_* ...
#include <linux/spinlock.h>	// spin_* ...
#include <linux/delay.h>	// for usleep_range
#include <linux/uaccess.h>	// for copy_to_user



/*** defines ***/
/******************************************************************************************/
typedef u8 IOCTLBUFFER[128];
#define FALSE 0
#define TRUE 1

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)


// Hardware type definitions
enum AGEX_DEVICE_SUBTYPE
{
	SubType_Invalid 	= 0,
	SubType_AGEX		= 1,
	SubType_AGEX2		= 2,
	SubType_MVC0		= 3,
	SubType_AGEX2_CL	= 4,
	SubType_VCXM		= 5,
	SubType_LEMANS		= 6,
   	SubType_PCIE_CL		= 7,
	SubType_AGEX5		= 8,
	SubType_AGEX5_CL	= 9,
	SubType_DAYTONA		= 10
};

// Device info flags used by struct sAGEXDrv_device_info
#define AGEXDRV_FLAG_COMMONBUFFER	0x01
#define AGEXDRV_FLAG_PCI			0x02
#define AGEXDRV_FLAG_PCI64BIT		0x04
#define AGEXDRV_FLAG_DMA2HOST		0x08
#define AGEXDRV_FLAG_SPI			0x10


//> Ioctl definitions, see "ioctl-number.txt"
/******************************************************************************************/
// magic number
#define AGEXDRV_IOC_MAGIC  '['

// direction is from user's view
#define AGEXDRV_IOC_GET_VERSION 				_IOR(AGEXDRV_IOC_MAGIC, 0, IOCTLBUFFER)
#define AGEXDRV_IOC_GET_BUILD_DATE 				_IOR(AGEXDRV_IOC_MAGIC, 1, IOCTLBUFFER)
#define AGEXDRV_IOC_RELEASE_DEVICEID 			_IOWR(AGEXDRV_IOC_MAGIC, 2, u8)
#define AGEXDRV_IOC_CREATE_DEVICEID 			_IOR(AGEXDRV_IOC_MAGIC, 3, u8)
#define AGEXDRV_IOC_GET_SUBTYPE 				_IOR(AGEXDRV_IOC_MAGIC, 4, u8)
#define AGEXDRV_IOC_ABORT_LONGTERM_READ 		_IOWR(AGEXDRV_IOC_MAGIC, 5, u8)
#define AGEXDRV_IOC_DMAREAD_CONFIG				_IOW(AGEXDRV_IOC_MAGIC, 6, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_ADD_BUFFER			_IOW(AGEXDRV_IOC_MAGIC, 7, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER		_IOWR(AGEXDRV_IOC_MAGIC, 8, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_ABORT_DMA			_IOW(AGEXDRV_IOC_MAGIC, 9, u8)
#define AGEXDRV_IOC_DMAREAD_ABORT_WAITER		_IOW(AGEXDRV_IOC_MAGIC, 10, u8)
#define AGEXDRV_IOC_DMAREAD_RESETCHANNEL		_IOW(AGEXDRV_IOC_MAGIC, 11, u8)
#define AGEXDRV_IOC_DMAREAD_MAP_BUFFER			_IOWR(AGEXDRV_IOC_MAGIC, 12, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_UNMAP_BUFFER		_IOW(AGEXDRV_IOC_MAGIC, 13, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_ADD_MAPPED_BUFFER	_IOW(AGEXDRV_IOC_MAGIC, 14, IOCTLBUFFER)


/******************************************************************************************/

// maximum number of supported devices (FPGAs)
#define MAX_DEVICE_COUNT 4

// number of SUN device IDs for each FPGA
#define MAX_IRQDEVICECOUNT 64
#define MAX_SUNPACKETSIZE  (4*3) // SUN packet size, 2x 32 bit header + 32 bit payload

// register offset for interrupt enable
#define ISR_ONOFF_OFFSET_AGEX (1<<20)
#define ISR_ONOFF_OFFSET_AGEX2 (0x10010)
// register offset for setting the address of interrupt data
#define ISR_COMMONBUFFER_ADR_AGEX2 (0x10000)
// register offset for reading interrupt flag and FIFO level
#define ISR_AVAILABLE_OFFSET (1<<20)

// maximum number of supported DMA channels
#define MAX_DMA_READ_DMACHANNELS 	4	
// maximum number of transactions for each DMA channel
#define MAX_DMA_READ_CHANNELTCS		6
// maximum number of scatter gather elements per transaction
#define MAX_DMA_READ_TCSGS			2048
#if (MAX_DMA_READ_CHANNELTCS*MAX_DMA_READ_DMACHANNELS) > 28
 #error Too many DMA transfer channels (> 28)
#endif

#define DMA_READ_TC_SG_OFFSET 		0x40000			// FPGA address offset for SG elements
#define DMA_READ_TC_TC2TC_SETPBYTES 16				// size of SG entry: 4 flags + 4 size + 8 ptr
#define DMA_READ_TC_SG_MAX_BYTECOUNT ( (1<<22)-1 )	// transfer limit in bytes for SG element: FPGA limit is 20 bit word count (32-bit words)

// flags for SG elements
#define DMA_READ_TC_SG_FLAG_START_TRANSACTION 	(0x09) // transaction start: SG descriptor FIFO reset + DMA start flag
#define DMA_READ_TC_SG_FLAG_START_TRANSFER 		(0x00) // intermediate transfer: no additional flags required
#define DMA_READ_TC_SG_FLAG_END_TRANSACTION		(0x06) // end of transaction: discriptor link + interrupt flag
#define DMA_READ_TC_SG_FLAG_END_TRANSFER 		(0x04) // end of intermediate transfer: discriptor link + interrupt flag 
#define DMA_READ_TC_SG_FLAG_ERROR 				(0x10 | DMA_READ_TC_SG_FLAG_END_TRANSACTION)


// maximum number of queued jobs for each DMA channel handled by the driver
#define MAX_DMA_READ_JOBFIFO_SIZE	32


//2xDWORD, 		IRQFlags
//2xDWORD, 		IRQPacket, Header0/Header1
//128xDWORD, 	IRQPacket, data
//DMAs*TCsx8,	DMABuffer counters (UINT16)
#define HOST_BUFFER_SIZE ((4*(2+2+128))+(MAX_DMA_READ_DMACHANNELS*MAX_DMA_READ_CHANNELTCS*8)) 
#define HOST_BUFFER_DMAREAD_COUNTER_OFFSET (4*(2+2+128))


#if PAGE_SIZE != 4096
 #error Page size must be 4096 bytes
#endif


/*** structs ***/
/******************************************************************************************/

struct sAGEXDrv_device_info {
	char *name;
	u8 flags;
};

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
	struct semaphore semResult;
	u32	packet[MAX_SUNPACKETSIZE/4];
};


// DMA job structure
typedef struct _DMA_READ_JOB
{
 	uintptr_t 	pVMUser;			// user buffer
	size_t 		bufferSize;

	// job status, valid only if in Jobs_Done FIFO
	bool		boIsOk;				// no errors
	u16 		BufferCounter; 		// buffer counter comming from FPGA

	// pinned user buffer
	bool			boIsPageListValid;	// page list 'ppPageList' is allocated
	bool 			boIsPinned;			// get_user_pages() was called
	struct page **	ppPageList;
	u32				pagesPinned;

	// SG table
	bool 			boIsSGValid;	// SG table 'SGTable' is valid
	bool 			boIsSGMapped;	// SG table is mapped for DMA
	struct sg_table SGTable;
	u32				SGcount;		// number of mapped SG elements
	u32				SGItemsLeft;	// number of remaining SG elements for DMA to complete
	struct scatterlist *pSGNext;	// SG elements to transfer
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
	DMA_READ_JOB jobBuffers[MAX_DMA_READ_JOBFIFO_SIZE-1];				// storage of job buffers, -1 for dummyJob in kfifo
	DMA_READ_JOB dummyJob;												// dummy job for DMA abort

	// job FIFOs store only pointer to jobs:
	DECLARE_KFIFO(Jobs_ToDo, PDMA_READ_JOB, MAX_DMA_READ_JOBFIFO_SIZE);	// pending transfer jobs
	DECLARE_KFIFO(Jobs_Done, PDMA_READ_JOB, MAX_DMA_READ_JOBFIFO_SIZE);	// transfer jobs done or aborted

	struct semaphore WaitSem;									// counting semapore, increments for each job in Jobs_Done

	DMA_READ_TC TCs[MAX_DMA_READ_CHANNELTCS];					// transfer channel data
	bool doManualMap;
}  DMA_READ_CHANNEL, *PDMA_READ_CHANNEL;



// device data structure
typedef struct _DEVICE_DATA
{		
	//> Device	
	//***************************************************************/
	bool			boIsDeviceOpen;	//true <> Device ist valid
	struct cdev		DeviceCDev;		//das KernelObj vom Module	
	struct device*	dev;	//pcidev->dev
	u8 				DeviceSubType;	//was sind wir AGEX, AGEX2... <> AGEX_DEVICE_SUBTYPE	
	struct semaphore DeviceSem;		//lock für ein Device (diese struct & common buffer)
	dev_t			DeviceNumber;	//Nummer von CHAR device
	u8				flags;

	//> IDs/MetaInfos fuer ein read	
	//***************************************************************/
	spinlock_t				lock;
	struct SUN_DEVICE_DATA	SunDeviceData[MAX_IRQDEVICECOUNT];

	//> BAR0
	//***************************************************************/
	bool			boIsBAR0Requested;	//ist die Bar0 gültig
//	unsigned long	BAR0SizeBytes;
	void*			pVABAR0;			//zeigt auf den Anfang des gemapped mem vom PCIDev (eg 0xffffc90017480000)

	//> ~IRQ
	//***************************************************************/
	bool					boIsIRQOpen;	//true<>IRQ ist open
	bool 					boIsDPCRunning;	//damit nur ein DPC zur Zeit l�uft&gequeued 
	struct tasklet_struct	IRQTasklet;		//~SWI worker
	
	//> CommonBuffer (AGEX2/4...)
	//***************************************************************/
	void* 					pVACommonBuffer;	//Virtuelleradresse	(eg: 0xffff8800d43dc000)
	dma_addr_t 				pBACommonBuffer;	//(Phy)(PCI)Busadresse (eg. 0xd43dc000)

	//> DMA (AGEX2 CL, VCXM)
	//***************************************************************/
	u8						DMARead_channels;	// actual number of DMA channels
	u8						DMARead_TCs;		// actual number of transfer channels
	u16  					DMARead_SGs;		// actual number of scatter gather elements
	spinlock_t				DMARead_SpinLock;	// DMA spinlock
	DMA_READ_CHANNEL		DMARead_Channel[MAX_DMA_READ_DMACHANNELS];	// DMA channel data
} DEVICE_DATA, *PDEVICE_DATA;

// module data structure
typedef struct _MODULE_DATA
{
	DEVICE_DATA		Devs[MAX_DEVICE_COUNT];				// device data, index is minor number
	bool			boIsMinorUsed[MAX_DEVICE_COUNT];	// used flag

	dev_t 			FirstDeviceNumber;	// MAJOR(devNumber),MINOR(devNumber) (eg 240 , 0)
	struct class	*pModuleClass;		// /sys/class/*

} MODULE_DATA, *PMODULE_DATA;


// globals
extern MODULE_DATA _ModuleData;
extern struct sAGEXDrv_device_info AGEXDrv_device_info[];
extern struct spi_driver imago_spi_driver;

/*** prototypes ***/
/******************************************************************************************/
/* locked functions */
long Locked_write(PDEVICE_DATA pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite);
long Locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes);

/* file operations */
int AGEXDrv_open(struct inode *node, struct file *filp);
ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos);
ssize_t AGEXDrv_write (struct file *filp, const char __user *buf, size_t count,loff_t *pos);
long AGEXDrv_unlocked_ioctl (struct file *filp, unsigned int cmd,unsigned long arg);

/* ~IRQ functions */
irqreturn_t AGEXDrv_interrupt(int irq, void *dev_id);
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn);
void AGEXDrv_tasklet_PCIe(unsigned long data);
void AGEXDrv_tasklet_PCI(unsigned long data);
void AGEXDrv_tasklet_SPI(unsigned long data);

/* PCI functions */
int AGEXDrv_PCI_probe(struct pci_dev *pcidev, const struct pci_device_id *id);
void AGEXDrv_PCI_remove(struct pci_dev *pcidev);

/* DMA functions */
int AGEXDrv_DMARead_AddJob(PDEVICE_DATA pDevData, u32 iDMA, DMA_READ_JOB *pJob);
void AGEXDrv_DMARead_DPC(PDEVICE_DATA pDevData, const u32 isDoneReg, const u32 isOkReg, const u16* pBufferCounters);
void AGEXDrv_DMARead_StartNextTransfer_Locked(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC);

bool AGEXDrv_DMARead_MapUserBuffer(PDEVICE_DATA pDevData, DMA_READ_CHANNEL *pDMAChannel, uintptr_t pVMUser, u64 bufferSize, DMA_READ_JOB **ppJob);
void AGEXDrv_DMARead_UnMapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob);

void AGEXDrv_DMARead_Abort_DMAChannel(PDEVICE_DATA pDevData, const u32 iDMA);
void AGEXDrv_DMARead_Abort_DMAWaiter(PDEVICE_DATA pDevData,  const u32 iDMA);
int AGEXDrv_DMARead_Reset_DMAChannel(PDEVICE_DATA pDevData, unsigned int dma_channel);


/* module functions */
int AGEXDrv_init(void);
void AGEXDrv_exit(void);
void AGEXDrv_InitDrvData(PDEVICE_DATA pDat);


// device uses PCIe interface (common buffer + MSI)
static inline bool IS_TYPEWITH_COMMONBUFFER(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & AGEXDRV_FLAG_COMMONBUFFER) != 0);
}

// device uses PCI interface
static inline bool IS_TYPEWITH_PCI(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & AGEXDRV_FLAG_PCI) != 0);
}

// device supports 64-bit addressing
static inline bool IS_TYPEWITH_PCI64BIT(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & AGEXDRV_FLAG_PCI64BIT) != 0);
}

// device supports DMA
static inline bool IS_TYPEWITH_DMA2HOST(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & AGEXDRV_FLAG_DMA2HOST) != 0);
}

// device uses SPI interface
static inline bool IS_TYPEWITH_SPI(DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & AGEXDRV_FLAG_SPI) != 0);
}


#endif /* AGEXDRV_H_ */

