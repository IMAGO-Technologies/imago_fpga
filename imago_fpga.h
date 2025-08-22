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

#ifndef IMAGO_FPGA_H_
#define IMAGO_FPGA_H_

// module definitions
/******************************************************************************************/
#define MODVERSION "1.3.2.0"
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
#include <linux/of_device.h>	// for of*
#include <linux/interrupt.h>// for IRQ*
#include <linux/dma-mapping.h>	// for dma_*
#include <linux/scatterlist.h>	// sg_* ...
#include <linux/spinlock.h>	// spin_* ...
#include <linux/delay.h>	// for usleep_range
#include <linux/uaccess.h>	// for copy_to_user
#include <linux/io.h>

#ifndef IS_ENABLED
#define IS_ENABLED(option) defined(option) || defined(option##_MODULE)
#endif

// writel_relaxed / writeq_relaxed is not defined for old kernels on all architectures
#ifndef writel_relaxed
	#define writel_relaxed(v, a) writel(v, a)
#endif
#ifndef writeq_relaxed
	#define writeq_relaxed(v, a) writeq(v, a)
#endif


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
	DeviceType_VSPV3	= 11,
	DeviceType_MVM2		= 12,
	DeviceType_AI		= 13,
	DeviceType_VCXM2	= 14,
};

// Device flags used by struct struct DEVICE_DATA
#define IMAGO_DEV_FLAG_PCIE			0x01
#define IMAGO_DEV_FLAG_PCI			0x02
#define IMAGO_DEV_FLAG_PCI64BIT		0x04
#define IMAGO_DEV_FLAG_DMA2HOST		0x08
#define IMAGO_DEV_FLAG_SPI			0x10
#define IMAGO_DEV_FLAG_HID			0x20


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
struct DMA_READ_JOB
{
 	uintptr_t 			pVMUser;			// user buffer

	u64					timestamp;
	u16 				BufferCounter; 		// buffer counter comming from FPGA
	bool				success;			// job status, valid only if in job_list_complete

	u32					pagesPinned;
	struct page **		ppPageList;

	struct sg_table 	SGTable;
	
	struct list_head	list;				// a job can be in job_list_allocated, job_list_pending, or job_list_complete
};

// transfer channel structure
struct DMA_READ_TC
{
	struct DMA_READ_JOB	*pJob;				// current job data
	struct scatterlist	*sg_list;			// SG list for current transfer
	u32					*pDesriptorFifo;
	u16					sg_remaining;		// number of remaining SG elements for DMA to complete
};

// DMA channel structure
struct DMA_READ_CHANNEL
{
	struct list_head	job_list_allocated;
	struct list_head	job_list_pending;
	struct list_head	job_list_complete;
	
	struct completion job_complete;					// DMA job completion
	u8 dmaWaitCount;								// number of threads waiting for completion
	u8 abortWait;									// signal DMA abort event to waiting threads

	struct DMA_READ_TC TCs[MAX_DMA_READ_CHANNELTCS];	// transfer channel data
	bool			doManualMap;
};


// device data structure
struct DEVICE_DATA
{		
	//> Device	
	//***************************************************************/
	bool					boIsDeviceOpen;	//true <> Device ist valid
	struct cdev				DeviceCDev;		//das KernelObj vom Module	
	struct device*			dev;
	struct device*			sub_dev;
	u8 						device_type;	//was sind wir AGEX, AGEX2... <> IMAGO_DEVICE_TYPE	
	struct semaphore		DeviceSem;		//lock für ein Device (diese struct & common buffer)
	dev_t					DeviceNumber;	//Nummer von CHAR device
	u8						flags;
	int						(*write)(struct DEVICE_DATA *pDevData, u32 *packet, unsigned int packet_size);

	//> SunDeviceData[] stores the state of read requests for different FPGA registers ('devices')
	//***************************************************************/
	raw_spinlock_t			lock;				// spin lock for access to SunDeviceData[], we use a raw spin
												// lock because a normal spin lock may sleep under PREEMPT_RT
	struct SUN_DEVICE_DATA	SunDeviceData[MAX_IRQDEVICECOUNT];

	//> BAR0
	//***************************************************************/
	bool					boIsBAR0Requested;	//ist die Bar0 gültig
	void*					pVABAR0;			//zeigt auf den Anfang des gemapped mem vom PCIDev (eg 0xffffc90017480000)

	//> CommonBuffer (AGEX2/4...)
	//***************************************************************/
	void* 					pVACommonBuffer;	//Virtuelleradresse	(eg: 0xffff8800d43dc000)
	dma_addr_t 				pBACommonBuffer;	//(Phy)(PCI)Busadresse (eg. 0xd43dc000)

	//> DMA (CL, VCXM)
	//***************************************************************/
	u8						DMARead_channels;	// actual number of DMA channels
	u8						DMARead_TCs;		// actual number of transfer channels
	u16  					DMARead_SGs;		// actual number of scatter gather elements
	raw_spinlock_t			DMARead_SpinLock;	// DMA spinlock
	bool					setupTcInHWI;		// setup transfer channel in hardware interrupt
	bool					irqEnableInHWI;		// if disabled: DRA7x workaround for IRQ race in old kernels
	struct DMA_READ_CHANNEL	DMARead_Channel[MAX_DMA_CHANNELS];	// DMA channel data
	struct kmem_cache		*dma_job_cache;
};

// module data structure
typedef struct _MODULE_DATA
{
	struct DEVICE_DATA *dev_data[MAX_DEVICE_COUNT];	// device data, index is the minor number
	dev_t 			FirstDeviceNumber;				// MAJOR(devNumber),MINOR(devNumber) (eg 240 , 0)
	struct class	*pModuleClass;					// /sys/class/*
	int				dma_update_in_hwi;
} MODULE_DATA, *PMODULE_DATA;


// globals
extern MODULE_DATA _ModuleData;
extern struct pci_driver imago_pci_driver;
extern struct spi_driver imago_spi_driver;
extern struct hid_driver imago_hid_driver;

/*** prototypes ***/
/******************************************************************************************/
struct DEVICE_DATA *imago_alloc_dev_data(struct device *dev, u8 dev_type);
void imago_free_dev_data(struct DEVICE_DATA *pDevData);
void imago_dev_close(struct DEVICE_DATA *pDevData);
long imago_locked_ioctl(struct DEVICE_DATA *pDevData, u32 cmd, u8 __user * pToUserMem);
int imago_create_device(struct DEVICE_DATA *pDevData);
long imago_create_deviceid(struct DEVICE_DATA *pDevData, u8* deviceIdOut);
long imago_release_deviceid(struct DEVICE_DATA *pDevData, u8 deviceID);
long imago_abort_longterm_read(struct DEVICE_DATA *pDevData, u8 deviceID);
void imago_sun_interrupt(struct DEVICE_DATA *pDevData, u32 *sun_packet);

/* DMA functions */
int imago_dma_addjob(struct DEVICE_DATA *pDevData, u32 iDMA, struct DMA_READ_JOB *pJob);
void imago_dma_event(struct DEVICE_DATA *pDevData);
int imago_dma_map(struct DEVICE_DATA *pDevData, struct DMA_READ_CHANNEL *pDMAChannel, uintptr_t pVMUser,
		u32 bufferSize, u8 reversePages, struct DMA_READ_JOB **ppJob);
void imago_dma_unmap(struct DEVICE_DATA *pDevData, struct DMA_READ_CHANNEL *pDMAChannel, struct DMA_READ_JOB *pJob);
int imago_dma_abort(struct DEVICE_DATA *pDevData, const u32 iDMA);
int imago_dma_abort_threads(struct DEVICE_DATA *pDevData,  const u32 iDMA);
int imago_dma_reset(struct DEVICE_DATA *pDevData, unsigned int dma_channel);

/* I2C adapter functions */
long imago_init_i2cAdapter(struct DEVICE_DATA *pDevData);
void imago_remove_i2cAdapter(void);

int imago_write_internal(struct DEVICE_DATA *pDevData, u32 *packet, unsigned int packet_size);
int imago_read_internal(struct DEVICE_DATA *pDevData, u32* buf, unsigned int count);

// device uses PCIe interface (common buffer + MSI)
static inline bool IS_TYPEWITH_COMMONBUFFER(struct DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCIE) != 0);
}

// device uses PCI interface
static inline bool IS_TYPEWITH_PCI(struct DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCI) != 0);
}

// device supports 64-bit addressing
static inline bool IS_TYPEWITH_PCI64BIT(struct DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_PCI64BIT) != 0);
}

// device supports DMA
static inline bool IS_TYPEWITH_DMA2HOST(struct DEVICE_DATA *pDeviceData)
{
	return ((pDeviceData->flags & IMAGO_DEV_FLAG_DMA2HOST) != 0);
}

static inline unsigned long imago_dma_lock(struct DEVICE_DATA *pDevData)
{
	unsigned long flags = 0;
	if (pDevData->setupTcInHWI)
		raw_spin_lock_irqsave(&pDevData->DMARead_SpinLock, flags);
	else
		raw_spin_lock(&pDevData->DMARead_SpinLock);
	return flags;
}

static inline void imago_dma_unlock(struct DEVICE_DATA *pDevData, unsigned long flags)
{
	if (pDevData->setupTcInHWI)
		raw_spin_unlock_irqrestore(&pDevData->DMARead_SpinLock, flags);
	else
		raw_spin_unlock(&pDevData->DMARead_SpinLock);
}

#endif /* IMAGO_FPGA_H_ */

