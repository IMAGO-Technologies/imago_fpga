/*
 * AGEXDrv.h
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.*
 */

#ifndef AGEXDRV_H_
#define AGEXDRV_H_

//> defines about the Module
/******************************************************************************************/
#define MODVERSION "1.1.7.1"
#define MODDATECODE __DATE__ " - " __TIME__
#define MODLICENSE "GPL";
#define MODDESCRIPTION "Kernel module for the VisionBox AGE-X PCI(e) devices";
#define MODAUTHOR "IMAGO Technologies GmbH";

#define MODCLASSNAME	"agexdrv"
#define MODMODULENAME	"agexpcidrv"
#define MODDEBUGOUTTEXT	"agexpcidrv:"


/*** includes ***/
/******************************************************************************************/
//aus usr/src/linux-headers-2.6.38-8-generic/include
#include <linux/init.h>		// f√ºr module_init(),
#include <linux/module.h>	// f√ºr MODULE_LICENSE
#include <linux/version.h>	// f√ºr die Version

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	#include <linux/printk.h>	// f√ºr printk
#endif

#include <linux/types.h>	// f√ºr dev_t
#include <asm/types.h>		// f√ºr u8, s8
#include <linux/sched.h>	// f¸r current (pointer to the current process)
#include <linux/fs_struct.h>
#include <linux/kdev_t.h>	// f√ºr MAJOR/MINOR
#include <linux/cdev.h>		// f√ºr cdev_*
#include <linux/device.h>	// f√ºr class_create
#include <linux/fs.h>		// f√ºr alloc_chrdev_region /file_*
#include <linux/semaphore.h>// f√ºr up/down ...
#include <linux/kfifo.h>	// f√ºr kfifo_*
#include <linux/pci.h>		// f√ºr pci*
#include <linux/ioport.h>	// f√ºr _regio*
#include <linux/interrupt.h>// f√ºr IRQ*
#include <linux/dma-mapping.h>	//f√ºr dma_*
#include <linux/scatterlist.h>	// sg_* ...
#include <linux/delay.h>	// f¸r usleep_range
#if LINUX_VERSION_CODE != KERNEL_VERSION(2,6,32)
	#include <asm/uaccess.h>	// f√ºr copy_to_user
#else
	#include <linux/uaccess.h>	// f√ºr copy_to_user
#endif



/*** defines ***/
/******************************************************************************************/
typedef u8 IOCTLBUFFER[128];
#define FALSE 0
#define TRUE 1

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)


//AGEX<>AGEX2<>Invalid
enum AGEX_DEVICE_SUBTYPE
{
	SubType_Invalid = 0,
	SubType_AGEX	= 1,
	SubType_AGEX2	= 2,
	SubType_MVC0	= 3,
	SubType_AGEX2_CL= 4,
	SubType_VCXM	= 5
};


//Device kann MSIX, CommonBuffer im PC-Mem
#define IS_TYPEWITH_COMMONBUFFER(DevType) ( (DevType==SubType_AGEX2) || (DevType==SubType_MVC0) || (DevType==SubType_AGEX2_CL) || (DevType==SubType_VCXM) )
//Device kann 64Bit Adressen
#define IS_TYPEWITH_PCI64BIT(DevType) ( (DevType==SubType_AGEX2_CL) )
//Device kann DMAReads (Device schreibt in PC-Mem)
#define IS_TYPEWITH_DMA2HOST(DevType) ( (DevType==SubType_AGEX2_CL) || (DevType==SubType_VCXM) )


//> Ioctl definitions siehe. "ioctl-number.txt"
/******************************************************************************************/
//magic number
#define AGEXDRV_IOC_MAGIC  '['

//richtung ist aus UserSicht, size ist ehr der Type 
//(Achtung! alteDefs nicht ‰ndern wegen alte libs)
#define AGEXDRV_IOC_GET_VERSION 			_IOR(AGEXDRV_IOC_MAGIC, 0, IOCTLBUFFER)
#define AGEXDRV_IOC_GET_BUILD_DATE 			_IOR(AGEXDRV_IOC_MAGIC, 1, IOCTLBUFFER)
#define AGEXDRV_IOC_RELEASE_DEVICEID 		_IOWR(AGEXDRV_IOC_MAGIC, 2, u8)
#define AGEXDRV_IOC_CREATE_DEVICEID 		_IOR(AGEXDRV_IOC_MAGIC, 3, u8)
#define AGEXDRV_IOC_GET_SUBTYPE 			_IOR(AGEXDRV_IOC_MAGIC, 4, u8)
#define AGEXDRV_IOC_ABORT_LONGTERM_READ 	_IOWR(AGEXDRV_IOC_MAGIC, 5, u8)

#define AGEXDRV_IOC_DMAREAD_CONFIG			_IOW(AGEXDRV_IOC_MAGIC, 6, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_ADD_BUFFER		_IOW(AGEXDRV_IOC_MAGIC, 7, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER	_IOWR(AGEXDRV_IOC_MAGIC, 8, IOCTLBUFFER)
#define AGEXDRV_IOC_DMAREAD_ABORT_DMA		_IOW(AGEXDRV_IOC_MAGIC, 9, u8)
#define AGEXDRV_IOC_DMAREAD_ABORT_WAITER	_IOW(AGEXDRV_IOC_MAGIC, 10, u8)

//max num (nur zum Testen)
#define AGEXDRV_IOC_MAXNR 10


//> Infos √ºber die Device bzw. LongTermRequest
/******************************************************************************************/
//Wie viele devices k√∂nnen wir zugleich bedienen
#define MAX_DEVICE_COUNT 4

//Gibt wie viele LongTerm Requests offen sein d√ºrfen/k√∂nnen
#define MAX_LONG_TERM_IO_REQUEST 8

// SUN "Ger√§te"
#define MAX_IRQDEVICECOUNT 128	// 2^n, max. 128
#define MAX_SUNPACKETSIZE  (4*3)// gr√∂√üe in Bytes mit Header

//1 schaltet den Interrupt an
#define ISR_ONOFF_OFFSET_AGEX (1<<20)
#define ISR_ONOFF_OFFSET_AGEX2 (0x10010)
//wo kommt die PCI-Adr des common buffers hin
#define ISR_COMMONBUFFER_ADR_AGEX2 (0x10000)
//1 sagt das, dass device einen Interrupt angelegt hat
#define ISR_AVAILABLE_OFFSET (1<<20)

//wie viele DMAs kann es geben(sagt nichts dar¸ber aus ob die HW so viele kann)
//[DMA*TC<=28 Bits sonst reicht das FlagReg' im CommonBuffer nicht mehr]
//[auch muss der CommonBuffer groﬂ genug f¸r die BufferZ‰hler]
//[beides sind im drv UINT8]
#define MAX_DMA_READ_DMACHANNELS 	4	
//wie viele DMATransaction pro DMAChannel
#define MAX_DMA_READ_CHANNELTCS		6
//wie viele SGs f¸r ein Transfer einer TC einer DMA (grˆﬂe h‰ngt am Kernel, ab SG_MAX_SINGLE_ALLOC dann chained)
#define MAX_DMA_READ_TCSGS			2048
#if (MAX_DMA_READ_CHANNELTCS*MAX_DMA_READ_DMACHANNELS) > 28
 #error MAX Bits 28 for DMARead!
#endif

#define DMA_READ_TC_SG_OFFSET 		0x40000
#define DMA_READ_TC_TC2TC_SETPBYTES (16/*4 flags + 4 size(DWORDs) + 8 ptr(4k align) */) /*muss nDWORDs sein*/

//die SGFlags
#define DMA_READ_TC_SG_FLAG_START_TRANSACTION 	(0x09) /* Loest FIFO-Reset aus + Bildstart */
#define DMA_READ_TC_SG_FLAG_START_TRANSFER 		(0x00) /* FPGA wertet einfach FIFO-empty Flag aus */
#define DMA_READ_TC_SG_FLAG_END_TRANSACTION		(0x02) /* DescrData.Link */
#define DMA_READ_TC_SG_FLAG_END_TRANSFER 		(0x04) /* DescrData.EnableIRQ */
#define DMA_READ_TC_SG_FLAG_ERROR 				(0x10 | DMA_READ_TC_SG_FLAG_END_TRANSACTION | DMA_READ_TC_SG_FLAG_END_TRANSFER)



//Anzahl der Elemente im .Jobs_ToTo, .Jobs_Done FIFO
#define MAX_DMA_READ_JOBFIFO_SIZE	32


//2xDWORD, 		IRQFlags
//2xDWORD, 		IRQPaket, Header0/Header1
//128xDWORD, 	IRQPaket, Daten	
//DMAs*TCsx8,	DMABuffer Z‰hler (UINT16)
#define HOST_BUFFER_SIZE ((4*(2+2+128))+(MAX_DMA_READ_DMACHANNELS*MAX_DMA_READ_CHANNELTCS*8)) 
#define HOST_BUFFER_DMAREAD_COUNTER_OFFSET (4*(2+2+128))


//f¸r DMA_READ_* brauchen wir eine PageSize von 4k 
#if PAGE_SIZE!=4096
 #error PageSize must be 4096!
#endif


/*** structs ***/
/******************************************************************************************/

/*NOTE:
 * es gibt 3 Arten der Nutzung einer DeviceID
 *
 * -> sie ist im FPGA unterwegs
 * -> ein Process wartet auf eine DeviceID
 * -> die DLL hat eine ID belegt
*/

//Fast alles zusammen was es √ºber solch ein Request zu wissen gibt
typedef struct _LONG_TERM_IO_REQUEST
{
	//> eintrag ist erst frei wenn beide(boIsInFPGA, boIsInProcessUse) flags false sind
	//true gibt an das der Eintrag im FPGA ist, wird nur vom tasklet gel√∂scht
	bool boIsInFPGA;

	//true gibt an das ein process den eintrag noch nutzt, wird nur vom process gel√∂cht
	bool boIsInProcessUse ;

	//der Request selbst bzw der prozess
	struct semaphore WaitSem;
	
	//die semaphore wurde signalisiert, aber nicht im ISR sondern durch den User(IOCTL) ==> Abbruch
	//wird nur in Locked_startlongtermread() gel√∂scht
	bool boAbortWaiting;

	//ID um die Antwort wieder zu zuordnen
	u8	DeviceID;

	//im DPC/Tasklet wird das Paket aufgehoben(dort kann nicht in den USER Mem geschrieben werden)
	u32	IRQBuffer[MAX_SUNPACKETSIZE/4];
	u8	IRQBuffer_anzBytes;
}  LONGTERM_IOREQUEST, *PLONGTERM_IOREQUEST;



//Fast alles zusammen f¸r ein UserBuffer f¸r eine DMA (Read)
typedef struct _DMA_READ_JOB
{
	//UserBuffer
 	uintptr_t 	pVMUser;			//NULL ptr, dann ein DummyJob vom Abort
	size_t 		anzBytesToTransfer;

	//Status (nur g¸ltig/gesetzt) wenn in .Jobs_Done)
	bool		boIsOk;				//ohne Fehler ¸bertragen
	u16 		BufferCounter; 		//laufender Z‰hler, FPGA z‰hlt

	//Pinned UserBuffer
	bool			boIsPageListValid;	//1<>PageListe wurde angelegt 
	bool 			boIsPinned;			//1<>UserMem ist gepinned
	struct page **	ppPageList;
	u32				anzPagesPinned;

	//SGListe
	bool 			boIsSGValid;	//1<>sgTable wurde angelegt 
	bool 			boIsSGMapped;	//1<>wurde gemapped
	struct sg_table SGTable;	
	u32				anzSGItemsMapped;
	struct scatterlist *pSGNext;	//N‰chste Element was gesendet werden kann, kann aber auch sg_is_last()==true sein oder NULL f¸r last+1
}  DMA_READ_JOB, *PDMA_READ_JOB;

//Fast alles zusammen f¸r ein DMA (Read) TC 
typedef struct _DMA_READ_TC
{
	bool 			boIsUsed;		//1<>Rest ist der struct g¸ltig bzw. die boIsxxx

	//der Job(UserBuffer)
	DMA_READ_JOB	Job;
}  DMA_READ_TC, *PDMA_READ_TC;

//Fast alles zusammen f¸r einen DMA (Read) Channel
typedef struct _DMA_READ_CHANNEL
{
	DECLARE_KFIFO(Jobs_ToDo, DMA_READ_JOB, MAX_DMA_READ_JOBFIFO_SIZE);	//sind nicht einer DMA/TC zugewiesen (auch nicht gewesen)
	DECLARE_KFIFO(Jobs_Done, DMA_READ_JOB, MAX_DMA_READ_JOBFIFO_SIZE);	//sind nicht mehr einer DMA/TC zugewiesen (kann aber auch abgebrochen worden sein)

	struct semaphore WaitSem;									// der WaitRequest, es ist im Jobs_Done was drin, im AbortFall ist es ein DummyJob

	DMA_READ_TC		TCs[MAX_DMA_READ_CHANNELTCS];				//liste der mˆglichen TCs
}  DMA_READ_CHANNEL, *PDMA_READ_CHANNEL;



//Fast alle Infos zu seinen PCI(e) Device zusammen
typedef struct _DEVICE_DATA
{		
	//> Device	
	//***************************************************************/
	bool			boIsDeviceOpen;	//true <> Device ist valid
	struct cdev		DeviceCDev;		//das KernelObj vom Module	
	struct device*	pDeviceDevice;	//pcidev->dev
	u8 				DeviceSubType;	//was sind wir AGEX, AGEX2... <> AGEX_DEVICE_SUBTYPE	
	struct semaphore DeviceSem;		//lock f√ºr ein Device (diese struct & common buffer)
	dev_t			DeviceNumber;	//Nummer von CHAR device

	//> IDs/MetaInfos f√ºr ein read	
	//***************************************************************/
	bool				boIsDeviceIDUsed[MAX_IRQDEVICECOUNT];	//Feld mit den DeviceIDs, UserMode fragt an, welche frei sind
	LONGTERM_IOREQUEST  LongTermRequestList[MAX_LONG_TERM_IO_REQUEST];

	//> BAR0
	//***************************************************************/
	bool			boIsBAR0Requested;	//ist die Bar0 g√ºltig
	unsigned long	BAR0SizeBytes;
	void*			pVABAR0;			//zeigt auf den Anfang des gemapped mem vom PCIDev (eg 0xffffc90017480000)

	//> ~IRQ
	//***************************************************************/
	bool					boIsIRQOpen;	//true<>IRQ ist open
	bool 					boIsDPCRunning;	//damit nur ein DPC zur Zeit l‰uft&gequeued 
	struct tasklet_struct	IRQTasklet;		//~SWI worker
	
	//> CommonBuffer (AGEX2/4...)
	//***************************************************************/
	void* 					pVACommonBuffer;	//Virtuelleradresse	(eg: 0xffff8800d43dc000)
	dma_addr_t 				pBACommonBuffer;	//(Phy)(PCI)Busadresse (eg. 0xd43dc000)

	//> DMA (AGEX2 CL, VCXM)
	//***************************************************************/
	u8						DMARead_anzChannels;//wie viele Channel gibt es?
	u8						DMARead_anzTCs;		//wie viele TCs pro Channel?
	u16  					DMARead_anzSGs;		//wie viele Scatter/Gather Pairs pro TC 
	struct semaphore		DMARead_SpinLock;	//Lock f¸r DMAStructs/und DMAUnit im FPGA
	DMA_READ_CHANNEL		DMARead_Channels[MAX_DMA_READ_DMACHANNELS];	//Params f¸r alle mˆglichen DMAChannels	
} DEVICE_DATA, *PDEVICE_DATA;

//Fast alles zusammen was zu diesem Module geh√∂rt, (Note: n PCIdevs f√ºr das Module, Module wird nur 1x geladen)
typedef struct _MODULE_DATA
{
	DEVICE_DATA		Devs[MAX_DEVICE_COUNT];			//Infos/Context f√ºr je device, Index ist der Minor
	bool			boIsMinorUsed[MAX_DEVICE_COUNT];//daher ist Devs[Minor] benutzt/frei?

	dev_t 			FirstDeviceNumber;	//MAJOR(devNumber),MINOR(devNumber) (eg 240 , 0)
	struct class	*pModuleClass;		// /sys/class/*

} MODULE_DATA, *PMODULE_DATA;


//Global vars
extern MODULE_DATA _ModuleData;


/*** prototypes ***/
/******************************************************************************************/
/*LOCKED fns*/
long Locked_startlongtermread(PDEVICE_DATA pDevData, const u32 DeviceID);
long Locked_write(PDEVICE_DATA pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite);
long Locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes);

/*File fns*/
int AGEXDrv_open(struct inode *node, struct file *filp);
ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos);
ssize_t AGEXDrv_write (struct file *filp, const char __user *buf, size_t count,loff_t *pos);
long AGEXDrv_unlocked_ioctl (struct file *filp, unsigned int cmd,unsigned long arg);

/* ~IRQ fns*/
irqreturn_t AGEXDrv_interrupt(int irq, void *dev_id);
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn);
void AGEXDrv_tasklet(unsigned long devIndex);

/* PCI fns*/
int AGEXDrv_PCI_probe(struct pci_dev *pcidev, const struct pci_device_id *id);
void AGEXDrv_PCI_remove(struct pci_dev *pcidev);

/* DMA fns */
void AGEXDrv_DMARead_StartDMA(PDEVICE_DATA pDevData);
void AGEXDrv_DMARead_DPC(PDEVICE_DATA pDevData, const u32 isDoneReg, const u32 isOkReg, const u16* pBufferCounters);
void AGEXDrv_DMARead_StartNextTransfer_Locked(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC);

bool AGEXDrv_DMARead_MapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob, uintptr_t pVMUser, u64 anzBytesToTransfer);
void AGEXDrv_DMARead_UnMapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob);

void AGEXDrv_DMARead_Abort_DMAChannel(PDEVICE_DATA pDevData, const u32 iDMA);
void AGEXDrv_DMARead_Abort_DMAWaiter(PDEVICE_DATA pDevData,  const u32 iDMA);


/*Module fns*/
int AGEXDrv_init(void);
void AGEXDrv_exit(void);
void AGEXDrv_InitDrvData(PDEVICE_DATA pDat);



#endif /* AGEXDRV_H_ */

