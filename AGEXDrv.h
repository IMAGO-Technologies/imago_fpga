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
#define MODVERSION "1.1.6.2"
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
#include <linux/init.h>		// für module_init(),
#include <linux/module.h>	// für MODULE_LICENSE
#include <linux/version.h>	// für die Version

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	#include <linux/printk.h>	// für printk
#endif

#include <linux/types.h>	// für dev_t
#include <asm/types.h>		// für u8, s8
#include <linux/kdev_t.h>	// für MAJOR/MINOR
#include <linux/cdev.h>		// für cdev_*
#include <linux/device.h>	// für class_create
#include <linux/fs.h>		// für alloc_chrdev_region /file_*
#include <linux/semaphore.h>// für up/down ...
#include <linux/pci.h>		// für pci*
#include <linux/ioport.h>	// für _regio*
#include <linux/interrupt.h>// für IRQ*
#include <linux/dma-mapping.h>	//für dma_*
#if LINUX_VERSION_CODE != KERNEL_VERSION(2,6,32)
	#include <asm/uaccess.h>	// für copy_to_user
#else
	#include <linux/uaccess.h>	// für copy_to_user
#endif



/*** defines ***/
/******************************************************************************************/
typedef u8 IOCTLBUFFER[128];
#define FALSE 0
#define TRUE 1

//AGEX<>AGEX2<>Invalid
enum AGEX_DEVICE_SUBTYPE
{
	SubType_Invalid = 0,
	SubType_AGEX	= 1,
	SubType_AGEX2	= 2,
	SubType_MVC0	= 3
};


//> Ioctl definitions siehe. "ioctl-number.txt"
/******************************************************************************************/
//magic number
#define AGEXDRV_IOC_MAGIC  '['

//richtung ist aus UserSicht, size ist ehr der Type
#define AGEXDRV_IOC_GET_VERSION 	_IOR(AGEXDRV_IOC_MAGIC, 0, IOCTLBUFFER)
#define AGEXDRV_IOC_GET_BUILD_DATE 	_IOR(AGEXDRV_IOC_MAGIC, 1, IOCTLBUFFER)
#define AGEXDRV_IOC_RELEASE_DEVICEID _IOWR(AGEXDRV_IOC_MAGIC, 2, u8)
#define AGEXDRV_IOC_CREATE_DEVICEID _IOR(AGEXDRV_IOC_MAGIC, 3, u8)
#define AGEXDRV_IOC_GET_SUBTYPE 	_IOR(AGEXDRV_IOC_MAGIC, 4, u8)
#define AGEXDRV_IOC_ABORT_LONGTERM_READ _IOWR(AGEXDRV_IOC_MAGIC, 5, u8)

//max num (nur zum Testen)
#define AGEXDRV_IOC_MAXNR 5


//> Infos über die Device bzw. LongTermRequest
/******************************************************************************************/
//Wie viele devices können wir zugleich bedienen
#define MAX_DEVICE_COUNT 4

//Gibt wie viele LongTerm Requests offen sein dürfen/können
#define MAX_LONG_TERM_IO_REQUEST 8

// SUN "Geräte"
#define MAX_IRQDEVICECOUNT 128	// 2^n, max. 128
#define MAX_SUNPACKETSIZE  (4*3)// größe in Bytes mit Header

//1 schaltet den Interrupt an
#define ISR_ONOFF_OFFSET_AGEX (1<<20)
#define ISR_ONOFF_OFFSET_AGEX2 (0x10010)
//wo kommt die PCI-Adr des common buffers hin
#define ISR_COMMONBUFFER_ADR_AGEX2 (0x10000)
//1 sagt das, dass device einen Interrupt angelegt hat
#define ISR_AVAILABLE_OFFSET (1<<20)



/*** structs ***/
/******************************************************************************************/

/*NOTE:
 * es gibt 3 Arten der Nutzung einer DeviceID
 *
 * -> sie ist im FPGA unterwegs
 * -> ein Process wartet auf eine DeviceID
 * -> die DLL hat eine ID belegt
*/
//Fast alles zusammen was es über solch ein Request zu wissen gibt
typedef struct _LONG_TERM_IO_REQUEST
{
	//> eintrag ist erst frei wenn beide(boIsInFPGA, boIsInProcessUse) flags false sind
	//true gibt an das der Eintrag im FPGA ist, wird nur vom tasklet gelöscht
	bool boIsInFPGA;

	//true gibt an das ein process den eintrag noch nutzt, wird nur vom process gelöcht
	bool boIsInProcessUse ;

	//der Request selbst bzw der prozess
	struct semaphore WaitSem;
	
	//die semaphore wurde signalisiert, aber nicht im ISR sondern durch den User(IOCTL) ==> Abbruch
	//wird nur in Locked_startlongtermread() gelöscht
	bool boAbortWaiting;

	//ID um die Antwort wieder zu zuordnen
	u8	DeviceID;

	//im DPC/Tasklet wird das Paket aufgehoben(dort kann nicht in den USER Mem geschrieben werden)
	u32	IRQBuffer[MAX_SUNPACKETSIZE/4];
	u8	IRQBuffer_anzBytes;
}  LONGTERM_IOREQUEST, *PLONGTERM_IOREQUEST;

//Fast alle Infos zu seinen PCI(e) Device zusammen, das Module wird ja nur 1x pro System geladen
typedef struct _DEVICE_DATA
{		
	//> Device	
	bool			boIsDeviceOpen;	//true <> Device ist valid
	struct cdev		Device;			//das KernelObj vom Module	
	u8 				DeviceSubType;	//was sind wir AGEX, AGEX2... <> AGEX_DEVICE_SUBTYPE	
	struct semaphore DeviceSem;		//lock für ein Device (diese struct & common buffer)
	dev_t			DeviceNumber;	//Nummer von CHAR device

	//> IDs/MetaInfos für ein read	
	bool				boIsDeviceIDUsed[MAX_IRQDEVICECOUNT];	//Feld mit den DeviceIDs, UserMode fragt an, welche frei sind
	LONGTERM_IOREQUEST  LongTermRequestList[MAX_LONG_TERM_IO_REQUEST];

	//> BAR0
	bool			boIsBAR0Requested;	//ist die Bar0 gültig
	unsigned long	BAR0SizeBytes;
	void*			pVABAR0;			//zeigt auf den Anfang des gemapped mem vom PCIDev (eg 0xffffc90017480000)

	//> ~IRQ
	bool					boIsIRQOpen;	//true<>IRQ ist open
	bool 					boIsDPCRunning;	//damit nur ein DPC zur Zeit l�uft&gequeued 
	struct tasklet_struct	IRQTasklet;		//~SWI worker
	
	//> CommonBuffer (AGEX2)
	void* 		pVACommonBuffer;	//Virtuelleradresse	(eg: 0xffff8800d43dc000)
	dma_addr_t 	pBACommonBuffer;	//(Phy)(PCI)Busadresse (eg. 0xd43dc000)

} DEVICE_DATA, *PDEVICE_DATA;

//Fast alles zusammen was zu diesem Module gehört, (Note: n PCIdevs für das Module, Module wird nur 1x geladen)
typedef struct _MODULE_DATA
{
	DEVICE_DATA		Devs[MAX_DEVICE_COUNT];			//Infos/Context für je device, Index ist der Minor
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

/*Module fns*/
int AGEXDrv_init(void);
void AGEXDrv_exit(void);
void AGEXDrv_InitDrvData(PDEVICE_DATA pDat);



#endif /* AGEXDRV_H_ */

