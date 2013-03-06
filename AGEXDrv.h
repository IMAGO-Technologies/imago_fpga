/*
 * AGEXDrv.h
 *
 *  Created on: 20.12.2011
 *      Author: imago
 */

#ifndef AGEXDRV_H_
#define AGEXDRV_H_

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
#if LINUX_VERSION_CODE != KERNEL_VERSION(2,6,32)
	#include <asm/uaccess.h>	// für copy_to_user
#else
	#include <linux/uaccess.h>	// für copy_to_user
#endif

typedef u8 IOCTLBUFFER[128];
#define FALSE 0
#define TRUE 1

/*** LOCKED fns ***/
long Locked_startlongtermread(const u32 DeviceID);
long Locked_write(const u8 __user * pToUserMem, const size_t BytesToWrite);
long Locked_ioctl(const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes);


/*** Infos über die DeviceID buw LongTermRequest ***/
//Gibt wie viele LongTerm Requests offen sein dürfen/können
#define MAX_LONG_TERM_IO_REQUEST 8

// SUN "Geräte"
#define MAX_IRQDEVICECOUNT 128	// 2^n, max. 128
#define MAX_SUNPACKETSIZE  (4*3)// größe in Bytes mit Header

//1 schaltet den Interrupt an
#define ISR_ONOFF_OFFSET (1<<20)
//1 sagt das, dass device einen Interrupt angelegt hat
#define ISR_AVAILABLE_OFFSET (1<<20)


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
	//> eintrag ist erst frei wenn beide flags false sind
	//true gibt an das der Eintrag im FPGA ist, wird nur vom tasklet gelöscht
	bool boIsInFPGA;

	//true gibt an das ein process den eintrag noch nutzt, wird nur vom process gleöcht
	bool boIsInProcessUse ;

	//der Request selbst bzw der prozess
	struct semaphore WaitSem;

	//ID um die Antwort wieder zu zuordnen
	u8	DeviceID;

	//im DPC/Tasklet wird das Paket aufgehoben(dort kann nicht in den USER Mem geschrieben werden)
	u32	IRQBuffer[MAX_SUNPACKETSIZE/4];
	u8	IRQBuffer_anzBytes;
}  LONGTERM_IOREQUEST , *PLONGTERM_IOREQUEST;


//Feld mit den DeviceIDs, UserMode fragt an welche frei sind
extern bool _boIsDeviceIDUsed[MAX_IRQDEVICECOUNT];
extern LONGTERM_IOREQUEST  _LongTermRequestList[MAX_LONG_TERM_IO_REQUEST];




/*** Ioctl definitions siehe. "ioctl-number.txt" ***/
//magic number
#define AGEXDRV_IOC_MAGIC  '['

//richtung ist aus UserSicht, size ist ehr der Type
#define AGEXDRV_IOC_GET_VERSION 	_IOR(AGEXDRV_IOC_MAGIC, 0, IOCTLBUFFER)
#define AGEXDRV_IOC_GET_BUILD_DATE 	_IOR(AGEXDRV_IOC_MAGIC, 1, IOCTLBUFFER)
#define AGEXDRV_IOC_RELEASE_DEVICEID _IOWR(AGEXDRV_IOC_MAGIC, 2, u8)
#define AGEXDRV_IOC_CREATE_DEVICEID _IOR(AGEXDRV_IOC_MAGIC, 3, u8)


//max num (nur zum Testen)
#define AGEXDRV_IOC_MAXNR 3

//zeigt auf den Anfang des gemapped mem vom PCIDev
extern void* _PCI_IOMEM_StartAdr;

#endif /* AGEXDRV_H_ */

