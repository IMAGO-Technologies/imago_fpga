/*
 * FileOps.c
 *
 * handel the file read/write/io actions
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
 * -> module (un)load, read & write & IO, PCI probe/remove, IRQ(ISR)
 *
 */

#include "AGEXDrv.h"


//<====================================>
//	File fns
//<====================================>


int AGEXDrv_open(struct inode *node, struct file *filp)
{
	int iMinor;
	if( (node==NULL) || (filp==NULL) )
		return -EINVAL; 
	iMinor = iminor(node);

	pr_devel(MODDEBUGOUTTEXT" open (Minor:%d)\n", iMinor);

	//setzt ins "file" den Context
	if(iMinor >= MAX_DEVICE_COUNT)
		return -EINVAL;
	filp->private_data = &_ModuleData.Devs[iMinor];

	return 0;
}


//Note: alte ioctl war unter "big kernel lock"
//http://lwn.net/Articles/119652/
long AGEXDrv_unlocked_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	long ret = 0;
	PDEVICE_DATA pDevData = NULL;

	//Note: beim 1. write bekommen wir ein CMD:1, Type 'T', Size: 0 <-> FIONBIO (linux/include/asm-i386/ioctls.h, line 41)
	pr_devel(MODDEBUGOUTTEXT" ioctl (CMD %d, MAGIC %c, size %d)\n",_IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd));

	/* Alles gut */
	if( (filp==NULL) || (filp->private_data==NULL) ) return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;
	//ist ist das CMD eins für uns?
	if (_IOC_TYPE(cmd) != AGEXDRV_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > AGEXDRV_IOC_MAXNR) return -ENOTTY;

	//bei uns ist arg ein Pointer, und testen ob wir ihn nutzen dürfen (richtung aus UserSicht)
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;


	/* jetzt die CMDs auswerten */
	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&pDevData->DeviceSem) != 0)
		return -EINTR;

//----------------------------->
	//hier wird das CMD ausgeführt
	ret = Locked_ioctl(pDevData, cmd, (u8 __user *) arg, _IOC_SIZE(cmd) );
//<-----------------------------
	up(&pDevData->DeviceSem);


	return ret;
}

ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	u32 BytesToWrite, DeviceID;
	long res;
	unsigned long jiffiesTimeOut;
	s8 Index=-1;
	PDEVICE_DATA pDevData = NULL;

	/*Wie es geht:
	 * 1. (locked)
	 * 	- gibt es noch einen freien Platz in der LongTerm Liste, ja -> beide Flags setzen, sem auf unfrei setzen
	 * 	--> boIsInProcessUse <> es ist eine Anfrage im FPGA
	 * 	--> process wartet <> ein process is waiting for the sem
	 * 2. (locked)
	 * 	- ein write machen (2 DWORD im buffer gibt die anz BytesToWrite an)
	 * 3.
	 *  - auf die sem warten, wenn sem fehler, process flag löschen aber eintrag bleibt gültig
	 * (4.) async
	 *  - IRQ, von uns -> tasklet starten
	 *  - tasklet,
	 *  --> paket auslesen(immer),
	 *  --> gibt es einen gültigen LongTerm Eintrag ja, daten dort einfüllen, flag löschen
	 *  --> wenn auf die sem gewarted wird, aufwecken
	 * 5.
	 *	- Antwort in UserBuf füllen
	 *	- beide flags löschen
	 */

	pr_devel(MODDEBUGOUTTEXT" read (%d Bytes)\n", (int)count);

	/* Alles gut? */
	if( (filp==NULL) || (filp->private_data==NULL) ) return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;
	//mem ok?
	if( count > pDevData->BAR0SizeBytes)
		return -EFBIG;
	if(pDevData->pVABAR0 == NULL)
		return -EFBIG;
	if(pDevData->boIsIRQOpen == FALSE)
		return -EFAULT;
	if(count < (2*4))		//min 2 DWords, 2. is the size
		return -EFAULT;

	//dürfen wir den mem nutzen?
	if( !access_ok(VERIFY_WRITE, buf, count) )
		return -EFAULT;


	/* freien eintrag suchen */
	//0. DWord <> DevID
	//1. DWord <> anzBytesToWrite
	//2. DWord <> Header0
	//3. DWord <> Header1
	//4. DWord <> Data
	//5..n. DWord <> DummyDWords
	//args lesen
	if( get_user(DeviceID, buf) != 0)
		return -EFAULT;
	if( get_user(BytesToWrite, buf+4) != 0)
		return -EFAULT;
	if( (BytesToWrite+2*4) > count)
		return -EFBIG;

	pr_devel(MODDEBUGOUTTEXT" read, devID %d, BytesToWrite %d\n",DeviceID,BytesToWrite );

	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&pDevData->DeviceSem) != 0)
		return -EINTR;
//----------------------------->
	//freien eintrag suchen
	Index = -1;
	res = Locked_startlongtermread(pDevData, DeviceID);

	/* wenn ok jetzt ins FPGA schreiben */
	if(res >= 0){
		Index = (s8)res;
		res =  Locked_write(pDevData, buf+2*4, BytesToWrite);
	}
//<-----------------------------
	up(&pDevData->DeviceSem);

	if(Index <0 || Index > MAX_LONG_TERM_IO_REQUEST)	//nur um ganz sicher zu sein
		return -EFAULT;
	if(res < 0)
		goto EXIT_READ;

	/* warten auf Antwort */
	//if( down_killable(&LongTermRequestList[Index].WaitSem) != 0){
	jiffiesTimeOut = msecs_to_jiffies(1*1000);
	if( down_timeout(&pDevData->LongTermRequestList[Index].WaitSem,jiffiesTimeOut) != 0){
		res = -EINTR;
		goto EXIT_READ;
	}


	/* daten copy */
	//zu viel für den user buf bzw. gültig?
	if( 	(pDevData->LongTermRequestList[Index].IRQBuffer_anzBytes > count)
		||	(pDevData->LongTermRequestList[Index].IRQBuffer_anzBytes > MAX_SUNPACKETSIZE)
		||	(pDevData->LongTermRequestList[Index].IRQBuffer_anzBytes == 0) ){
		res = -EFBIG;
		goto EXIT_READ;
	}
	//copy
	if( copy_to_user(buf, pDevData->LongTermRequestList[Index].IRQBuffer, pDevData->LongTermRequestList[Index].IRQBuffer_anzBytes ) != 0 ){
		res = -EFAULT;
		goto EXIT_READ;
	}
	else
		res = pDevData->LongTermRequestList[Index].IRQBuffer_anzBytes;



	//gibt den eintrag frei
EXIT_READ:
	pDevData->LongTermRequestList[Index].boIsInProcessUse = FALSE;
	return res;
}

ssize_t AGEXDrv_write (struct file *filp, const char __user *buf, size_t count,loff_t *pos)
{
	long res;
	PDEVICE_DATA pDevData = NULL;

	pr_devel(MODDEBUGOUTTEXT" write (%d Bytes)\n", (int)count);

	/* Alles gut? */
	if( (filp==NULL) || (filp->private_data==NULL) ) return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;
	//mem ok?
	if(count > pDevData->BAR0SizeBytes)
		return -EFBIG;
	if(pDevData->pVABAR0 == NULL)
		return -EFBIG;

	//dürfen wir den mem nutzen?
	if( !access_ok(VERIFY_READ, buf, count) )
		return -EFAULT;

	/* jetzt kommt das schreiben */
	//warten (für immer auf die sem) wenn das prog abgeschossen wird, kommt sie zurück mit -EINTR
	if( down_killable(&pDevData->DeviceSem) != 0)
		return -EINTR;
//----------------------------->
	res =  Locked_write(pDevData, buf, count);
//<-----------------------------
		up(&pDevData->DeviceSem);

	return res;
}


