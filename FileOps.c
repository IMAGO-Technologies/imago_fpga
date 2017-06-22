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
	//ist ist das CMD eins f�r uns?
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
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if( down_interruptible(&pDevData->DeviceSem) != 0)
		{pr_devel(MODDEBUGOUTTEXT" AGEXDrv_unlocked_ioctl(), down_interruptible() failed!\n"); return -ERESTARTSYS;}
//----------------------------->
	//hier wird das CMD ausgef�hrt
	ret = Locked_ioctl(pDevData, cmd, (u8 __user *) arg, _IOC_SIZE(cmd) );
//<-----------------------------
	up(&pDevData->DeviceSem);


	return ret;
}

ssize_t AGEXDrv_read (struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	u32 TimeOut_ms, BytesToWrite, DeviceID;
	long res;	
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
	//2. DWord <> TimeOut_ms (-1 <> für immer)
	//3. DWord <> Header0
	//4. DWord <> Header1
	//5. DWord <> Data
	//6..n. DWord <> DummyDWords
	//args lesen
	if( get_user(DeviceID, (u32*)buf) != 0)
		return -EFAULT;
	if( get_user(BytesToWrite, (u32*)(buf+1*4) ) != 0)
		return -EFAULT;
	if( get_user(TimeOut_ms, (u32*)(buf+2*4) ) != 0)
		return -EFAULT;	
	if( (BytesToWrite+3*4) > count)
		return -EFBIG;

	pr_devel(MODDEBUGOUTTEXT" read, devID %d, BytesToWrite %d, TimeOut 0x%x\n",DeviceID,BytesToWrite,TimeOut_ms);

	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen	
	if( down_interruptible(&pDevData->DeviceSem) != 0)	
		{pr_devel(MODDEBUGOUTTEXT" AGEXDrv_read():down_interruptible('DeviceSem') failed!\n"); return -ERESTARTSYS;}
//----------------------------->
	//freien eintrag suchen
	Index = -1;
	res = Locked_startlongtermread(pDevData, DeviceID);

	/* wenn ok jetzt ins FPGA schreiben */
	if(res >= 0){
		Index = (s8)res;
		res =  Locked_write(pDevData, buf+3*4, BytesToWrite);
	}
//<-----------------------------
	up(&pDevData->DeviceSem);

	if(res < 0){
		printk(KERN_WARNING MODDEBUGOUTTEXT" read, Locked_startlongtermread() failed (%ld)!\n", res);
		goto EXIT_READ;
	}
	if(Index <0 || Index > MAX_LONG_TERM_IO_REQUEST)	//nur um ganz sicher zu sein
		return -EFAULT;
	pr_devel(MODDEBUGOUTTEXT" read, wait for request %d\n", Index);


	/* warten auf Antwort */
	// aufwachen durch Signal, up vom SWI, oder User [Abort], bzw TimeOut
	if( TimeOut_ms == 0xFFFFFFFF )
	{
		//Note: nicht unterbrechbar(durch gdb), abbrechbar nur durch kill -9, nicht kill -15(term) 
		if( down_killable(&pDevData->LongTermRequestList[Index].WaitSem) != 0)
			{res = -EINTR; printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_read():down_killable('LongTermRequest') failed!\n"); goto EXIT_READ;}
	}
	else
	{
		//Note: nicht unter(durch GDB) bzw. abbrechbar down_timeout() > __down_timeout() > __down_common(sem, TASK_UNINTERRUPTIBLE, timeout);
		unsigned long jiffiesTimeOut = msecs_to_jiffies(TimeOut_ms);
		int waitRes = down_timeout(&pDevData->LongTermRequestList[Index].WaitSem,jiffiesTimeOut);
		if( waitRes == (-ETIME))
			{res = -ETIME; pr_devel(MODDEBUGOUTTEXT" AGEXDrv_read> down_timeout(), timeout!\n"); goto EXIT_READ;}
		else if ( waitRes != 0)
			{res = -EINTR; printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_read> down_timeout(), failed!\n"); goto EXIT_READ;}
	}

	//abort durch user?
	if(pDevData->LongTermRequestList[Index].boAbortWaiting){
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
	pr_devel(MODDEBUGOUTTEXT" read, result %ld\n", res);
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
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if( down_interruptible(&pDevData->DeviceSem) != 0)
		{pr_devel(MODDEBUGOUTTEXT" AGEXDrv_write():down_interruptible('DeviceSem') failed!\n"); return -ERESTARTSYS;}
//----------------------------->
	res = Locked_write(pDevData, buf, count);
//<-----------------------------
		up(&pDevData->DeviceSem);

	return res;
}


