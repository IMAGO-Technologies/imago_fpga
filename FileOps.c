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
	if (node == NULL || filp == NULL)
		return -EINVAL; 
	iMinor = iminor(node);

	pr_devel(MODDEBUGOUTTEXT" open (Minor:%d)\n", iMinor);

	//setzt ins "file" den Context
	if (iMinor >= MAX_DEVICE_COUNT)
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
	pr_devel(MODDEBUGOUTTEXT" ioctl (CMD %d, MAGIC %c, size %d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd));

	/* Alles gut */
	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;
	//ist ist das CMD eins für uns?
	if (_IOC_TYPE(cmd) != AGEXDRV_IOC_MAGIC)
		return -ENOTTY;

	//bei uns ist arg ein Pointer, und testen ob wir ihn nutzen dÃ¼rfen (richtung aus UserSicht)
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;


	/* jetzt die CMDs auswerten */
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		pr_devel(MODDEBUGOUTTEXT" AGEXDrv_unlocked_ioctl(), down_interruptible() failed!\n");
		return -ERESTARTSYS;
	}
//----------------------------->
	//hier wird das CMD ausgeführt
	ret = Locked_ioctl(pDevData, cmd, (u8 __user *) arg, _IOC_SIZE(cmd) );
//<-----------------------------
	up(&pDevData->DeviceSem);


	return ret;
}

ssize_t AGEXDrv_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	u32 TimeOut_ms, BytesToWrite, DeviceID;
	long res;	
	PDEVICE_DATA pDevData = NULL;
	struct SUN_DEVICE_DATA *pSunDevice;
	unsigned long flags;
	int toggleId = 0;
	u32 restart = 0;

	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;

	dev_dbg(pDevData->dev, "AGEXDrv_read() > %d Bytes\n", (int)count);

	//mem ok?
	if (pDevData->boIsIRQOpen == FALSE)
		return -EFAULT;
	if (count < (3 * 4))
		return -EFAULT;

	//duerfen wir den mem nutzen?
	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;


	/* freien eintrag suchen */
	//0. DWord <> DevID
	//1. DWord <> anzBytesToWrite
	//2. DWord <> TimeOut_ms (-1 <> fÃ¼r immer)
	//3. DWord <> Header0
	//4. DWord <> Header1
	//5. DWord <> Data
	//6..n. DWord <> DummyDWords
	//args lesen
	if( get_user(DeviceID, (u32*)buf) != 0)
		return -EFAULT;
	// strip serialID (bit 6)
	DeviceID &= (MAX_IRQDEVICECOUNT - 1);
	if (get_user(BytesToWrite, (u32*)(buf+1*4)) != 0)
		return -EFAULT;
	if (get_user(TimeOut_ms, (u32*)(buf+2*4)) != 0)
		return -EFAULT;	
	if (count < (BytesToWrite+3*4))
		return -EFBIG;
	if (count >= (BytesToWrite+4*4))
		get_user(restart, (u32*)(buf+BytesToWrite+3*4));

	dev_dbg(pDevData->dev, "AGEXDrv_read() > DeviceID %d, BytesToWrite %d, TimeOut %u\n", DeviceID, BytesToWrite, TimeOut_ms);

	pSunDevice = &pDevData->SunDeviceData[DeviceID];

	if (restart) {
		dev_dbg(pDevData->dev, "AGEXDrv_read(): restart wait for completion for DeviceID %d\n", DeviceID);
	} else {
		//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
		//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen	
		if (down_interruptible(&pDevData->DeviceSem) != 0) {
			dev_dbg(pDevData->dev, "AGEXDrv_read() > down_interruptible('DeviceSem') failed!\n");
			return -ERESTARTSYS;
		}

		spin_lock_irqsave(&pDevData->lock, flags);
		if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA ||
			pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
			// Pending request is still in FPGA (after timeout or abort) => toggle serial ID
			pSunDevice->serialID = !pSunDevice->serialID;
			toggleId = 1;
		}
		pSunDevice->requestState = SUN_REQ_STATE_INFPGA;
		spin_unlock_irqrestore(&pDevData->lock, flags);

		if (toggleId)
			dev_warn(pDevData->dev, "AGEXDrv_read() > pending FPGA request for DeviceID %u, toggling serial ID -> %u\n", DeviceID, pSunDevice->serialID);

		// clear pending completions
		while (try_wait_for_completion(&pSunDevice->result_complete)) {
			// may happen if a previous read() that timed out was answered in the meantime
			dev_dbg(pDevData->dev, "AGEXDrv_read(): clearing old completion for DeviceID %u\n", DeviceID);
		}

		res = Locked_write(pDevData, buf+3*4, BytesToWrite);
		up(&pDevData->DeviceSem);
		if (res < 0)
			return res;

		dev_dbg(pDevData->dev, "AGEXDrv_read() > wait for completion for DeviceID %d\n", DeviceID);
	}

	/* wait for completion */
	if (TimeOut_ms == 0xFFFFFFFF) {
		res = wait_for_completion_interruptible(&pSunDevice->result_complete);
	} else {
		unsigned long jiffies = msecs_to_jiffies(TimeOut_ms);
		res = wait_for_completion_interruptible_timeout(&pSunDevice->result_complete, jiffies);
		if (res == 0) {
			dev_dbg(pDevData->dev, "AGEXDrv_read(): completion timeout for DeviceID: %u\n", DeviceID);
			return -ETIME;
		}
	}
	if (res == -ERESTARTSYS) {
		dev_dbg(pDevData->dev, "AGEXDrv_read(): waiting for completion was interrupted for DeviceID: %u\n", DeviceID);
		// restart must be handled in user space => we return -EAGAIN instead of -ERESTARTSYS:
		return -EAGAIN;
	}


	// check for abort by user?
	spin_lock_irqsave(&pDevData->lock, flags);
	if (pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
		// request was canceled
		// do not toggle serial ID yet, request may still be answered by the FPGA
//		pSunDevice->serialID = !pSunDevice->serialID;
//		pSunDevice->requestState = SUN_REQ_STATE_IDLE;
		spin_unlock_irqrestore(&pDevData->lock, flags);
		dev_dbg(pDevData->dev, "AGEXDrv_read() > aborting read for DeviceID: %u\n", DeviceID);
		return -EINTR;
	}
	else if (pSunDevice->requestState != SUN_REQ_STATE_RESULT) {
		spin_unlock_irqrestore(&pDevData->lock, flags);
		dev_warn(pDevData->dev, "AGEXDrv_read() > unexpected request state (%u)\n", pSunDevice->requestState);
		return -EFAULT;
	}

	if (copy_to_user(buf, pSunDevice->packet, 3*4) != 0 ) {
		return -EFAULT;
	}

	pSunDevice->requestState = SUN_REQ_STATE_IDLE;
	spin_unlock_irqrestore(&pDevData->lock, flags);

	return 3*4;
}

ssize_t AGEXDrv_write(struct file *filp, const char __user *buf, size_t count,loff_t *pos)
{
	long res;
	PDEVICE_DATA pDevData = NULL;

	/* Alles gut? */
	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;

	dev_dbg(pDevData->dev, "AGEXDrv_write() > %d bytes\n", (int)count);

	// is mem access OK?
	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	// write data
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		dev_dbg(pDevData->dev, "AGEXDrv_write() > down_interruptible('DeviceSem') failed!\n");
		return -ERESTARTSYS;
	}
//----------------------------->
	res = Locked_write(pDevData, buf, count);
//<-----------------------------
	up(&pDevData->DeviceSem);

	return res;
}


