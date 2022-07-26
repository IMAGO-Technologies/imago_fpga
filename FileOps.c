/*
 * File operations
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "imago_fpga.h"


#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
#define __access_ok__(type, addr, size) access_ok(type, addr, size)
#else
#define __access_ok__(type, addr, size) access_ok(addr, size)
#endif


static int imago_open(struct inode *node, struct file *filp)
{
	int iMinor;
	if (node == NULL || filp == NULL)
		return -EINVAL; 
	iMinor = iminor(node);

	pr_devel(MODMODULENAME": open (Minor: %d)\n", iMinor);

	//setzt ins "file" den Context
	if (iMinor >= MAX_DEVICE_COUNT)
		return -EINVAL;
	filp->private_data = _ModuleData.dev_data[iMinor];

	return 0;
}

//Note: alte ioctl war unter "big kernel lock"
//http://lwn.net/Articles/119652/
static long imago_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	PDEVICE_DATA pDevData = NULL;
	void __user *pUser = (void __user *)arg;

	//Note: beim 1. write bekommen wir ein CMD:1, Type 'T', Size: 0 <-> FIONBIO (linux/include/asm-i386/ioctls.h, line 41)
	pr_devel(MODMODULENAME": ioctl (CMD %d, MAGIC %c, size %d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd));
	/* Alles gut */
	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;
	//ist ist das CMD eins für uns?
	if (_IOC_TYPE(cmd) != IMAGO_IOC_MAGIC)
		return -ENOTTY;

	if ((_IOC_DIR(cmd) & _IOC_READ) && !__access_ok__(VERIFY_WRITE, pUser, _IOC_SIZE(cmd)))
		return -EFAULT;
	if ((_IOC_DIR(cmd) & _IOC_WRITE) && !__access_ok__(VERIFY_READ, pUser, _IOC_SIZE(cmd)))
		return -EFAULT;

	/* jetzt die CMDs auswerten */
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		dev_dbg(pDevData->dev, "imago_unlocked_ioctl(): down_interruptible() failed\n");
		return -ERESTARTSYS;
	}
//----------------------------->
	//hier wird das CMD ausgeführt
	ret = imago_locked_ioctl(pDevData, cmd, pUser);
//<-----------------------------
	up(&pDevData->DeviceSem);


	return ret;
}

static ssize_t imago_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
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

	dev_dbg(pDevData->dev, "imago_read() > %d Bytes\n", (int)count);

	//mem ok?
	if (!pDevData->boIsIRQOpen)
		return -EFAULT;
	if (count < (3 * 4))
		return -EFAULT;

	//duerfen wir den mem nutzen?
	if (!__access_ok__(VERIFY_WRITE, buf, count))
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

	dev_dbg(pDevData->dev, "imago_read() > DeviceID %d, BytesToWrite %d, TimeOut %u\n", DeviceID, BytesToWrite, TimeOut_ms);

	pSunDevice = &pDevData->SunDeviceData[DeviceID];

	if (restart) {
		dev_dbg(pDevData->dev, "imago_read(): restart wait for completion for DeviceID %d\n", DeviceID);
	} else {
		//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
		//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen	
		if (down_interruptible(&pDevData->DeviceSem) != 0) {
			dev_dbg(pDevData->dev, "imago_read() > down_interruptible('DeviceSem') failed!\n");
			return -ERESTARTSYS;
		}

		raw_spin_lock_irqsave(&pDevData->lock, flags);
		if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA ||
			pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
			// Pending request is still in FPGA (after timeout or abort) => toggle serial ID
			pSunDevice->serialID = !pSunDevice->serialID;
			toggleId = 1;
		}
		pSunDevice->requestState = SUN_REQ_STATE_INFPGA;
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);

		if (toggleId)
			dev_warn(pDevData->dev, "imago_read() > pending FPGA request for DeviceID %u, toggling serial ID -> %u\n", DeviceID, pSunDevice->serialID);

		// clear pending completions
		while (try_wait_for_completion(&pSunDevice->result_complete)) {
			// may happen if a previous read() that timed out was answered in the meantime
			dev_dbg(pDevData->dev, "imago_read(): clearing old completion for DeviceID %u\n", DeviceID);
		}

		res = pDevData->write(pDevData, buf+3*4, BytesToWrite);
		up(&pDevData->DeviceSem);
		if (res < 0)
			return res;

		dev_dbg(pDevData->dev, "imago_read() > wait for completion for DeviceID %d\n", DeviceID);
	}

	/* wait for completion */
	if (TimeOut_ms == 0xFFFFFFFF) {
		res = wait_for_completion_interruptible(&pSunDevice->result_complete);
	} else {
		unsigned long jiffies = msecs_to_jiffies(TimeOut_ms);
		res = wait_for_completion_interruptible_timeout(&pSunDevice->result_complete, jiffies);
		if (res == 0) {
			dev_dbg(pDevData->dev, "imago_read(): completion timeout for DeviceID: %u\n", DeviceID);
			return -ETIME;
		}
	}
	if (res == -ERESTARTSYS) {
		dev_dbg(pDevData->dev, "imago_read(): waiting for completion was interrupted for DeviceID: %u\n", DeviceID);
		// restart must be handled in user space => we return -EAGAIN instead of -ERESTARTSYS:
		return -EAGAIN;
	}


	// check for abort by user?
	raw_spin_lock_irqsave(&pDevData->lock, flags);
	if (pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
		// request was canceled
		// do not toggle serial ID yet, request may still be answered by the FPGA
//		pSunDevice->serialID = !pSunDevice->serialID;
//		pSunDevice->requestState = SUN_REQ_STATE_IDLE;
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
		dev_dbg(pDevData->dev, "imago_read() > aborting read for DeviceID: %u\n", DeviceID);
		return -EINTR;
	}
	else if (pSunDevice->requestState != SUN_REQ_STATE_RESULT) {
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
		dev_warn(pDevData->dev, "imago_read() > unexpected request state (%u)\n", pSunDevice->requestState);
		return -EFAULT;
	}

	if (copy_to_user(buf, pSunDevice->packet, 3*4) != 0 ) {
		return -EFAULT;
	}

	pSunDevice->requestState = SUN_REQ_STATE_IDLE;
	raw_spin_unlock_irqrestore(&pDevData->lock, flags);

	return 3*4;
}

static ssize_t imago_write(struct file *filp, const char __user *buf, size_t count,loff_t *pos)
{
	long res;
	PDEVICE_DATA pDevData = NULL;

	/* Alles gut? */
	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;

	dev_dbg(pDevData->dev, "imago_write() > %d bytes\n", (int)count);

	// is mem access OK?
	if (!__access_ok__(VERIFY_READ, buf, count))
		return -EFAULT;

	// write data
	//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
	//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen
	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		dev_dbg(pDevData->dev, "imago_write() > down_interruptible('DeviceSem') failed!\n");
		return -ERESTARTSYS;
	}
//----------------------------->
	res = pDevData->write(pDevData, buf, count);
//<-----------------------------
	up(&pDevData->DeviceSem);

	return res;
}


static struct file_operations fpga_ops = {
	.owner = THIS_MODULE,
	.open = imago_open,
	.read = imago_read,
	.write = imago_write,
	.unlocked_ioctl = imago_unlocked_ioctl,
	.llseek = no_llseek,
};


// Helper function for creating a char device
void imago_create_device(PDEVICE_DATA pDevData)
{
	int res;

	cdev_init(&pDevData->DeviceCDev, &fpga_ops);
	pDevData->DeviceCDev.owner = THIS_MODULE;
	pDevData->DeviceCDev.ops 	= &fpga_ops;	//notwendig in den quellen wird fops gesetzt?

	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
	res = cdev_add(&pDevData->DeviceCDev, pDevData->DeviceNumber, 1/*wie viele ab startNum*/);
	if (res < 0)
		dev_warn(pDevData->dev, "cdev_add() failed\n");
	else
		pDevData->boIsDeviceOpen = true;


	//> in Sysfs class eintragen
	/**********************************************************************/			
	//war mal class_device_create
	if (!IS_ERR(_ModuleData.pModuleClass)) {		
		// char devName[128];
		struct device *dev;

		// sprintf(devName, "%s%d", MODMODULENAME, MINOR(pDevData->DeviceNumber));
		dev = device_create(
				_ModuleData.pModuleClass, 	/* die Type classe */
				NULL, 			/* pointer zum Eltern, dann wird das dev ein Kind vom parten*/
				pDevData->DeviceNumber, /* die nummer zum device */
				NULL,
				MODMODULENAME"%d", MINOR(pDevData->DeviceNumber)
				);

		if (IS_ERR(dev))
			dev_warn(pDevData->dev, "error creating sysfs device (%ld)\n", PTR_ERR(dev));
	}
}

