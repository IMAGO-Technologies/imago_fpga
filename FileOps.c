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

	get_device(_ModuleData.dev_data[iMinor]->sub_dev);

	return 0;
}

static int imago_release(struct inode *inode, struct file *filp)
{
	PDEVICE_DATA pDevData = NULL;
	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;

	pDevData = (PDEVICE_DATA) filp->private_data;

	// dev_info(pDevData->dev, "imago_release()\n");
	put_device(pDevData->sub_dev);

	return 0;
}

//Note: alte ioctl war unter "big kernel lock"
//http://lwn.net/Articles/119652/
static long imago_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	PDEVICE_DATA pDevData = NULL;
	void __user *pUser = (void __user *)arg;

	if (filp == NULL || filp->private_data == NULL)
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;

	if (!pDevData->boIsDeviceOpen) {
		dev_warn(pDevData->dev, "imago_unlocked_ioctl(): device is not ready\n");
		return -ENODEV;
	}

	if (_IOC_TYPE(cmd) != IMAGO_IOC_MAGIC)
		return -ENOTTY;

	if ((_IOC_DIR(cmd) & _IOC_READ) && !__access_ok__(VERIFY_WRITE, pUser, _IOC_SIZE(cmd)))
		return -EFAULT;
	if ((_IOC_DIR(cmd) & _IOC_WRITE) && !__access_ok__(VERIFY_READ, pUser, _IOC_SIZE(cmd)))
		return -EFAULT;

	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		dev_dbg(pDevData->dev, "imago_unlocked_ioctl(): down_interruptible() failed\n");
		// the operation can be restarted by the kernel
		return -ERESTARTSYS;
	}

	ret = imago_locked_ioctl(pDevData, cmd, pUser);

	up(&pDevData->DeviceSem);


	return ret;
}

int imago_read_internal(PDEVICE_DATA pDevData, u32* buf, unsigned int count)
{
	struct SUN_DEVICE_DATA* pSunDevice;
	u32 DeviceID;
	u32 BytesToWrite;
	u32 TimeOut_ms;
	u32 restart = 0;
	int toggleId = 0;
	int res;
	unsigned long flags;
	unsigned long timeout, t_start;
	
	//0. DWord <> DevID
	//1. DWord <> BytesToWrite
	//2. DWord <> TimeOut_ms (-1 <> für immer)
	//3. DWord <> Header0
	//4. DWord <> Header1
	//5. DWord <> Data
	//6. DWord <> DummyDWord

	if (count < (3 * 4))
		return -EINVAL;

	// DeviceID: strip serialID (bit 6)
	DeviceID = buf[0] & (MAX_IRQDEVICECOUNT - 1);
	BytesToWrite = buf[1];
	TimeOut_ms = buf[2];
	
	pSunDevice = &pDevData->SunDeviceData[DeviceID];

	if (BytesToWrite % 4)
		return -EINVAL;
	if (BytesToWrite == 0) {
		restart = 1;
	}
	else {
		if (count < (BytesToWrite + 3 * 4))
			return -EFBIG;
		// check for restart read by user space:
		if (count >= (BytesToWrite + 4 * 4))
			restart = buf[BytesToWrite / 4 + 3];
	}

	if (restart) {
		dev_dbg(pDevData->dev, "imago_read(): restart wait for completion for DeviceID %d\n", DeviceID);
	}
	else {
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

		res = imago_write_locked(pDevData, buf + 3, BytesToWrite / 4);

		up(&pDevData->DeviceSem);
		if (res < 0)
			return res;

		dev_dbg(pDevData->dev, "imago_read() > wait for completion for DeviceID %d\n", DeviceID);
	}

	/* wait for completion */
	timeout = TimeOut_ms == 0xFFFFFFFF ? MAX_SCHEDULE_TIMEOUT : msecs_to_jiffies(TimeOut_ms);
	t_start = jiffies;
	res = wait_for_completion_interruptible_timeout(&pSunDevice->result_complete, timeout);

	if (res == 0) {
		dev_dbg(pDevData->dev, "imago_read(): completion timeout for DeviceID: %u\n", DeviceID);
		return -ETIME;
	}
	if (res == -ERESTARTSYS) {
		dev_dbg(pDevData->dev, "imago_read(): waiting for completion was interrupted for DeviceID: %u\n", DeviceID);

		// ERESTART_RESTARTBLOCK does not work on ARM => return -ERESTARTSYS and
		// set write size in the buffer to 0 for the restarted call by the kernel:
		buf[1] = 0;

		if (TimeOut_ms != 0xFFFFFFFF && TimeOut_ms != 0) {
			// also update the timeout value:
			unsigned long timeout_done = jiffies_to_msecs(jiffies - t_start);
			if (timeout_done < TimeOut_ms)
				TimeOut_ms -= timeout_done;
			else
				TimeOut_ms = 0;
			dev_dbg(pDevData->dev, "imago_read(): timeout remaining: %u ms\n", TimeOut_ms);
			buf[2] = TimeOut_ms;
		}

		return res;
	}

	// check for abort by user?
	raw_spin_lock_irqsave(&pDevData->lock, flags);
	if (pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
		// request was canceled
		// do not toggle serial ID yet, the request may still be answered by the FPGA
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
	memcpy(buf, pSunDevice->packet, 3 * 4);
	pSunDevice->requestState = SUN_REQ_STATE_IDLE;
	raw_spin_unlock_irqrestore(&pDevData->lock, flags);

	return 3 * 4;
}

static ssize_t imago_read(struct file* filp, char __user* buf, size_t count, loff_t* pos)
{
	PDEVICE_DATA pDevData = NULL;
	int ret;
	u32 data[7];

	if (filp == NULL || filp->private_data == NULL || count > sizeof(data))
		return -EINVAL;

	pDevData = (PDEVICE_DATA)filp->private_data;
	if (!pDevData->boIsDeviceOpen) {
		dev_warn(pDevData->dev, "imago_read(): device is not ready\n");
		return -ENODEV;
	}

	// dev_dbg(pDevData->dev, "imago_read(): %d bytes\n", (int)count);

	if (copy_from_user(data, buf, count) != 0){
		return -EFAULT;
	}

	ret = imago_read_internal(pDevData, data, count);
	if (ret == 3 * 4) {
		if (copy_to_user(buf, data, 3 * 4) != 0) {
			return -EFAULT;
		}
	}
	else if (ret == -ERESTARTSYS) {
		// store modified write size and timeout in user memory
		put_user(data[1], (u32*)(buf + 1));
		put_user(data[2], (u32*)(buf + 2));
	}
	return ret;
}

int imago_write_locked(PDEVICE_DATA pDevData, u32* packet, unsigned int packet_size)
{
	// insert serialID to Header1:
	u8 deviceID = (packet[1] >> 20) & (MAX_IRQDEVICECOUNT - 1);
	if (deviceID != 0) {
		packet[1] &= ~(1 << 26);
		packet[1] |= pDevData->SunDeviceData[deviceID].serialID << 26;
	}

	return pDevData->write(pDevData, packet, packet_size);
}

int imago_write_internal(PDEVICE_DATA pDevData, u32* packet, unsigned int packet_size)
{
	int res;

	// lock FPGA access, may be interrupted and restarted by the kernel
	if (down_interruptible(&pDevData->DeviceSem) != 0) {
		dev_dbg(pDevData->dev, "imago_write_internal(): down_interruptible() timeout\n");
		return -ERESTARTSYS;
	}

	res = imago_write_locked(pDevData, packet, packet_size);

	up(&pDevData->DeviceSem);

	return res;
}

static ssize_t imago_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	int res;
	PDEVICE_DATA pDevData = NULL;
	u32 packet[4];

	if (filp == NULL || filp->private_data == NULL || (count % 4) != 0 || count > sizeof(packet))
		return -EINVAL;
	pDevData = (PDEVICE_DATA) filp->private_data;

	if (!pDevData->boIsDeviceOpen) {
		dev_warn(pDevData->dev, "imago_write(): device is not ready\n");
		return -ENODEV;
	}

	dev_dbg(pDevData->dev, "imago_write(): %d bytes\n", (int)count);

	if (copy_from_user(packet, buf, count) != 0) {
		dev_warn(pDevData->dev, "imago_write(): copy_from_user() failed\n");
		return -EFAULT;
	}

	res = imago_write_internal(pDevData, packet, count / 4);
	return res;
}



struct file_operations fpga_ops = {
	.owner = THIS_MODULE,
	.open = imago_open,
	.release = imago_release,
	.read = imago_read,
	.write = imago_write,
	.unlocked_ioctl = imago_unlocked_ioctl,
	.llseek = no_llseek,
};

