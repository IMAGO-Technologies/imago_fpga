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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "imago_fpga.h"

typedef u8 IOCTLBUFFER[128];

// direction is from user space's view
#define IOC_GET_VERSION 				_IOR(IMAGO_IOC_MAGIC, 0, IOCTLBUFFER)
#define IOC_GET_BUILD_DATE 				_IOR(IMAGO_IOC_MAGIC, 1, IOCTLBUFFER)
#define IOC_RELEASE_DEVICEID 			_IOWR(IMAGO_IOC_MAGIC, 2, u8)
#define IOC_CREATE_DEVICEID 			_IOR(IMAGO_IOC_MAGIC, 3, u8)
#define IOC_GET_SUBTYPE 				_IOR(IMAGO_IOC_MAGIC, 4, u8)
#define IOC_ABORT_LONGTERM_READ 		_IOWR(IMAGO_IOC_MAGIC, 5, u8)
#define IOC_DMAREAD_CONFIG				_IOW(IMAGO_IOC_MAGIC, 6, IOCTLBUFFER)
#define IOC_DMAREAD_ADD_BUFFER			_IOW(IMAGO_IOC_MAGIC, 7, IOCTLBUFFER)
#define IOC_DMAREAD_WAIT_FOR_BUFFER		_IOWR(IMAGO_IOC_MAGIC, 8, IOCTLBUFFER)
#define IOC_DMAREAD_WAIT_FOR_BUFFER_TS	_IOC(_IOC_READ|_IOC_WRITE, IMAGO_IOC_MAGIC, 8, 19)
#define IOC_DMAREAD_ABORT_DMA			_IOW(IMAGO_IOC_MAGIC, 9, u8)
#define IOC_DMAREAD_ABORT_WAITER		_IOW(IMAGO_IOC_MAGIC, 10, u8)
#define IOC_DMAREAD_RESETCHANNEL		_IOW(IMAGO_IOC_MAGIC, 11, u8)
#define IOC_DMAREAD_MAP_BUFFER			_IOWR(IMAGO_IOC_MAGIC, 12, IOCTLBUFFER)
#define IOC_DMAREAD_UNMAP_BUFFER		_IOW(IMAGO_IOC_MAGIC, 13, IOCTLBUFFER)
#define IOC_DMAREAD_ADD_MAPPED_BUFFER	_IOW(IMAGO_IOC_MAGIC, 14, IOCTLBUFFER)
#define IOC_INIT_I2C_ADAPTER			_IOW(IMAGO_IOC_MAGIC, 15, u8)

long imago_locked_ioctl(PDEVICE_DATA pDevData, u32 cmd, u8 __user * pToUserMem)
{
	unsigned long flags;

	switch (cmd)
	{
		/* Gibt die Version als String zurück */
		/**********************************************************************/
		case IOC_GET_VERSION:
			if (copy_to_user(pToUserMem, MODVERSION, sizeof(MODVERSION)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> copy_to_user faild\n");
				return -EFAULT;
			}
			return sizeof(MODVERSION);


		/* Gibt das Build date/time als String zurück */
		/**********************************************************************/
		case IOC_GET_BUILD_DATE:
			if (copy_to_user(pToUserMem, MODDATECODE, sizeof(MODDATECODE)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> copy_to_user faild\n");
				return -EFAULT;
			}
			return sizeof(MODDATECODE);


		/* Gibt SubType zurück */
		/**********************************************************************/
		case IOC_GET_SUBTYPE:
		{
			u8 device_type = pDevData->device_type;
			if (__put_user(device_type, pToUserMem) != 0) {
				dev_err(pDevData->dev, "Locked_ioctl > put_user() failed\n");
				return -EFAULT;
			}
			return sizeof(device_type);
		}


		/* Markiert die DeviceID als frei*/
		/**********************************************************************/
		//	-> kein Fehler wenn ungenutzt oder noch im LongTermRequest noch OutOfRange
		case IOC_RELEASE_DEVICEID:
		{
			u8 deviceID;

			if (__get_user(deviceID, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			return imago_release_deviceid(pDevData, deviceID);
		}


		/* Gibt eine neue DeviceID zurück */
		/**********************************************************************/
		// -> nur wenn sie zur Zeit als frei markiert ist boIsDeviceIDUsed[]==false
		// -> nur wenn sie nicht im LongTermRequestList[] ist
		// 		dass kann passieren wenn die DLL eine LongTermAnfrage startet und dann das device zu macht
		// 		die Anfrage ist dann noch unterwegs, ja der Request wurde abgebrochen aber die DeviceID ist
		// 		noch genutzt
		case IOC_CREATE_DEVICEID:
		{
			u8 deviceIdUser;
			long retVal;
			retVal = imago_create_deviceid(pDevData, &deviceIdUser);
			if (__put_user(deviceIdUser, pToUserMem) != 0) {
				dev_err(pDevData->dev, "Locked_ioctl > put_user() failed\n");
				return -EFAULT;
			}
			return retVal;
		}

		/* Wenn zur DeviceID ein LONGTERM_IOREQUEST läuft, sem posten. flag setzen*/
		/**********************************************************************/
		case IOC_ABORT_LONGTERM_READ:
		{
			u8 deviceID;

			//DevId lesen
			if (__get_user(deviceID, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			imago_abort_longterm_read(pDevData, deviceID);
			return 0;
		}


		/************************ DMA commands **********************/

		// Set number of DMA channels / Transfer Channels / SG elements which are present in the FPGA.
		// Values can only be modified when 0 (or set with the same value).
		case IOC_DMAREAD_CONFIG:
		{
			u16 tmpDMAs, tmpTCs, tmpSGs;
			u8 iDMA, iTC;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {				
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (__get_user(tmpDMAs, (u16*)(pToUserMem + 0)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(tmpTCs,(u16*)(pToUserMem + 2)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(tmpSGs,(u16*)(pToUserMem + 4)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}

			// reconfiguring with the same values is allowed
			if (	pDevData->DMARead_channels == tmpDMAs
				&&	pDevData->DMARead_TCs == tmpTCs
				&&	pDevData->DMARead_SGs == tmpSGs)
				return 0;

			// only allow configuration the first time
			if (	pDevData->DMARead_channels != 0
				|| 	pDevData->DMARead_TCs != 0
				|| 	pDevData->DMARead_SGs != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> Can't configure used DMA/TCs!\n");
				return -EFAULT;
			}
			//nicht ueber max setzen
			if ((tmpDMAs > MAX_DMA_CHANNELS) || (tmpTCs > MAX_DMA_READ_CHANNELTCS) || (tmpSGs > MAX_DMA_READ_TCSGS)) {
				dev_warn(pDevData->dev, "Locked_ioctl> ConfigDMARead anzDMA/TC/SG too large!\n");
				return -EFAULT;
			}

			pDevData->DMARead_channels	= tmpDMAs;
			pDevData->DMARead_TCs		= tmpTCs;
			pDevData->DMARead_SGs		= tmpSGs;

			// save FPGA address for writing into the descriptor FIFO for each TC
			for (iDMA = 0; iDMA < pDevData->DMARead_channels; iDMA++) {
				for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
					pDevData->DMARead_Channel[iDMA].TCs[iTC].pDesriptorFifo = pDevData->pVABAR0 
						+ DMA_READ_TC_SG_OFFSET
						+ (iDMA * pDevData->DMARead_TCs * DMA_READ_TC_TC2TC_SETPBYTES)
						+ iTC * DMA_READ_TC_TC2TC_SETPBYTES;
				}
			}

			dev_dbg(pDevData->dev, "ioctl Config DMA: cannels: %d, TCs: %d, SGs: %d\n",
					pDevData->DMARead_channels, pDevData->DMARead_TCs, pDevData->DMARead_SGs);
			return 0;
		}

		// reset DMA channel in case a transfer is pending or lost mapped buffers after a process was killed
		case IOC_DMAREAD_RESETCHANNEL:
		{
			u8 iDMAChannel;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!\n");
				return -EFAULT;
			}
			
			return imago_DMARead_Reset_DMAChannel(pDevData, iDMAChannel);
		}

		// map user buffer for DMA
		case IOC_DMAREAD_MAP_BUFFER:
		{
			u8 	iDMAChannel, reversePages;
			u64 UserPTR, bufferSize;
			DMA_READ_CHANNEL *pDMAChannel;
			DMA_READ_JOB *pJob = NULL;
			int result;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(UserPTR, (u64*)(pToUserMem + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(bufferSize, (u64*)(pToUserMem + sizeof(u64) + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(reversePages, (u8*)(pToUserMem + 2*sizeof(u64) + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
		
			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (bufferSize == 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> invalid buffer size!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pDMAChannel->doManualMap = true;

			// map buffer
			result = imago_DMARead_MapUserBuffer(pDevData, pDMAChannel, (uintptr_t)UserPTR, bufferSize, reversePages, &pJob);
			if (result < 0) {
				imago_DMARead_UnMapUserBuffer(pDevData, pJob);
				return result;
			}

			// return index of pJob in jobBuffers[] to user space
			if (__put_user((u64)(pJob - pDMAChannel->jobBuffers), (u64 *)pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> put_user failed\n");
				return -EFAULT;
			}

			return sizeof(u64);
		}

		// unmap user buffer
		case IOC_DMAREAD_UNMAP_BUFFER:
		{
			u8 iDMAChannel;
			u64 jobIndex;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(jobIndex, (u64*)(pToUserMem + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (jobIndex > _ModuleData.max_dma_buffers) {
				dev_warn(pDevData->dev, "Locked_ioctl> jobIndex is out of range!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pJob = &pDMAChannel->jobBuffers[jobIndex];

			imago_DMARead_UnMapUserBuffer(pDevData, pJob);
			
			return 0;
		}
		
		// add buffer for DMA (old version which also maps the buffer)
		case IOC_DMAREAD_ADD_BUFFER:
		{
			u8 iDMAChannel;
			u64 UserPTR, bufferSize;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob = NULL;
			int result;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(UserPTR, (u64*)(pToUserMem + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(bufferSize, (u64*)(pToUserMem + sizeof(u64) + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}

			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (bufferSize == 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> invalid buffer size!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pDMAChannel->doManualMap = false;

			// map user buffer for DMA
			result =  imago_DMARead_MapUserBuffer(pDevData, pDMAChannel, (uintptr_t) UserPTR, bufferSize, 0, &pJob);
			if (result < 0) {
				imago_DMARead_UnMapUserBuffer(pDevData, pJob);
				return result;
			}

			// start DMA if idle, else add job to Jobs_ToDo FIFO
			result = imago_DMARead_AddJob(pDevData, iDMAChannel, pJob);
			if (result != 0) {
				imago_DMARead_UnMapUserBuffer(pDevData, pJob);
				return -result;
			}

			return 0;
		}

		// add mapped buffer for DMA
		case IOC_DMAREAD_ADD_MAPPED_BUFFER:
		{
			u8 iDMAChannel;
			u64 jobIndex;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob;
			int result;

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (__get_user(jobIndex, (u64*)(pToUserMem + 1)) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (jobIndex > _ModuleData.max_dma_buffers) {
				dev_warn(pDevData->dev, "Locked_ioctl> jobIndex is out of range!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pJob = &pDMAChannel->jobBuffers[jobIndex];

			dma_sync_sg_for_device(pDevData->dev, pJob->SGTable.sgl, pJob->SGTable.nents, DMA_FROM_DEVICE);
			
			// start DMA if idle, else add job to Jobs_ToDo FIFO
			result = imago_DMARead_AddJob(pDevData, iDMAChannel, pJob);
			if (result != 0)
				return -result;

			return 0;
		}

		// wait for DMA completion
		case IOC_DMAREAD_WAIT_FOR_BUFFER:
		case IOC_DMAREAD_WAIT_FOR_BUFFER_TS:
		{
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob = NULL;
			int result = 0;
			struct {
				u8 iDMAChannel;
				u32 timeOut_ms;
				u32 reserved;
			} __attribute__((packed)) ctl_in;
			struct s_ctl_out {
				u8 success;
				u16 buffer_counter;
				u64 pVMUser;
				u64 timestamp;
			} __attribute__((packed)) ctl_out = {0, 0, 0, 0};
			int bytes_out;
			
			// compatibility with old library: result size must have fixed value
			if (cmd == IOC_DMAREAD_WAIT_FOR_BUFFER)
				bytes_out = offsetof(struct s_ctl_out, timestamp);
			else
				bytes_out = sizeof(ctl_out);

			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_err(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: No DMA support!\n");
				return -EFAULT;
			}
			if (__copy_from_user(&ctl_in, pToUserMem, sizeof(ctl_in))) {
				dev_err(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: copy_from_user() faild\n");
				return -EFAULT;
			}
			if (ctl_in.iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: DMAChannel is out of range!\n");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[ctl_in.iDMAChannel];

			dev_dbg(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: DMA: %u, TimeOut_ms: %u\n",
				ctl_in.iDMAChannel, ctl_in.timeOut_ms);

			if (pDMAChannel->abortWait) {
				// check if abort operation is in progress => no additional waiting threads are allowed
				// (should already be avoided by the library, but is a race condition)
				if (copy_to_user(pToUserMem, &ctl_out, bytes_out)) {
					dev_err(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: copy_to_user() failed\n");
					return -EFAULT;
				}
				return bytes_out;
			}

			// wait for DMA completion
			pDMAChannel->dmaWaitCount++;

			// but first, release IOCTL lock while waiting
			up(&pDevData->DeviceSem);

			if (ctl_in.timeOut_ms == 0xFFFFFFFF) {
				// wait for completion without timeout, interruptible
				if (wait_for_completion_interruptible(&pDMAChannel->job_complete) != 0) {
					dev_info(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: wait_for_completion_interruptible() was interrupted\n");
					result = -ERESTARTSYS;
				}
			}
			else {
				// wait for completion with timeout, not interruptible
				if (wait_for_completion_timeout(&pDMAChannel->job_complete, msecs_to_jiffies(ctl_in.timeOut_ms)) == 0) {
					dev_dbg(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: timeout\n");
					result = -EINTR;
				}
			}

			// take IOCTL lock again
			down(&pDevData->DeviceSem);

			pDMAChannel->dmaWaitCount--;

			if (pDMAChannel->abortWait) {
				// abort operation has started, don't use valid buffer in Jobs_Done FIFO
				if (result < 0) {
					// avoid race condition: abort operation just started after completion timeout or signal,
					// we need to correct the completion count now:
					wait_for_completion(&pDMAChannel->job_complete);
					dev_dbg(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: abortWait with completion error %d\n", result);
				}
				if (__copy_to_user(pToUserMem, &ctl_out, bytes_out)) {
					dev_err(pDevData->dev, "Locked_ioctl: copy_to_user() failed\n");
					return -EFAULT;
				}
				return bytes_out;
			}
			if (result < 0)
				return result;

			// get buffer from jobs done FIFO
			flags = imago_DMARead_Lock(pDevData);
			if (kfifo_get(&pDMAChannel->Jobs_Done, &pJob) == 0) {
				imago_DMARead_Unlock(pDevData, flags);
				dev_err(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: DMA completed without buffer\n");
				return -EFAULT;
			}
			imago_DMARead_Unlock(pDevData, flags);

			// send buffer to user (can also be dummy-Buffer)
			ctl_out.success = pJob->boIsOk;
			ctl_out.buffer_counter = pJob->BufferCounter;
			ctl_out.pVMUser = pJob->pVMUser;
			ctl_out.timestamp = pJob->timestamp;
			if (copy_to_user(pToUserMem, &ctl_out, bytes_out)) {
				dev_err(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: copy_to_user() failed\n");
				return -EFAULT;
			}

			dev_dbg(pDevData->dev, "Locked_ioctl IOC_DMAREAD_WAIT_FOR_BUFFER: return buffer iDMA: %d, res: %d, Seq: %d, VMPtr: %p\n",
				ctl_in.iDMAChannel, pJob->boIsOk, pJob->BufferCounter, (void*)pJob->pVMUser);

			// unmap buffer (pJob is released) or handle cache
			if (!pDMAChannel->doManualMap)
				imago_DMARead_UnMapUserBuffer(pDevData, pJob);
			else
				dma_sync_sg_for_cpu(pDevData->dev, pJob->SGTable.sgl, pJob->SGTable.nents, DMA_FROM_DEVICE);

			return bytes_out;
		}

		// abort DMA transfer
		case IOC_DMAREAD_ABORT_DMA:
		// abort waiting user threads
		case IOC_DMAREAD_ABORT_WAITER:
		{
			//> Args vom User lesen und testen
			u8 	iDMAChannel;
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				dev_warn(pDevData->dev, "Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (__get_user(iDMAChannel, pToUserMem) != 0) {
				dev_warn(pDevData->dev, "Locked_ioctl> get_user failed\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				dev_warn(pDevData->dev, "Locked_ioctl> DMAChannel is out of range!\n");
				return -EFAULT;
			}

			//> DMA/UserThreads abbrechen
			if(cmd == IOC_DMAREAD_ABORT_DMA)
				return imago_DMARead_Abort_DMAChannel(pDevData, iDMAChannel);
			else
				return imago_DMARead_Abort_DMAWaiter(pDevData, iDMAChannel);
		}
		case IOC_INIT_I2C_ADAPTER:
		{
			long busNumber;
			u8 busNum;
			busNumber = imago_init_i2cAdapter(pDevData);
			busNum = busNumber;
			if (__put_user(busNum, pToUserMem) != 0) {
				dev_err(pDevData->dev, "Locked_ioctl > put_user() failed\n");
				return -EFAULT;
			}
			return 0;
		}
		default:
			return -ENOTTY;
	}
}


long imago_create_deviceid(PDEVICE_DATA pDevData, u8* deviceIdOut)
{
	struct SUN_DEVICE_DATA* pSunDevice;
	unsigned int deviceId;

	// DeviceID 0 is reserved to avoid conflicts with register writes without
	// a DeviceID in Header1
	for (deviceId = 1; deviceId < MAX_IRQDEVICECOUNT; deviceId++)
	{
		if (pDevData->SunDeviceData[deviceId].requestState == SUN_REQ_STATE_FREE)
		{
			pSunDevice = &pDevData->SunDeviceData[deviceId];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,00)
			reinit_completion(&pDevData->SunDeviceData[deviceId].result_complete);
#else
			INIT_COMPLETION(pDevData->SunDeviceData[deviceId].result_complete);
#endif
			// add serialID in case the user space uses the DeviceID directly
			* deviceIdOut = deviceId | (pSunDevice->serialID << 6);

			pSunDevice->requestState = SUN_REQ_STATE_IDLE;
			dev_dbg(pDevData->dev, "create_deviceid > NewDeviceID = %d\n", deviceId);
			return 1;
		}
	}

	dev_warn(pDevData->dev, "create_deviceid > No free DeviceID\n");
	return -EMFILE;
}

long imago_release_deviceid(PDEVICE_DATA pDevData, u8 deviceID) {

	struct SUN_DEVICE_DATA* pSunDevice;
	unsigned long flags;
	// strip serialID (bit 6)
	deviceID &= (MAX_IRQDEVICECOUNT - 1);
	pSunDevice = &pDevData->SunDeviceData[deviceID];

	dev_dbg(pDevData->dev, "release_deviceid: %u\n", deviceID);

	raw_spin_lock_irqsave(&pDevData->lock, flags);
	if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA ||
		pSunDevice->requestState == SUN_REQ_STATE_ABORT) {
		// Pending request is still in FPGA => toggle serial ID
		pSunDevice->serialID = !pSunDevice->serialID;
	}
	pSunDevice->requestState = SUN_REQ_STATE_FREE;
	raw_spin_unlock_irqrestore(&pDevData->lock, flags);

	return 0;
}

long imago_abort_longterm_read(PDEVICE_DATA pDevData, u8 deviceID) {

	struct SUN_DEVICE_DATA* pSunDevice;
	unsigned long flags;

	// strip serialID (bit 6)
	deviceID &= (MAX_IRQDEVICECOUNT - 1);
	pSunDevice = &pDevData->SunDeviceData[deviceID];

	dev_dbg(pDevData->dev, "abort_longterm_read > Aborting read for DeviceID %u\n", deviceID);

	raw_spin_lock_irqsave(&pDevData->lock, flags);
	if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA) {
		pSunDevice->requestState = SUN_REQ_STATE_ABORT;
		// do not toggle serial ID yet, request may still be answered by the FPGA!
//				pSunDevice->serialID = !pSunDevice->serialID;
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
		complete(&pSunDevice->result_complete);
	}
	else if (pSunDevice->requestState == SUN_REQ_STATE_RESULT) {
		// avoid race condition: the result may still be processed by read(), so leave it there
//				pSunDevice->requestState = SUN_REQ_STATE_IDLE;
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
	}
	else {
		// already in state SUN_REQ_STATE_ABORT or SUN_REQ_STATE_IDLE
		raw_spin_unlock_irqrestore(&pDevData->lock, flags);
	}
	return 0;

}