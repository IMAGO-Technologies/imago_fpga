/*
 * LockedOps.c
 *
 * Handle the the read/write/IO Ops 
 * (only one thread is inside this file at the same time, every arg should be valid)
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

#include "AGEXDrv.h"


// writes FPGA packet
long Locked_write(PDEVICE_DATA pDevData, const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	u8 TempBuffer[4*(3+1)];		// +1 damit bei PCIe immer 64 Bit sind
	u8 deviceID;

	// User Data -> Kernel
	if (BytesToWrite > sizeof(TempBuffer))
		return -EFBIG;
	if (copy_from_user (TempBuffer, pToUserMem, BytesToWrite) != 0) {
		dev_warn(pDevData->dev, "Locked_write(): copy_from_user() faild\n");
		return -EFAULT;
	}

	// insert serialID to Header1:
	deviceID = (((u32*)TempBuffer)[1] >> 20) & (MAX_IRQDEVICECOUNT - 1);
	((u32*)TempBuffer)[1] |= pDevData->SunDeviceData[deviceID].serialID << 26;

	if (IS_TYPEWITH_PCI(pDevData) || IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		/* Kernel -> PCI */
		//Notes:
		// -für die AGEX muss sich die Adr nicht ändern
		// -bei der AGEX2 müssen es 32Bit mit steigender Adr sein
		//
		// aus include/asm-generic/iomap.h für ioread/writeX_rep
		// "...They do _not_ update the port address. If you
		//	want MMIO that copies stuff laid out in MMIO
		//	memory across multiple ports, use "memcpy_toio()..."
		//
		// aber auch memcpy_toio() macht nicht immer 32Bit
		// "http://www.gossamer-threads.com/lists/linux/kernel/650995?do=post_view_threaded#650995"
		if (BytesToWrite % 4) {
			u32 ByteIndex =0;
			for(; ByteIndex < BytesToWrite; ByteIndex++)
				iowrite8(TempBuffer[ByteIndex], pDevData->pVABAR0+ByteIndex);
		}
		else {
			u32 WordsToCopy = BytesToWrite/4;
			u32 WordIndex;
			for (WordIndex = 0; WordIndex < WordsToCopy; WordIndex++)
				iowrite32(((u32*)TempBuffer)[WordIndex], pDevData->pVABAR0 + WordIndex*4 );
		}
	}
	else if (IS_TYPEWITH_SPI(pDevData)) {
		struct spi_transfer transfer;
		struct spi_message message;
		struct spi_device *spi = to_spi_device(pDevData->dev);
		int result;
		u8 txbuf[1+3*4];		// SPI Header + SUN Paket

		txbuf[0] = 0; // SPI Header: in CPU Source-FIFO schreiben
		memcpy(&txbuf[1], TempBuffer, 3*4);

		spi_message_init(&message);
		memset(&transfer, 0, sizeof(transfer));
		transfer.tx_buf = txbuf;
		transfer.len = sizeof(txbuf);
		spi_message_add_tail(&transfer, &message);

		result = spi_sync(spi, &message);
		if (result < 0) {
			dev_err(&spi->dev, "Locked_write> SPI error %d\n", result);
			return -EFAULT;
		}
	}

	return BytesToWrite;
}

//führt eine IOOp durch, gibt >= 0 für OK sonst fehler code zurück,
long Locked_ioctl(PDEVICE_DATA pDevData, const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes)
{
	struct SUN_DEVICE_DATA *pSunDevice;
	unsigned long flags;

	switch (cmd)
	{
		/* Gibt die Version als String zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_VERSION:
			if (BufferSizeBytes < sizeof(MODVERSION)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (copy_to_user(pToUserMem, MODVERSION, sizeof(MODVERSION)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
				return -EFAULT;
			}
			return sizeof(MODVERSION);


		/* Gibt das Build date/time als String zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_BUILD_DATE:
			if (sizeof(MODDATECODE) > BufferSizeBytes ) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (copy_to_user(pToUserMem, MODDATECODE, sizeof(MODDATECODE)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
				return -EFAULT;
			}
			return sizeof(MODDATECODE);


		/* Gibt SubType zurück */
		/**********************************************************************/
		case AGEXDRV_IOC_GET_SUBTYPE:
			if (sizeof(pDevData->DeviceSubType) > BufferSizeBytes ) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (copy_to_user(pToUserMem, &pDevData->DeviceSubType, sizeof(pDevData->DeviceSubType)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> copy_to_user faild\n");
				return -EFAULT;
			}
			return sizeof(pDevData->DeviceSubType);


		/* Markiert die DeviceID als frei*/
		/**********************************************************************/
		//	-> kein Fehler wenn ungenutzt oder noch im LongTermRequest noch OutOfRange
		case AGEXDRV_IOC_RELEASE_DEVICEID:
		{
			u8 DeviceID;

			if (BufferSizeBytes != 1) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}

			if (get_user(DeviceID, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}

			//gültig?
			if (DeviceID >= MAX_IRQDEVICECOUNT)
			{
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DeviceID out of range!\n");
				return 0;
			}

			pSunDevice = &pDevData->SunDeviceData[DeviceID];

			spin_lock_irqsave(&pDevData->lock, flags);
			if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA) {
				pSunDevice->serialID = !pSunDevice->serialID;		// Pending request is still in FPGA => toggle serial ID
			}
			pSunDevice->requestState = SUN_REQ_STATE_FREE;
			spin_unlock_irqrestore(&pDevData->lock, flags);

			return 0;
		}


		/* Gibt eine neue DeviceID zurück */
		/**********************************************************************/
		// -> nur wenn sie zur Zeit als frei markiert ist boIsDeviceIDUsed[]==false
		// -> nur wenn sie nicht im LongTermRequestList[] ist
		// 		dass kann passieren wenn die DLL eine LongTermAnfrage startet und dann das device zu macht
		// 		die Anfrage ist dann noch unterwegs, ja der Request wurde abgebrochen aber die DeviceID ist
		// 		noch genutzt
		case AGEXDRV_IOC_CREATE_DEVICEID:
		{
			unsigned int index;
			
			if (BufferSizeBytes != 1) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}

			for (index=0; index < MAX_IRQDEVICECOUNT; index++)
			{
				if (pDevData->SunDeviceData[index].requestState == SUN_REQ_STATE_FREE)
				{
					pSunDevice = &pDevData->SunDeviceData[index];

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
					sema_init(&pDevData->SunDeviceData[index].semResult, 0);
#else
					init_MUTEX_LOCKED(&pDevData->SunDeviceData[index].semResult);
#endif
					if (put_user((u8)index, pToUserMem) != 0) {
						dev_err(pDevData->dev, "Locked_ioctl > put_user() failed\n");
						return -EFAULT;
					}

					pSunDevice->requestState = SUN_REQ_STATE_IDLE;
					dev_dbg(pDevData->dev, "Locked_ioctl > NewDeviceID = %d\n", index);
					return 1;
				}
			}

			dev_warn(pDevData->dev, "Locked_ioctl > No free DeviceID\n");
			return -EMFILE;
		}

		/* Wenn zur DeviceID ein LONGTERM_IOREQUEST läuft, sem posten. flag setzen*/
		/**********************************************************************/
		case AGEXDRV_IOC_ABORT_LONGTERM_READ:
		{
			u8 DeviceID;

			if (BufferSizeBytes != 1) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}

			//DevId lesen
			if (get_user(DeviceID, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}

			pSunDevice = &pDevData->SunDeviceData[DeviceID];

			dev_dbg(pDevData->dev, "Locked_ioctl > Aborting read for DeviceID %u\n", DeviceID);

			spin_lock_irqsave(&pDevData->lock, flags);
			if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA) {
				pSunDevice->requestState = SUN_REQ_STATE_ABORT;
				pSunDevice->serialID = !pSunDevice->serialID;		// toggle serial ID
				spin_unlock_irqrestore(&pDevData->lock, flags);
				up(&pSunDevice->semResult);
			}
			else if (pSunDevice->requestState == SUN_REQ_STATE_RESULT) {
				pSunDevice->requestState = SUN_REQ_STATE_ABORT;
				spin_unlock_irqrestore(&pDevData->lock, flags);
			}
			else {
				spin_unlock_irqrestore(&pDevData->lock, flags);
			}
			return 0;
		}


		/************************ DMA commands **********************/

		// Set number of DMA channels / Transfer Channels / SG elements which are present in the FPGA.
		// Values can only be modified when 0 (or set with the same value).
		case AGEXDRV_IOC_DMAREAD_CONFIG:
		{
			u16 tmpDMAs, tmpTCs, tmpSGs;
			u8 iDMA, iTC;

			if (BufferSizeBytes < (3*2)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {				
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (get_user(tmpDMAs, (u16*)(pToUserMem + 0)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(tmpTCs,(u16*)(pToUserMem + 2)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(tmpSGs,(u16*)(pToUserMem + 4)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}

			//nur gr��er setzen, bzw nur wenn noch nicht gesetzt
			// - in AGEXDrv_init()/AGEXDrv_PCI_probe() > AGEXDrv_InitDrvData() wurde alle max m�glichen geinit
			// - nicht �ndern wegen RaceCond, da wir davon augehen das die IRQFlags fortlaufend sind
			if ( 	!((pDevData->DMARead_channels == 0) || (pDevData->DMARead_channels == tmpDMAs) ) 
				|| 	!((pDevData->DMARead_TCs == 0) || (pDevData->DMARead_TCs == tmpTCs)) 
				|| 	!((pDevData->DMARead_SGs == 0) || (pDevData->DMARead_SGs == tmpSGs))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Can't configure used DMA/TCs!\n");
				return -EFAULT;
			}
			//nicht ueber max setzen
			if ((tmpDMAs > MAX_DMA_READ_DMACHANNELS) || (tmpTCs > MAX_DMA_READ_CHANNELTCS) || (tmpSGs > MAX_DMA_READ_TCSGS)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> ConfigDMARead anzDMA/TC/SG too large!\n");
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

			dev_dbg(pDevData->dev, "Locked_ioctl > ConfigDMARead anzDMAs: %d, anzTCs: %d, anzSGs: %d\n",
					pDevData->DMARead_channels, pDevData->DMARead_TCs, pDevData->DMARead_SGs);
			return 0;
		}

		// reset DMA channel in case a transfer is pending or lost mapped buffers after a process was killed
		case AGEXDRV_IOC_DMAREAD_RESETCHANNEL:
		{
			u8 iDMAChannel;

			if (BufferSizeBytes < 1) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!\n");
				return -EFAULT;
			}
			
			return AGEXDrv_DMARead_Reset_DMAChannel(pDevData, iDMAChannel);
		}

		// map user buffer for DMA
		case AGEXDRV_IOC_DMAREAD_MAP_BUFFER:
		{
			u8 	iDMAChannel;
			u64 UserPTR, bufferSize;
			DMA_READ_CHANNEL *pDMAChannel;
			DMA_READ_JOB *pJob = NULL;

			if (BufferSizeBytes < (1 + 2 * sizeof(u64))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(UserPTR, (u64*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(bufferSize, (u64*)(pToUserMem + sizeof(u64) + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
		
			//wenn es kein DMA gibt ist anz=0
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (bufferSize == 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> invalid buffer size!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pDMAChannel->doManualMap = true;

			// map buffer
			if (!AGEXDrv_DMARead_MapUserBuffer(pDevData, pDMAChannel, (uintptr_t)UserPTR, bufferSize, &pJob)) {
				AGEXDrv_DMARead_UnMapUserBuffer(pDevData, pJob);
				dev_warn(pDevData->dev, "Locked_ioctl / AGEXDRV_IOC_DMAREAD_MAP_BUFFER > AGEXDrv_DMARead_MapUserBuffer() failed\n");
				return -EINTR;
			}

			// return index of pJob jobBuffers to user space
			if (put_user((u64)(pJob - pDMAChannel->jobBuffers), (u64 *)pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}

			return sizeof(u64);
		}

		// unmap user buffer
		case AGEXDRV_IOC_DMAREAD_UNMAP_BUFFER:
		{
			u8 iDMAChannel;
			u64 jobIndex;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob;

			if (BufferSizeBytes < (1 + sizeof(u16))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(jobIndex, (u64*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (jobIndex > ARRAY_SIZE(pDMAChannel->jobBuffers)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> jobIndex is out of range!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pJob = &pDMAChannel->jobBuffers[jobIndex];

			AGEXDrv_DMARead_UnMapUserBuffer(pDevData, pJob);
			
			return 0;
		}
		
		// add buffer for DMA (old version which also maps the buffer)
		case AGEXDRV_IOC_DMAREAD_ADD_BUFFER:
		{
			u8 iDMAChannel;
			u64 UserPTR, bufferSize;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob = NULL;
			int result;

			if (BufferSizeBytes < (1 + 2 * sizeof(u64))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(UserPTR, (u64*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(bufferSize, (u64*)(pToUserMem + sizeof(u64) + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (bufferSize == 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> invalid buffer size!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pDMAChannel->doManualMap = false;

			// map user buffer for DMA
			if (!AGEXDrv_DMARead_MapUserBuffer(pDevData, pDMAChannel, (uintptr_t) UserPTR, bufferSize, &pJob)) {
				AGEXDrv_DMARead_UnMapUserBuffer(pDevData, pJob);
				printk(KERN_WARNING MODDEBUGOUTTEXT"Locked_ioctl> (AGEXDRV_IOC_DMAREAD_ADD_BUFFER), AGEXDrv_DMARead_MapUserBuffer() failed!\n");
				return -EINTR;
			}

			// start DMA if idle, else add job to Jobs_ToDo FIFO
			result = AGEXDrv_DMARead_AddJob(pDevData, iDMAChannel, pJob);
			if (result != 0) {
				AGEXDrv_DMARead_UnMapUserBuffer(pDevData, pJob);
				return -result;
			}

			return 0;
		}

		// add mapped buffer for DMA
		case AGEXDRV_IOC_DMAREAD_ADD_MAPPED_BUFFER:
		{
			u8 iDMAChannel;
			u64 jobIndex;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob;
			int result;

			if (BufferSizeBytes < (1 + sizeof(u64))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//> Args vom User lesen und testen
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(jobIndex, (u64*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!");
				return -EFAULT;
			}
			if (jobIndex > ARRAY_SIZE(pDMAChannel->jobBuffers)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> jobIndex is out of range!");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];
			pJob = &pDMAChannel->jobBuffers[jobIndex];

			dma_sync_sg_for_device(pDevData->dev, pJob->SGTable.sgl, pJob->SGTable.nents, DMA_FROM_DEVICE);
			
			// start DMA if idle, else add job to Jobs_ToDo FIFO
			result = AGEXDrv_DMARead_AddJob(pDevData, iDMAChannel, pJob);
			if (result != 0)
				return -result;

			return 0;
		}

		// wait for DMA completion
		case AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER:
		{
			u8 	iDMAChannel;
			u32 TimeOut_ms;
			PDMA_READ_CHANNEL pDMAChannel;
			DMA_READ_JOB *pJob = NULL;
			uintptr_t pVMUser;

			if (BufferSizeBytes < (1 + sizeof(u16) + sizeof(u64))) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}

			//args vom USER
			if (get_user(iDMAChannel, pToUserMem + 0) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (get_user(TimeOut_ms, (u32*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!\n");
				return -EFAULT;
			}

			pDMAChannel = &pDevData->DMARead_Channel[iDMAChannel];

			//> warten (mit timeout)
			//IOCTRL lock tempor�r l�sen
			up(&pDevData->DeviceSem);
//<......................................................................
			// aufwachen durch Signal, up vom SWI, oder User [Abort], bzw TimeOut
			if (TimeOut_ms == 0xFFFFFFFF)
			{	
				//Note: unterbrechbar(durch gdb), abbrechbar durch kill -9 & kill -15(term) 
				//  noch ist nix passiert, Kernel darf den Aufruf wiederhohlen ohne den User zu benachrichtigen							
				if (down_interruptible(&pDMAChannel->WaitSem) != 0) {
					dev_dbg(pDevData->dev, "Locked_ioctl AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER > down_interruptible() failed!\n");
					down(&pDevData->DeviceSem);
					return -ERESTARTSYS;
				}
			}
			else
			{
				//Note: nicht unter(durch GDB) bzw. abbrechbar down_timeout() > __down_timeout() > __down_common(sem, TASK_UNINTERRUPTIBLE, timeout);
				int waitRes = down_timeout(&pDMAChannel->WaitSem, msecs_to_jiffies(TimeOut_ms));
				if (waitRes == (-ETIME)) {
					dev_dbg(pDevData->dev, "Locked_ioctl AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER > timeout\n");
					down(&pDevData->DeviceSem);
					return -ETIME;
				}
				if (waitRes != 0) {
					dev_err(pDevData->dev, "Locked_ioctl AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER > down_timeout() failed\n");
					down(&pDevData->DeviceSem);
					return -EINTR;
				}
			}
			down(&pDevData->DeviceSem);
//......................................................................>
			
			// get buffer from jobs done FIFO
			flags = AGEXDrv_DMARead_Lock(pDevData);
			if (kfifo_get(&pDMAChannel->Jobs_Done, &pJob) == 0) {
				AGEXDrv_DMARead_Unlock(pDevData, flags);
				dev_err(pDevData->dev, "Locked_ioctl AGEXDRV_IOC_DMAREAD_WAIT_FOR_BUFFER > DMARead wake up, without buffer!\n");
				return -EFAULT;
			}
			AGEXDrv_DMARead_Unlock(pDevData, flags);

			pVMUser = pJob->pVMUser;	// save user pointer, gets cleared by AGEXDrv_DMARead_UnMapUserBuffer()

			// unmap buffer or handle cache
			if (!pDMAChannel->doManualMap)
				AGEXDrv_DMARead_UnMapUserBuffer(pDevData, pJob);
			else
				dma_sync_sg_for_cpu(pDevData->dev, pJob->SGTable.sgl, pJob->SGTable.nents, DMA_FROM_DEVICE);

			// send buffer to user (can also be dummy-Buffer)
			if (put_user( (u8) pJob->boIsOk, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (put_user(pJob->BufferCounter, (u16*)(pToUserMem + 1)) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (put_user(pVMUser, (u64*)(pToUserMem + 1 + sizeof(u16))) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}

			pr_devel(MODDEBUGOUTTEXT" Locked_ioctl> DMARead return buffer iDMA: %d, res: %d, Seq: %d, VMPtr: %p\n",
				iDMAChannel, pJob->boIsOk, pJob->BufferCounter, (void*)pVMUser);
			return 1 + sizeof(u16) +  sizeof(u64);
		}

		// abort DMA transfer
		case AGEXDRV_IOC_DMAREAD_ABORT_DMA:
		// abort waiting user threads
		case AGEXDRV_IOC_DMAREAD_ABORT_WAITER:
		{
			//> Args vom User lesen und testen
			u8 	iDMAChannel;
			if (BufferSizeBytes < 1) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> Buffer Length to short\n");
				return -EFBIG;
			}
			if (!IS_TYPEWITH_DMA2HOST(pDevData)) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> No DMA support!\n");
				return -EFAULT;
			}
			if (get_user(iDMAChannel, pToUserMem) != 0) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> get_user faild\n");
				return -EFAULT;
			}
			if (iDMAChannel >= pDevData->DMARead_channels) {
				printk(KERN_WARNING MODDEBUGOUTTEXT" Locked_ioctl> DMAChannel is out of range!\n");
				return -EFAULT;
			}

			//> DMA/UserThreads abbrechen
			if(cmd == AGEXDRV_IOC_DMAREAD_ABORT_DMA)
				AGEXDrv_DMARead_Abort_DMAChannel(pDevData, iDMAChannel);
			else
				AGEXDrv_DMARead_Abort_DMAWaiter(pDevData, iDMAChannel);

			return 0;
		}

		default:
			return -ENOTTY;
	}
}
