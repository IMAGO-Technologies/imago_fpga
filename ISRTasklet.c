/*
 * ISRTasklet.c
 *
 * handel the PCI(e) IRQs/MSI and the tasklet
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
 */

#include "AGEXDrv.h"

//wird mit FPGA-IRQ Off im IRQ-Tasklet aufgerufen, kümmert sich um das SUN-Paket, pDevData ist gültig
static void InterruptTaskletSun(PDEVICE_DATA pDevData, u32 *sun_packet);


//<====================================>
//	IRQ fns
//
//<====================================>

//Schaltet im PCI-Geraet die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn)
{
	if (pDevData == NULL || pDevData->DeviceSubType == SubType_Invalid || pDevData->pVABAR0 == NULL)
		return;

	//der DPC ist/ muss durch sein
	//(kann aber erst hier gesetzt werden da sonst race cond mit CommonBuffer Clear, IRQEnable und DPCFlag)
	if (boTurnOn) {

		if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
			//nur um ganz sicher zu sein
			if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
				return;

			//> das IRQ Flag löschen (2 Word [Flags])
			//ACHTUNG! die 3 Word [Header0/Header1/Data[0]] nicht überschreiben da alte VCXM/CL-PCIe FPGAs während ein DMA IRQ behandelt wurde
			//	schon das SUNPaket in den CommonBuffer geschrieben haben, 
			//	nach dem setzen des "IRQ-Enable Bits" im FPGA hat dieser dann das FLAG gesetzt und den MSI geschickt
			memset(pDevData->pVACommonBuffer, 0, 2 * sizeof(u32));
			smp_mb();	//nop bei x86
		}
	}

	/* IRQ enable flag */
	iowrite32(boTurnOn ? 1 : 0, pDevData->pVABAR0 + (IS_TYPEWITH_COMMONBUFFER(pDevData) ? ISR_ONOFF_OFFSET_AGEX2 : ISR_ONOFF_OFFSET_AGEX));
}



// PCI interrupt
irqreturn_t AGEXDrv_pci_interrupt(int irq, void *dev_id)
{
	u32 regVal;
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;

	BUG_ON(pDevData == NULL || pDevData->pVABAR0 == NULL || pDevData->DeviceSubType == SubType_Invalid);

	// check the FPGA interrupt flag (cleared whein FIFO is empty or when interrupt is disabled)
	regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);
	if ((regVal & 0x1) == 0)
		return IRQ_NONE;	// not our interrupt

	// disable interrupt in FPGA
	AGEXDrv_SwitchInterruptOn(pDevData, FALSE);

	// wake up thread
	return IRQ_WAKE_THREAD;
}

// PCIe interrupt
irqreturn_t AGEXDrv_pcie_interrupt(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			IRQReg_A;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	irqreturn_t result = IRQ_HANDLED;

	IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];

	// check for DMA done flag
	if (IS_TYPEWITH_DMA2HOST(pDevData) && (IRQReg_A >> 4) != 0) {
		if (pDevData->setupTcInHWI)
			AGEXDrv_DMARead_DPC(pDevData);
		else
			result = IRQ_WAKE_THREAD;
	}

	// check for SUN interrupt flag
	if ((IRQReg_A & 0x1) != 0) {
		// get SUN Header0/1
		sun_packet[0] = ((u32*)pDevData->pVACommonBuffer)[2+0];
		sun_packet[1] = ((u32*)pDevData->pVACommonBuffer)[2+1];
		sun_packet[2] = ((u32*)pDevData->pVACommonBuffer)[2+2];

		// process SUN packet
		InterruptTaskletSun(pDevData, sun_packet);
	}

	if (result == IRQ_HANDLED)
		AGEXDrv_SwitchInterruptOn(pDevData, TRUE);

	return result;
}


// IRQ threads

// PCIe: only process DMA interrupts, SUN packets are already handled by the interrupt
irqreturn_t AGEXDrv_pcie_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;

	AGEXDrv_DMARead_DPC(pDevData);

	AGEXDrv_SwitchInterruptOn(pDevData, TRUE);

	return IRQ_HANDLED;
}

irqreturn_t AGEXDrv_pci_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			regVal, fifoLevel;

	BUG_ON(pDevData->DeviceSubType == SubType_Invalid);
	BUG_ON(pDevData->pVABAR0 == NULL);

	// AGEX hat ein FIFO => FIFO-Fuellstand einlesen
	regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);

	// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
	fifoLevel = (regVal >> 1) & 0x1FF;
	if (fifoLevel < 3) {
		// sollte nicht auftreten
		dev_err(pDevData->dev, "AGEXDrv_pci_thread() > SUN Error: FIFO level to small: %d\n", fifoLevel);
		return IRQ_HANDLED;
	}

	// SUN Header0/1 einlesen
	sun_packet[0] = ioread32(pDevData->pVABAR0);	// Header0
	sun_packet[1] = ioread32(pDevData->pVABAR0);	// Header1
	sun_packet[2] = ioread32(pDevData->pVABAR0);	// Payload

	// SUN Packet verarbeiten
	InterruptTaskletSun(pDevData, sun_packet);

	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	return IRQ_HANDLED;
}

irqreturn_t AGEXDrv_spi_thread(int irq, void *dev_id)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)dev_id;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	struct spi_transfer transfer;
	struct spi_message message;
	struct spi_device *spi;
	u8 txbuf[1+3*4] = {1 /* message type: read from CPU Target-FIFO */};
	u8 rxbuf[1+3*4];
	int result;

	BUG_ON(pDevData->DeviceSubType == SubType_Invalid);

	spi = to_spi_device(pDevData->dev);

	spi_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	transfer.tx_buf = txbuf;
	transfer.rx_buf = rxbuf;
	transfer.len = sizeof(txbuf);
	spi_message_add_tail(&transfer, &message);

	result = spi_sync(spi, &message);
	if (result < 0)
	{
		dev_err(pDevData->dev, "AGEXDrv_tasklet_SPI() > SPI error %d\n", result);
		return IRQ_HANDLED;
	}

	memcpy(sun_packet, &rxbuf[1], 3 * 4);

	// Process SUN packet
	InterruptTaskletSun(pDevData, sun_packet);

	return IRQ_HANDLED;
}

// Process a SUN interrupt packet
static void InterruptTaskletSun(DEVICE_DATA *pDevData, u32 *sun_packet)
{
	u8 wordCount = sun_packet[1] & 0xFF;
	u8 deviceID = (sun_packet[1] >> 20) & (MAX_IRQDEVICECOUNT-1);
	u8 serialId = (sun_packet[1] >> 27) & 1;
	struct SUN_DEVICE_DATA *pSunDevice = &pDevData->SunDeviceData[deviceID];
	unsigned long flags;

	dev_dbg(pDevData->dev, "InterruptTaskletSun() > packet word count: %d\n", wordCount);

	if (wordCount != 1) {
		dev_err(pDevData->dev, "InterruptTaskletSun() > received payload size is incorrect: %u words\n", wordCount);
		return;
	}

	spin_lock_irqsave(&pDevData->lock, flags);

	if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA && pSunDevice->serialID == serialId) {
		pSunDevice->requestState = SUN_REQ_STATE_RESULT;

		spin_unlock_irqrestore(&pDevData->lock, flags);

		dev_dbg(pDevData->dev, "completing request for DeviceID %u\n", deviceID);

		memcpy(pSunDevice->packet, sun_packet, 3*4);
		up(&pSunDevice->semResult);
	}
	else {
		spin_unlock_irqrestore(&pDevData->lock, flags);
		if (pSunDevice->requestState != SUN_REQ_STATE_INFPGA)
			dev_warn(pDevData->dev, "InterruptTaskletSun() > Unexpected state (%u) for DeviceID=%u, dropping packet\n", pSunDevice->requestState, deviceID);
		else
			dev_warn(pDevData->dev, "InterruptTaskletSun() > serial ID doesn't match for DeviceID=%u, dropping packet\n", deviceID);
	}
}

