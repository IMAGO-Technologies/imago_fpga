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
	u32 regAdr;

	/* alles gut? */
	if (pDevData == NULL || pDevData->DeviceSubType == SubType_Invalid)
		return;
	if ((IS_TYPEWITH_PCI(pDevData) || IS_TYPEWITH_COMMONBUFFER(pDevData)) && pDevData->pVABAR0 == NULL)
		return;

	//der DPC ist/ muss durch sein
	//(kann aber erst hier gesetzt werden da sonst race cond mit CommonBuffer Clear, IRQEnable und DPCFlag)
	if (boTurnOn) {

		/* CommonBuffer Adr setzen (nur gÃ¼ltig solange IRQon) & Init */
		if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
			//nur um ganz sicher zu sein
			if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
				return;

			//> das IRQ Flag löschen (2 Word [Flags])
			//ACHTUNG! die 3 Word [Header0/Header1/Data[0]] nicht überschreiben da alte VCXM/CL-PCIe FPGAs während ein DMA IRQ behandelt wurde
			//	schon das SUNPaket in den CommonBuffer geschrieben haben, 
			//	nach dem setzen des "IRQ-Enable Bits" im FPGA hat dieser dann das FLAG gesetzt und den MSI geschickt
			memset(pDevData->pVACommonBuffer,0, 2 * sizeof(u32));
			smp_mb();	//nop bei x86

			//> Adr ins FPGA setzen
			//(low 32Bit)
			iowrite32( (pDevData->pBACommonBuffer & 0xFFFFFFFF), pDevData->pVABAR0 + ISR_COMMONBUFFER_ADR_AGEX2);

#ifdef CONFIG_64BIT		
			//(high 32Bit)
			if( IS_TYPEWITH_PCI64BIT(pDevData) )
				iowrite32( (pDevData->pBACommonBuffer >> 32), pDevData->pVABAR0 + ISR_COMMONBUFFER_ADR_AGEX2 +4);
#endif		
		}

		pDevData->boIsDPCRunning = FALSE;
	}

	/* IRQ enable flag */

	//wo kommt das On/Off Bit hin?
	if( IS_TYPEWITH_COMMONBUFFER(pDevData) )
		regAdr = ISR_ONOFF_OFFSET_AGEX2;
	else
		regAdr = ISR_ONOFF_OFFSET_AGEX;

	if (IS_TYPEWITH_PCI(pDevData) || IS_TYPEWITH_COMMONBUFFER(pDevData))
		iowrite32(boTurnOn ? 1 : 0, pDevData->pVABAR0 + regAdr);
//	else if (IS_TYPEWITH_SPI(pDevData)	=> passiert automatisch
}



//< HWIRQ > nur pruefen ob von uns -> tasklet starten
irqreturn_t AGEXDrv_interrupt(int irq, void *dev_id)
{
	u32 regVal;
	PDEVICE_DATA pDevData = NULL;

	/* war der int von uns? reg lesen  */
	if( dev_id == NULL )
		return IRQ_NONE;
	pDevData = (PDEVICE_DATA) dev_id;
	if( (pDevData->pVABAR0 == NULL) || (pDevData->DeviceSubType == SubType_Invalid) )
		return IRQ_NONE;

	/* läuft der DPC noch 
	 * (damit wir nicht bei fremd HWIs, nicht falsch weil doppelt<>Flag im CommonBuffer, denken er währe unser)
	 * der DPC könnte noch laufen*/
	if(pDevData->boIsDPCRunning==TRUE)
		return IRQ_NONE;


	/* liest das IRQ flag ein */
	if( IS_TYPEWITH_COMMONBUFFER(pDevData) )
	{
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
			return IRQ_NONE;

		regVal = ((u32*)pDevData->pVACommonBuffer)[0];		//das Bit steht in uns Drin

		//bei nicht DMA nur 1. Bit (sicher ist sicher)
		if( !IS_TYPEWITH_DMA2HOST(pDevData) )
			regVal &= 0x1;
	}
	else
	{
		//das Flag geht weg wenn FIFO leer oder IRQs off sind
		regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);
		regVal &= 0x1;		//nur das 1. Bit ist das IRQFlag (sicher ist sicher)
	}


	/* ja, unser interrupt */
	if(regVal != 0)
	{
		//INTs abschalten
		//AGEX: IRQ geht weg wenn FIFO leer oder IRQs off sind
		//AGEX2...: automatisch
		if( !IS_TYPEWITH_COMMONBUFFER(pDevData) )
			AGEXDrv_SwitchInterruptOn(pDevData, FALSE);

		//merken das gleich der DPC läuft
		/* kann keine 2 HWIs zur gleichen Zeit geben, aber der 
		 * DPC(eigentlich nicht da der auf der selben CPU ausgeführt wird)
		 * könnte durch sein vor dem das die HWI durch ist*/
		pDevData->boIsDPCRunning=TRUE;

		// trigger the tasklet (sollte immer gehen)
		tasklet_schedule(&pDevData->IRQTasklet);

		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;
}


//<====================================>
//	tasklet fns
//
//<====================================>

//SWI bzw. DPC Achtung hat keinen process context
void AGEXDrv_tasklet_PCIe(unsigned long data)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)data;
	u32 		DMARead_isDone = 0, DMARead_isOK = 0;
	u16			DMARead_BufferCounters[MAX_DMA_READ_DMACHANNELS*MAX_DMA_READ_CHANNELTCS];
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			wordCount;
	u32			IRQReg_A, IRQReg_B;

	if (pDevData->DeviceSubType == SubType_Invalid) {
		dev_err(pDevData->dev, "AGEXDrv_tasklet_PCIe() > Invalid hardware type\n");
		return;
	}

	if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
		return;

	IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];
	IRQReg_B = ((u32*)pDevData->pVACommonBuffer)[1];

	if ((IRQReg_A & 0x1) == 1) {
		// SUN Header0/1 einlesen
		sun_packet[0] = ((u32*)pDevData->pVACommonBuffer)[2+0];
		sun_packet[1] = ((u32*)pDevData->pVACommonBuffer)[2+1];

		// Rest vom SUN Paket einlesen
		wordCount = sun_packet[1] & 0xFF;
		if (4 * (2 + wordCount) > sizeof(sun_packet)) {
			dev_err(pDevData->dev, "AGEXDrv_tasklet_PCIe() > SUN Error: word count is invalid: %d\n", wordCount);
			return;
		}
		memcpy(&sun_packet[2], ((u32*)pDevData->pVACommonBuffer) + 2 + 2, 4 * wordCount);
		
		// SUN Packet verarbeiten
		InterruptTaskletSun(pDevData, sun_packet);
	}

	if ( IS_TYPEWITH_DMA2HOST(pDevData) )
	{
		s32 i;
		u32 DMAMask;

		//Bit [3-0] sind reserved, Rest können für ReadDMAs sein
		DMAMask = pDevData->DMARead_channels * pDevData->DMARead_TCs;
		DMAMask = (DMAMask > 28) ? (28) : (DMAMask);	//kann eigentlich nicht sein... 
		DMAMask = (1 << DMAMask)-1;

		DMARead_isDone = (IRQReg_A >> 4)	& DMAMask;
		DMARead_isOK =  (~(IRQReg_B >> 4))	& DMAMask & DMARead_isDone; 

		//nach den IRQFlags, CPUPaket kommt ein BufferZähler
		for (i = 0; i < (pDevData->DMARead_channels+pDevData->DMARead_TCs); i++)
		{
			u8*	pCommonBuffer = (u8*)pDevData->pVACommonBuffer;
			pCommonBuffer += HOST_BUFFER_DMAREAD_COUNTER_OFFSET;
			pCommonBuffer += i*8;	//je 8Byte
			DMARead_BufferCounters[i] = *((u16*)pCommonBuffer);
		}

		//IRQ?
		if (DMARead_isDone != 0)
			AGEXDrv_DMARead_DPC(pDevData, DMARead_isDone, DMARead_isOK, DMARead_BufferCounters);
	}

	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	return;
}

void AGEXDrv_tasklet_PCI(unsigned long data)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)data;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			wordCount;
	u32			regVal, fifoLevel;

	if (pDevData->DeviceSubType == SubType_Invalid) {
		dev_err(pDevData->dev, "AGEXDrv_tasklet_PCI() > Invalid hardware type\n");
		return;
	}

	if (pDevData->pVABAR0 == NULL)
		return;

	// AGEX hat ein FIFO => FIFO-Fuellstand einlesen
	regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);

	// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
	fifoLevel = (regVal >> 1) & 0x1FF;
	if (fifoLevel < 3) {
		// sollte nicht auftreten
		dev_err(pDevData->dev, "AGEXDrv_tasklet_PCI() > SUN Error: FIFO level to small: %d\n", fifoLevel);
		return;
	}

	// SUN Header0/1 einlesen
	sun_packet[0] = ioread32(pDevData->pVABAR0);	// Header0
	sun_packet[1] = ioread32(pDevData->pVABAR0);	// Header1

	// Rest vom SUN Paket einlesen
	wordCount = sun_packet[1] & 0xFF;
	if (4 * (2 + wordCount) > sizeof(sun_packet)) {
		dev_err(pDevData->dev, "AGEXDrv_tasklet_PCI() > SUN Error: wordcount is invalid: %d\n", wordCount);
		return;
	}
	ioread32_rep(pDevData->pVABAR0, &sun_packet[2], wordCount);

	// SUN Packet verarbeiten
	InterruptTaskletSun(pDevData, sun_packet);

	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	return;
}

void AGEXDrv_tasklet_SPI(unsigned long data)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)data;
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			wordCount;
	struct spi_transfer transfer;
	struct spi_message message;
	struct spi_device *spi;
	u8 txbuf[1+3*4] = {1 /* message type: read from CPU Target-FIFO */};
	u8 rxbuf[1+3*4];
	int result;

	if (pDevData->DeviceSubType == SubType_Invalid) {
		dev_err(pDevData->dev, "AGEXDrv_tasklet_SPI() > Invalid hardware type\n");
		return;
	}

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
		return;
	}

	wordCount = rxbuf[1+4];	// Header1
	if (wordCount != 1) {
		dev_err(pDevData->dev, "AGEXDrv_tasklet_SPI() > word count is invalid: %d\n", wordCount);
		return;
	}

	memcpy(sun_packet, &rxbuf[1], 3 * 4);

	// Process SUN packet
	InterruptTaskletSun(pDevData, sun_packet);

	return;
}

//wird mit FPGA-IRQ Off im IRQ-Tasklet aufgerufen, kümmert sich um das SUN-Paket, pDevData ist gültig
static void InterruptTaskletSun(DEVICE_DATA *pDevData, u32 *sun_packet)
{
	u8 wordCount = sun_packet[1] & 0xFF;
	u8 deviceID = (sun_packet[1] >> 20) & (MAX_IRQDEVICECOUNT-1);
	u8 serialId = (sun_packet[1] >> 27) & 1;
	struct SUN_DEVICE_DATA *pSunDevice = &pDevData->SunDeviceData[deviceID];

	dev_printk(KERN_DEBUG, pDevData->dev, "InterruptTaskletSun() > packet word count: %d\n", wordCount);

	if (wordCount != 1)
		dev_warn(pDevData->dev, "InterruptTaskletSun() > received payload size is incorrect: %u words\n", wordCount);

	spin_lock_bh(&pDevData->lock);

	if (pSunDevice->requestState == SUN_REQ_STATE_INFPGA && pSunDevice->serialID == serialId) {
		pSunDevice->requestState = SUN_REQ_STATE_RESULT;

		spin_unlock_bh(&pDevData->lock);

		dev_printk(KERN_DEBUG, pDevData->dev, "completing request for DeviceID %u\n", deviceID);

		memcpy(pSunDevice->packet, sun_packet, 3*4);
		up(&pSunDevice->semResult);
	}
	else {
		spin_unlock_bh(&pDevData->lock);
		if (pSunDevice->requestState != SUN_REQ_STATE_INFPGA)
			dev_warn(pDevData->dev, "InterruptTaskletSun() > Unexpected state (%u) for DeviceID=%u, dropping packet\n", pSunDevice->requestState, deviceID);
		else
			dev_warn(pDevData->dev, "InterruptTaskletSun() > serial ID doesn't match for DeviceID=%u, dropping packet\n", deviceID);
	}
}

