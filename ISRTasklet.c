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

//Schaltet im PCI-GerÃ¤t die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn)
{
	u32 regAdr;

	/* alles gut? */
	if( pDevData == NULL || pDevData->DeviceSubType == SubType_Invalid)
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



//< HWIRQ > nur prÃ¼fen ob von uns, ja tasklet starten
irqreturn_t AGEXDrv_interrupt(int irq, void *dev_id)
{
	u32 regVal;
	PDEVICE_DATA pDevData = NULL;

pr_devel(MODDEBUGOUTTEXT" AGEXDrv_interrupt (IRQ:%d, devptr:%p)\n", irq, dev_id);

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
void AGEXDrv_tasklet(unsigned long data)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)data;
	u32 		DMARead_isDone = 0, DMARead_isOK = 0;
	u16			DMARead_BufferCounters[MAX_DMA_READ_DMACHANNELS*MAX_DMA_READ_CHANNELTCS];
	u32			sun_packet[MAX_SUNPACKETSIZE/4];
	u32			wordCount;

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_tasklet> (Minor:%lu)\n", devIndex);


	if (pDevData->DeviceSubType == SubType_Invalid) {
		printk(KERN_ERR MODDEBUGOUTTEXT" AGEXDrv_tasklet> Invalid hardware type\n");
		return;
	}

	if (IS_TYPEWITH_COMMONBUFFER(pDevData)) {
		u32 IRQReg_A, IRQReg_B;

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
				printk(KERN_ERR MODDEBUGOUTTEXT" SUN Error: wordcount is too big: %d\n", wordCount);
				return;
			}
			memcpy(&sun_packet[2], ((u32*)pDevData->pVACommonBuffer) + 2, 4 * (wordCount + 2));
			
			// SUN Packet verarbeiten
			InterruptTaskletSun(pDevData, sun_packet);
		}

		if ( IS_TYPEWITH_DMA2HOST(pDevData) )
		{
			s32 i;
			u32 DMAMask;

			//Bit [3-0] sind reserved, Rest können für ReadDMAs sein
			DMAMask = pDevData->DMARead_anzChannels * pDevData->DMARead_anzTCs;
			DMAMask = (DMAMask > 28) ? (28) : (DMAMask);	//kann eigentlich nicht sein... 
			DMAMask = (1 << DMAMask)-1;

			DMARead_isDone = (IRQReg_A >> 4)	& DMAMask;
			DMARead_isOK =  (~(IRQReg_B >> 4))	& DMAMask & DMARead_isDone; 

			//nach den IRQFlags, CPUPaket kommt ein BufferZähler
			for (i = 0; i < (pDevData->DMARead_anzChannels+pDevData->DMARead_anzTCs); i++)
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
	}
	else if (IS_TYPEWITH_SPI(pDevData))
	{
		struct spi_transfer transfer;
		struct spi_message message;
		struct spi_device *spi = to_spi_device(pDevData->pDeviceDevice);
		u8 txbuf[1+3*4] = {1 /* message type: read from CPU Target-FIFO */};
		u8 rxbuf[1+3*4];
		int result;

		pr_devel(MODDEBUGOUTTEXT" AGEXDrv_tasklet> SPI transfer start\n");
		trace_printk("AGEXDrv_tasklet> SPI transfer start\n");
		
		spi_message_init(&message);
		memset(&transfer, 0, sizeof(transfer));
		transfer.tx_buf = txbuf;
		transfer.rx_buf = rxbuf;
		transfer.len = sizeof(txbuf);
		spi_message_add_tail(&transfer, &message);

		result = spi_sync(spi, &message);
		if (result < 0)
		{
			dev_err(&spi->dev, "AGEXDrv_tasklet> SPI error %d\n", result);
			return;
		}

		pr_devel(MODDEBUGOUTTEXT" AGEXDrv_tasklet> SPI transfer done\n");
		trace_printk("AGEXDrv_tasklet> SPI transfer done\n");

		wordCount = rxbuf[1+4];	// Header1
		if (wordCount != 1) {
			printk(KERN_ERR MODDEBUGOUTTEXT" SUN Error: wordcount is invalid: %d\n", wordCount);
			return;
		}

		memcpy(sun_packet, &rxbuf[1], 3 * 4);

		// SUN Packet verarbeiten
		InterruptTaskletSun(pDevData, sun_packet);
	}
	else	// => AGE-X (PCI)
	{
		u32 regVal, fifoLevel;

		if (pDevData->pVABAR0 == NULL)
			return;

		// AGEX hat ein FIFO => FIFO-Fuellstand einlesen
		regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);

		// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
		fifoLevel = (regVal >> 1) & 0x1FF;
		if (fifoLevel < 3) {
			// sollte nicht auftreten
			printk(KERN_ERR MODDEBUGOUTTEXT" SUN Error: FIFO level to small: %d\n", fifoLevel);
			return;
		}
		pr_devel(MODDEBUGOUTTEXT" SUN FIFO level: %d\n", fifoLevel);

		// SUN Header0/1 einlesen
		sun_packet[0] = ioread32(pDevData->pVABAR0);	// Header0
		sun_packet[1] = ioread32(pDevData->pVABAR0);	// Header1

		// Rest vom SUN Paket einlesen
		wordCount = sun_packet[1] & 0xFF;
		if (4 * (2 + wordCount) > sizeof(sun_packet)) {
			printk(KERN_ERR MODDEBUGOUTTEXT" SUN Error: wordcount is too big: %d\n", wordCount);
			return;
		}
		ioread32_rep(pDevData->pVABAR0, &sun_packet[2], wordCount);

		// SUN Packet verarbeiten
		InterruptTaskletSun(pDevData, sun_packet);
	}

	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	pr_devel(MODDEBUGOUTTEXT" DPC done\n");

	return;
}

//wird mit FPGA-IRQ Off im IRQ-Tasklet aufgerufen, kümmert sich um das SUN-Paket, pDevData ist gültig
static void InterruptTaskletSun(DEVICE_DATA *pDevData, u32 *sun_packet)
{
	u16 deviceID, wordCount, index;

	bool boPacketDataDone = FALSE;

	wordCount = sun_packet[1] & 0xFF;	//ist somit auch kleiner PAGE_SIZE
	deviceID = (sun_packet[1]>>20) & (MAX_IRQDEVICECOUNT-1);
	pr_devel(MODDEBUGOUTTEXT" SUN WordCount: %d\n", wordCount);

	/* lÃ¤uft noch eine Anfrage fÃ¼r die ID? */
	/**********************************************************************/
	for (index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		// DeviceID auswerten (warten wir auf das SUNPaket?)
		// 	-> DeviceID muss passen und wir haben die daten noch nicht reingefÃ¼llt
		if(		(pDevData->LongTermRequestList[index].boIsInFPGA == TRUE )
			&&	(pDevData->LongTermRequestList[index].DeviceID == deviceID) )
		{
			//Wartet (noch) ein Request aufs Paket?
			if (pDevData->LongTermRequestList[index].boIsInProcessUse == TRUE)
			{
				u32 bytes = 4 * (wordCount+2);	 // Paket mit Header0/1

//				pr_devel(MODDEBUGOUTTEXT" Completing request %d, id %d\n",index,deviceID);
				trace_printk("Completing request %d, id %d\n",index,deviceID);

				//wenns in den Buffer passt, Daten einlesen
				if (bytes <= MAX_SUNPACKETSIZE)
				{
					memcpy(pDevData->LongTermRequestList[index].IRQBuffer, sun_packet, bytes);
					boPacketDataDone	= TRUE;
					pDevData->LongTermRequestList[index].IRQBuffer_anzBytes = bytes;
				}
				else
				{
					pDevData->LongTermRequestList[index].IRQBuffer_anzBytes = 0;
					printk(KERN_WARNING MODDEBUGOUTTEXT" LongTerm buffer to small\n");
				}

				//Ungültige DeviceID setzen da sie gekommen ist (Eintrag an sich bleibt gültig)
				//
				//Note: - weil nach dem up(), es sein kann das der UserThread in Locked_startlongtermread() ist, vor dem 
				// 		 der DPC, boIsInFPGA=false, gemacht hat, würde es zum Fehler kommen da die devID noch als benutzt markiert ist.
				//		- erst boIsInFPGA, dann up() geht nicht, da falls ein Abort() kommt und dann ein Locked_startlongtermread()
				//		 könnte es sein das der DPC danach ein up() macht
				//		- frei ist der LongTermRequestList[] erst nach dem up() daher immer (!boIsInFPGA && !boIsInProcessUse) == frei
				//		- nach down*()/Locked_startlongtermread() auf boIsInFPGA=false warten, extra taskwechsel und wie lange? bei viel DPC laod
				//		- device SEM, im DPC geht nicht wegen DPC
				pDevData->LongTermRequestList[index].DeviceID = MAX_IRQDEVICECOUNT;

				//den process aufwecken
				up(&pDevData->LongTermRequestList[index].WaitSem);

			}//end if boIsInProcessUse == true


			//Eintrag freigeben
			//	-> egal ob die DLL noch auf eine Anfrage wartet oder nicht
			//	das Paket fÃ¼r diese ID ist gekommen
			pDevData->LongTermRequestList[index].boIsInFPGA = FALSE;

			//eine Antwort für einen Thread
			//Note: Wichtig! es kann sein das der UserThread mit >der< devID einen neues read gestartet hat 
			break;
		}//end if DeviceID == [index].DeviceID
	}//end for MAX_LONG_TERM_IO_REQUEST

	return;
}

