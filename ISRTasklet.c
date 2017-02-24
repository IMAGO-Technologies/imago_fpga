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
static void InterruptTaskletSun(PDEVICE_DATA pDevData);


//<====================================>
//	IRQ fns
//
//<====================================>

//Schaltet im PCI-GerÃ¤t die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn)
{
	u32 regVal,regAdr;

	/* alles gut? */
	if( pDevData == NULL )
		return;
	if( (pDevData->pVABAR0 == NULL) || (pDevData->DeviceSubType == SubType_Invalid))
		return;

	/* CommonBuffer Adr setzen (nur gÃ¼ltig solange IRQon) & Init */
	//hat nur eine AGEX2(CL), MVC0
	if( boTurnOn && IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
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
		if( IS_TYPEWITH_PCI64BIT(pDevData->DeviceSubType) )
			iowrite32( (pDevData->pBACommonBuffer >> 32), pDevData->pVABAR0 + ISR_COMMONBUFFER_ADR_AGEX2 +4);
#endif		
	}


	//der DPC ist/ muss durch sein
	//(kann aber erst hier gesetzt werden da sonst race cond mit CommonBuffer Clear, IRQEnable und DPCFlag)
	if(boTurnOn)
		pDevData->boIsDPCRunning = FALSE;


	/* IRQ enable flag */
	//was schreiben
	if(boTurnOn)
		regVal = 0x1;
	else
		regVal = 0x0;

	//wo kommt das On/Off Bit hin?
	if( IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
		regAdr = ISR_ONOFF_OFFSET_AGEX2;
	else
		regAdr = ISR_ONOFF_OFFSET_AGEX;

	iowrite32(regVal, pDevData->pVABAR0 + regAdr);
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
	if( IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
			return IRQ_NONE;

		regVal = ((u32*)pDevData->pVACommonBuffer)[0];		//das Bit steht in uns Drin

		//bei nicht DMA nur 1. Bit (sicher ist sicher)
		if( !IS_TYPEWITH_DMA2HOST(pDevData->DeviceSubType) )
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
		if( !IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
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
void AGEXDrv_tasklet (unsigned long devIndex)
{
	PDEVICE_DATA pDevData = NULL;
	bool		boIsSunIRQ 		= FALSE;
	bool		boIsDMAReadIRQ 	= FALSE;

	u32 		DMARead_isDone = 0, DMARead_isOK = 0;
	u16			DMARead_BufferCounters[MAX_DMA_READ_DMACHANNELS*MAX_DMA_READ_CHANNELTCS];

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_tasklet (Minor:%lu)\n", devIndex);


	//Alles gut?
	if(devIndex >= MAX_DEVICE_COUNT)
		return;
	pDevData = &_ModuleData.Devs[devIndex];
	if( (pDevData->pVABAR0 == NULL) || (pDevData->DeviceSubType == SubType_Invalid) )
		return;
	if( IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
			return;
	}


	/* Was für ein IRQ? SUN und/oder DMA (bei AGEX gibt es nur SUN)*/
	/**********************************************************************/
	if (pDevData->DeviceSubType==SubType_AGEX)
		boIsSunIRQ = TRUE;

	if( IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
		u32 IRQReg_A = ((u32*)pDevData->pVACommonBuffer)[0];
		u32 IRQReg_B = ((u32*)pDevData->pVACommonBuffer)[1];

		if(	(IRQReg_A & 0x1) == 1 )
			boIsSunIRQ = TRUE;

		if ( IS_TYPEWITH_DMA2HOST(pDevData->DeviceSubType) )
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
			for(i = 0; i< (pDevData->DMARead_anzChannels+pDevData->DMARead_anzTCs); i++)
			{
				u8*	pCommonBuffer = (u8*)pDevData->pVACommonBuffer;
				pCommonBuffer += HOST_BUFFER_DMAREAD_COUNTER_OFFSET;
				pCommonBuffer += i*8;	//je 8Byte
				DMARead_BufferCounters[i] = *((u16*)pCommonBuffer);
			}

			//IRQ?
			if(DMARead_isDone != 0)
				boIsDMAReadIRQ = TRUE;
		}

	}


	/* die IRQs behandeln */
	/**********************************************************************/
	//SUNIRQ 
	if(boIsSunIRQ)
		InterruptTaskletSun(pDevData);

	//DMA (Read)
	// da wir eine copy der IRQFlags haben könnten die IRQs eigentlich wieder on sein
	// bringt aber nur was wenn wir einen extra tasklet hätten
	if(boIsDMAReadIRQ)
		AGEXDrv_DMARead_DPC(pDevData, DMARead_isDone, DMARead_isOK, DMARead_BufferCounters);



	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	pr_devel(MODDEBUGOUTTEXT" DPC done\n");

	return;
}

//wird mit FPGA-IRQ Off im IRQ-Tasklet aufgerufen, kümmert sich um das SUN-Paket, pDevData ist gültig
void InterruptTaskletSun(PDEVICE_DATA pDevData)
{
	u32 regVal, fifoLevel, header0, header1;
	u16 deviceID, wordCount, index;

	bool boPacketDataDone = FALSE;


	/* vom FPGA den PaketKopf einlesen/auswerten */
	/**********************************************************************/
	// FIFO-Fuellstand einlesen (AGEX hat ein FIFO)
	// bei AGEX2 steht an Pos 3 Header0 im CommonBuffer das Paket hat daher keine grÃ¶ÃŸe
	if( !IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
		// FIFO-Fuellstand einlesen
		regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);

		// im Bit 0 steht das Interrupt-Flag, danach der Fuellstand
		fifoLevel = (regVal >> 1) & 0x1FF;

		if (fifoLevel < 3){
			// sollte nicht auftreten
			printk(KERN_ERR MODDEBUGOUTTEXT" SUN Error: FIFO level to small: %d\n", fifoLevel);
			return;
		}
		pr_devel(MODDEBUGOUTTEXT" SUN FIFO level: %d\n", fifoLevel);
	}

	// Header0 und Header1 einlesen
	if( !IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType) )
	{
		header0 = ioread32(pDevData->pVABAR0);
		header1 = ioread32(pDevData->pVABAR0);
	}
	else
	{
		header0 = ((u32*)pDevData->pVACommonBuffer)[2+0];
		header1 = ((u32*)pDevData->pVACommonBuffer)[2+1];
	}


	wordCount = header1 & 0xFF;	//ist somit auch kleiner PAGE_SIZE
	deviceID = (header1>>20) & (MAX_IRQDEVICECOUNT-1);
	pr_devel(MODDEBUGOUTTEXT" SUN WordCount: %d\n", wordCount);


	/* lÃ¤uft noch eine Anfrage fÃ¼r die ID? */
	/**********************************************************************/
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		// DeviceID auswerten (warten wir auf das SUNPaket?)
		// 	-> DeviceID muss passen und wir haben die daten noch nicht reingefÃ¼llt
		if(		(pDevData->LongTermRequestList[index].boIsInFPGA == TRUE )
			&&	(pDevData->LongTermRequestList[index].DeviceID == deviceID) )
		{
			//Wartet (noch) ein Request aufs Paket?
			if(pDevData->LongTermRequestList[index].boIsInProcessUse == TRUE)
			{
				pr_devel(MODDEBUGOUTTEXT" Completing request %d, id %d\n",index,deviceID);

				//wenns in den Buffer passt, Daten einlesen
				if(MAX_SUNPACKETSIZE >= 4*wordCount+8) // Paket mit Header0/1
				{
					if( !IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType))
					{
						pDevData->LongTermRequestList[index].IRQBuffer[0] = header0;
						pDevData->LongTermRequestList[index].IRQBuffer[1] = header1;
						ioread32_rep(pDevData->pVABAR0,pDevData->LongTermRequestList[index].IRQBuffer+2,wordCount);
					}
					else
						memcpy(	pDevData->LongTermRequestList[index].IRQBuffer, ((u32*)pDevData->pVACommonBuffer) + 2, 4*wordCount+8);
					boPacketDataDone	= TRUE;
					pDevData->LongTermRequestList[index].IRQBuffer_anzBytes = 4*wordCount+8;
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



	/* Immer! egal ob sie einer will oder nicht die daten weglesen */
	/**********************************************************************/
	//Nur bei der AGEX, bei der AGEX2 wird beim IRQenablen der Buffer ungÃ¼ltig gemacht
	if (!boPacketDataDone && ( !IS_TYPEWITH_COMMONBUFFER(pDevData->DeviceSubType)) )
	{
		u16 i;
		volatile u32 dummy;
		for (i=0; i<wordCount; i++)
			dummy = ioread32(pDevData->pVABAR0);
	}

	return;
}

