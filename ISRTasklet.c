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


//<====================================>
//	IRQ fns
//
//<====================================>

//Schaltet im PCI-Gerät die Ints ab/zu
void AGEXDrv_SwitchInterruptOn(PDEVICE_DATA pDevData, const bool boTurnOn)
{
	u32 regVal,regAdr;

	/* alles gut? */
	if( pDevData == NULL )
		return;
	if( (pDevData->pVABAR0 == NULL) || (pDevData->DeviceSubType == SubType_Invalid))
		return;

	/* CommonBuffer Adr setzen (nur gültig solange IRQon) & Init */
	//hat nur eine AGEX2
	if( boTurnOn && ( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_MVC0) ) )
	{
		//nur um ganz sicher zu sein
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
		  	return;

		//das IRQ Flag löschen (1 Word, 4 Word sind nur zu Sicherheit)
		memset(pDevData->pVACommonBuffer,0, (2+3) + sizeof(u32));
		smp_mb();	//nop bei x86

		//Adr ins FPGA setzen
		iowrite32(pDevData->pBACommonBuffer, pDevData->pVABAR0 +ISR_COMMONBUFFER_ADR_AGEX2);
	}


	/* IRQ enable flag */
	//was schreiben
	if(boTurnOn)
		regVal = 0x1;
	else
		regVal = 0x0;

	//wo kommt das On/Off Bit hin?
	if( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_MVC0) )
		regAdr = ISR_ONOFF_OFFSET_AGEX2;
	else
		regAdr = ISR_ONOFF_OFFSET_AGEX;

	iowrite32(regVal, pDevData->pVABAR0 + regAdr);
}



//< HWIRQ > nur prüfen ob von uns, ja tasklet starten
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


	/* liest das IRQ flag ein */
	if( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_MVC0) )
	{
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
			return IRQ_NONE;

		regVal = ((u32*)pDevData->pVACommonBuffer)[0];		//das Bit steht in uns Drin
	}
	else
	{
		//das Flag geht weg wenn FIFO leer oder IRQs off sind
		regVal = ioread32(pDevData->pVABAR0 + ISR_AVAILABLE_OFFSET);
	}


	/* ja, unser interrupt */
	if(regVal & 0x1)
	{
		//INTs abschalten
		//AGEX: IRQ geht weg wenn FIFO leer oder IRQs off sind
		//AGEX2: automatisch
		if(pDevData->DeviceSubType == SubType_AGEX)
			AGEXDrv_SwitchInterruptOn(pDevData, FALSE);

		// trigger the tasklet
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
	u32 regVal, fifoLevel, header0, header1;
	u16 deviceID, wordCount, index;
	PDEVICE_DATA pDevData = NULL;
	bool boPacketDataDone = FALSE;

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_tasklet (Minor:%lu)\n", devIndex);


	//Alles gut?
	if(devIndex >= MAX_DEVICE_COUNT)
		return;
	pDevData = &_ModuleData.Devs[devIndex];
	if( (pDevData->pVABAR0 == NULL) || (pDevData->DeviceSubType == SubType_Invalid) )
		return;
	if( (pDevData->DeviceSubType==SubType_AGEX2) || (pDevData->DeviceSubType==SubType_MVC0) )
	{
		if( (pDevData->pVACommonBuffer == NULL) || (pDevData->pBACommonBuffer == 0) )
			return;
	}


	/* vom FPGA den PaketKopf einlesen/auswerten */
	/**********************************************************************/
	// FIFO-Fuellstand einlesen (AGEX hat ein FIFO)
	// bei AGEX2 steht an Pos 3 Header0 im CommonBuffer das Paket hat daher keine größe
	if(pDevData->DeviceSubType == SubType_AGEX)
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
	if(pDevData->DeviceSubType == SubType_AGEX)
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


	/* läuft noch eine Anfrage für die ID? */
	/**********************************************************************/
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		// DeviceID auswerten (warten wir auf das SUNPaket?)
		// 	-> DeviceID muss passen und wir haben die daten noch nicht reingefüllt
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
					if(pDevData->DeviceSubType == SubType_AGEX)
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

				//den process aufwecken
				up(&pDevData->LongTermRequestList[index].WaitSem);

			}//end if boIsInProcessUse == true


			//Eintrag freigeben
			//	-> egal ob die DLL noch auf eine Anfrage wartet oder nicht
			//	das Paket für diese ID ist gekommen
			pDevData->LongTermRequestList[index].boIsInFPGA = FALSE;

		}//end if DeviceID == [index].DeviceID
	}//end for MAX_LONG_TERM_IO_REQUEST



	/* Immer! egal ob sie einer will oder nicht die daten weglesen */
	/**********************************************************************/
	//Nur bei der AGEX, bei der AGEX2 wird beim IRQenablen der Buffer ungültig gemacht
	if (!boPacketDataDone && (pDevData->DeviceSubType == SubType_AGEX) )
	{
		u16 i;
		volatile u32 dummy;
		for (i=0; i<wordCount; i++)
			dummy = ioread32(pDevData->pVABAR0);
	}


	/* Re-enable the interrupt */
	/**********************************************************************/
	AGEXDrv_SwitchInterruptOn(pDevData,TRUE);

	return;
}

