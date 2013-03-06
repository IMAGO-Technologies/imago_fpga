/*
 * LockedOps.c
 *
 * -> In dieser Datei darf sich immer nur ein Thread/process befinden
 * -> die args müssen soweit wie möglich gültig sein
 *
 *  Created on: 23.12.2011
 *      Author: imago
 */

#include "AGEXDrv.h"

char 				pBuildTime[] = {__DATE__ " - " __TIME__};
char 				pVersion[] = {"1.0.1.0"};




//führt die sucht einen freien eintrag und belegt diesen, gibt >= 0 für OK sonst fehler code zurück,
// -> Fehler, wenn eine Anfrage zur ID offen ist
// -> Fehler, wenn kein Platz mehr ist
// dass die DeviceID geöffnet wird nicht überprüft, ist für den Treiber Egal
long Locked_startlongtermread(const u32 DeviceID)
{
	u32 index;

	/* Es darf zu dieser Zeit keine Anfrage mit der übergebenen ID offen sein */
	//(über die ID erfolgt die Zuordnung IRQ <-> Request)
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		( (_LongTermRequestList[index].boIsInFPGA == TRUE) || (_LongTermRequestList[index].boIsInProcessUse == TRUE) )
			&&	(_LongTermRequestList[index].DeviceID == DeviceID) )
		{
			return -EBUSY;
		}
	}

	/* Ist noch Platz für noch eine Anfrage? */
	for(index=0; index<MAX_LONG_TERM_IO_REQUEST; index++)
	{
		if(		(_LongTermRequestList[index].boIsInFPGA == FALSE)
			&& 	(_LongTermRequestList[index].boIsInProcessUse == FALSE))
		{
			//init des eintrags
			_LongTermRequestList[index].IRQBuffer_anzBytes =0;
			_LongTermRequestList[index].DeviceID = DeviceID;
			while( down_trylock(&_LongTermRequestList[index].WaitSem) == 0){}		//die sem runter zählen bis sie blocked

			//Eintrag wird genutzt
			_LongTermRequestList[index].boIsInProcessUse= TRUE;
			_LongTermRequestList[index].boIsInFPGA		= TRUE;		//vor dem Write setzen, kann ja sein das gleich nach dem write ein IRQ kommt

			return index;
		}
	}


	return -EBUSY;
}

//führt die WriteOp aus, gibt >= 0 für OK sonst fehler code zurück,
long Locked_write(const u8 __user * pToUserMem, const size_t BytesToWrite)
{
	u8 TempBuffer[MAX_SUNPACKETSIZE];

	/* User Data -> Kernel */
	if(BytesToWrite > MAX_SUNPACKETSIZE)
		return -EFBIG;
	if( copy_from_user (TempBuffer, pToUserMem, BytesToWrite) != 0){
		printk(KERN_WARNING "agexdrv: Locked_write> copy_from_user faild\n");
		return -EFAULT;
	}



	/* Kernel -> PCI */
	if (BytesToWrite % 4)
		iowrite8_rep(_PCI_IOMEM_StartAdr, TempBuffer, BytesToWrite);
	else
		iowrite32_rep(_PCI_IOMEM_StartAdr, TempBuffer, BytesToWrite/4);

	return BytesToWrite;
}

//führt eine IOOp durch, gibt >= 0 für OK sonst fehler code zurück,
long Locked_ioctl(const u32 cmd, u8 __user * pToUserMem, const u32 BufferSizeBytes)
{
	long result=0;

	switch(cmd)
	{
		/* Gibt die Version als String zurück */
		case AGEXDRV_IOC_GET_VERSION:
			if( sizeof(pVersion) > BufferSizeBytes){
				printk(KERN_WARNING "agexdrv: Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,pVersion,sizeof(pVersion)) !=0 ){
					printk(KERN_WARNING "agexdrv: Locked_ioctl> copy_to_user faild\n");
					result = -EFAULT;
				}
				else
					result = sizeof(pVersion);
			}

			break;


		/* Gibt das Build date/time als String zurück */
		case AGEXDRV_IOC_GET_BUILD_DATE:
			if( sizeof(pBuildTime) > BufferSizeBytes){
				printk(KERN_WARNING "agexdrv: Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				if( copy_to_user(pToUserMem,pBuildTime,sizeof(pBuildTime)) !=0 ){
					printk(KERN_WARNING "agexdrv: Locked_ioctl> copy_to_user faild\n");
					result = -EFAULT;
				}
				else
					result = sizeof(pBuildTime);
			}

		break;


		/* Markiert die DeviceID als frei*/
		//	-> kein Fehler wenn ungenutzt oder noch im LongTermRequest noch OutOfRange
		case AGEXDRV_IOC_RELEASE_DEVICEID:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING "agexdrv: Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				//lesen
				u8 DeviceID;
				if( get_user(DeviceID,pToUserMem) != 0){
					printk(KERN_WARNING "agexdrv: Locked_ioctl> get_user faild\n");
					result = -EFAULT;
					break;
				}

				//gültig?
				if(DeviceID >= MAX_IRQDEVICECOUNT)
					printk(KERN_WARNING "agexdrv: Locked_ioctl> DeviceID out of range!\n");
				else
				{
					//nur zum Tracen
					printk(KERN_DEBUG "agexdrv: Locked_ioctl> Release DeviceID = %d, was %s\n",
								   DeviceID, (_boIsDeviceIDUsed[DeviceID])?("true"):("false") );

					//freigeben, immer egal was für ein Zustand war
					_boIsDeviceIDUsed[DeviceID] = FALSE;
				}
			}

			break;


		/* Gibt eine neue DeviceID zurück */
		// -> nur wenn sie zur Zeit als frei markiert ist boIsDeviceIDUsed[]==false
		// -> nur wenn sie nicht im LongTermRequestList[] ist
		// 		dass kann passieren wenn die DLL eine LongTermAnfrage startet und dann das device zu macht
		// 		die Anfrage ist dann noch unterwegs, ja der Request wurde abgebrochen aber die DeviceID ist
		// 		noch genutzt
		case AGEXDRV_IOC_CREATE_DEVICEID:
			if( BufferSizeBytes != 1){
				printk(KERN_WARNING "agexdrv: Locked_ioctl> Buffer Length to short\n");
				result = -EFBIG;
			}
			else
			{
				s16 NewDeviceID = -1;
				u8 iLongTerm;
				u16 index;

				//alle möglichen IDs testen
				for(index=0; index < MAX_IRQDEVICECOUNT; index++)
				{
					//benutzt?
					if(_boIsDeviceIDUsed[index])
						continue;

					//läuf noch ein LongTempRequest mit der ID?
					NewDeviceID = index;
					for(iLongTerm=0; iLongTerm<MAX_LONG_TERM_IO_REQUEST; iLongTerm++)
					{
						if(		( (_LongTermRequestList[iLongTerm].boIsInFPGA == TRUE) || (_LongTermRequestList[iLongTerm].boIsInProcessUse == TRUE) )
							&&	(_LongTermRequestList[iLongTerm].DeviceID == NewDeviceID) )
						{
							//wenn noch offen war rücksetzen, weiter suchen
							NewDeviceID = -1;
						}
					}

					//neue ID gefunden?
					if(NewDeviceID != -1)
					{
						//speichern
						if( put_user(NewDeviceID,pToUserMem) != 0){
							printk(KERN_WARNING "agexdrv: Locked_ioctl> put_user faild\n");
							result = -EFAULT;
							break;
						}
						result = 1;

						//marken
						_boIsDeviceIDUsed[NewDeviceID] = TRUE;

						break;
					}
				}

				//Fehler?
				if( NewDeviceID == -1){
					printk(KERN_WARNING "agexdrv: Locked_ioctl> No free DeviceID\n");
					result = -EMFILE;
				}
				else
					printk(KERN_DEBUG "AgeXEvtIoDeviceControl NewDeviceID = %d\n", NewDeviceID );
			}

			break;




		//sollte nie sein (siehe oben bzw. ebene)
		default:
			return -ENOTTY;
	}


	return result;
}
