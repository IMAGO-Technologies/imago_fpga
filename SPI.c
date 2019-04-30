/*
 * SPI.c
 *
 * SPI code
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

irqreturn_t imago_spi_interrupt(int irq, void *dev_id)
{
#if 1
	pr_devel(MODDEBUGOUTTEXT" imago_spi_interrupt> (IRQ:%d)\n", irq);
//	trace_printk("imago_spi_interrupt> (IRQ:%d)\n", irq);

	return IRQ_WAKE_THREAD;
#else

	PDEVICE_DATA pDevData = NULL;

	pr_devel(MODDEBUGOUTTEXT" imago_spi_interrupt> (IRQ:%d, devptr:%p)\n", irq, dev_id);

	/* war der int von uns? reg lesen  */
	if( dev_id == NULL )
		return IRQ_NONE;
	pDevData = (PDEVICE_DATA) dev_id;
	if (pDevData->DeviceSubType == SubType_Invalid)
		return IRQ_NONE;

	/* läuft der DPC noch 
	 * (damit wir nicht bei fremd HWIs, nicht falsch weil doppelt<>Flag im CommonBuffer, denken er währe unser)
	 * der DPC könnte noch laufen*/
	if (pDevData->boIsDPCRunning==TRUE)
		return IRQ_NONE;

	//merken das gleich der DPC läuft
	/* kann keine 2 HWIs zur gleichen Zeit geben, aber der 
	 * DPC(eigentlich nicht da der auf der selben CPU ausgeführt wird)
	 * könnte durch sein vor dem das die HWI durch ist*/
	pDevData->boIsDPCRunning=TRUE;

	// trigger the tasklet (sollte immer gehen)
	tasklet_schedule(&pDevData->IRQTasklet);

	return IRQ_HANDLED;
#endif
}

irqreturn_t imago_spi_thread(int irq, void *dev_id)
{
	AGEXDrv_tasklet((unsigned long)dev_id);
	
	return IRQ_HANDLED;
}

//struct mit den file fns
struct file_operations AGEXDrv_SPI_fops = {
	.owner	= THIS_MODULE,
	.open	= AGEXDrv_open,
	.read	= AGEXDrv_read,
	.write	= AGEXDrv_write,
	.unlocked_ioctl = AGEXDrv_unlocked_ioctl,
	.llseek = no_llseek,
};

static const struct of_device_id imago_spi_of_match[] = {
	{
		.compatible	= "imago,fpga-spi-daytona",
		.data		= (void *)SubType_DAYTONA,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imago_spi_of_match);

static const struct spi_device_id imago_spi_id[] = {
        {"fpga-spi-daytona", SubType_DAYTONA},
        {}
};
MODULE_DEVICE_TABLE(spi, imago_spi_id);

int imago_spi_probe(struct spi_device *spi)
{
	int res,i, DevIndex;
	u8 tempDevSubType;
	const struct of_device_id *of_id;

	pr_devel(MODDEBUGOUTTEXT" imago_spi_probe\n");

	of_id = of_match_device(imago_spi_of_match, &spi->dev);
	if (of_id)
		tempDevSubType = (unsigned long)of_id->data;
	else
		tempDevSubType = spi_get_device_id(spi)->driver_data;

	if (tempDevSubType != SubType_DAYTONA) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" invalid device identifier (%u)\n", tempDevSubType);
		return -EINVAL;
	}

	pr_devel(MODDEBUGOUTTEXT" found '%s' device\n", AGEXDrv_device_info[tempDevSubType].name);

	if (!spi->irq) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" missing interrupt configuration\n");
		return -EINVAL;
	}


	//>freie Minor Nummer?
	/**********************************************************************/
	DevIndex =-1;
	for(i=0; i<MAX_DEVICE_COUNT; i++) {
		if(!_ModuleData.boIsMinorUsed[i]) {
			DevIndex = i;
			AGEXDrv_InitDrvData(&_ModuleData.Devs[DevIndex]);			
			_ModuleData.Devs[DevIndex].DeviceSubType= tempDevSubType;
			_ModuleData.Devs[DevIndex].DeviceNumber = MKDEV(MAJOR(_ModuleData.FirstDeviceNumber), DevIndex);
			_ModuleData.Devs[DevIndex].pDeviceDevice = &spi->dev;
			_ModuleData.Devs[DevIndex].flags =  AGEXDrv_device_info[tempDevSubType].flags;
			spi_set_drvdata(spi, &_ModuleData.Devs[DevIndex]);
			break;
		}
	}
	if(DevIndex==-1){
		printk(KERN_WARNING MODDEBUGOUTTEXT" no free Minor-Number found!\n"); return -EINVAL;}
	else
		pr_devel(MODDEBUGOUTTEXT" use major/minor (%d:%d)\n", MAJOR(_ModuleData.Devs[DevIndex].DeviceNumber), MINOR(_ModuleData.Devs[DevIndex].DeviceNumber));


	//>IRQ & tasklet
	/**********************************************************************/

#if 1
	if (request_threaded_irq(spi->irq, NULL/*imago_spi_interrupt*/, imago_spi_thread,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, MODMODULENAME, &_ModuleData.Devs[DevIndex]) != 0) {
#else
	//tasklet
	tasklet_init(&_ModuleData.Devs[DevIndex].IRQTasklet, AGEXDrv_tasklet, DevIndex);

	if (request_irq(spi->irq,/* die IRQ nummer */
			imago_spi_interrupt,	/* die IRQ fn */
			IRQF_SHARED,		/* shared */
			MODMODULENAME,		/* name wird in /proc/interrupts angezeigt */
			&_ModuleData.Devs[DevIndex]	/* unique identifier/ wird auch dem CallBack mit gegeben */
			) != 0) {
#endif
		printk(KERN_ERR MODDEBUGOUTTEXT" request_irq failed\n");
		_ModuleData.Devs[DevIndex].boIsIRQOpen = FALSE;
		return -EIO;
	}

	AGEXDrv_SwitchInterruptOn(&_ModuleData.Devs[DevIndex], TRUE);

	pr_devel(MODDEBUGOUTTEXT" IRQ> %d \n", spi->irq);
	_ModuleData.Devs[DevIndex].boIsIRQOpen = TRUE;


	//>dev init & fügt das es hinzu
	/**********************************************************************/
	cdev_init(&_ModuleData.Devs[DevIndex].DeviceCDev, &AGEXDrv_SPI_fops);
	_ModuleData.Devs[DevIndex].DeviceCDev.owner = THIS_MODULE;
	_ModuleData.Devs[DevIndex].DeviceCDev.ops 	= &AGEXDrv_SPI_fops;	//notwendig in den quellen wird fops gesetzt?

	//fügt ein device hinzu, nach der fn können FileFns genutzt werden
	res = cdev_add(&_ModuleData.Devs[DevIndex].DeviceCDev, _ModuleData.Devs[DevIndex].DeviceNumber, 1/*wie viele ab startNum*/);
	if(res < 0)
		printk(KERN_WARNING MODDEBUGOUTTEXT" can't add device!\n");
	else
		_ModuleData.Devs[DevIndex].boIsDeviceOpen = TRUE;


	//> in Sysfs class eintragen
	/**********************************************************************/			
	//war mal class_device_create
	if (!IS_ERR(_ModuleData.pModuleClass)) {		
		char devName[128];
		struct device *temp;

		sprintf(devName, "%s%d", MODMODULENAME, MINOR(_ModuleData.Devs[DevIndex].DeviceNumber));
		temp = device_create(
				_ModuleData.pModuleClass, 	/* die Type classe */
				NULL, 			/* pointer zum Eltern, dann wird das dev ein Kind vom parten*/
				_ModuleData.Devs[DevIndex].DeviceNumber, /* die nummer zum device */
				NULL,
				devName			/*string for the device's name */
				);

		if( IS_ERR(temp))
			printk(KERN_WARNING MODDEBUGOUTTEXT" can't create sysfs device!\n");
	}


	// init von allem ist durch
	printk(KERN_INFO MODDEBUGOUTTEXT" SPI probe done\n");
	_ModuleData.boIsMinorUsed[DevIndex] = TRUE;

	return 0;
}


int imago_spi_remove(struct spi_device *spi)
{
	PDEVICE_DATA pDevData = (PDEVICE_DATA)spi_get_drvdata(spi);

	pr_devel(MODDEBUGOUTTEXT" imago_spi_remove\n");

	if (pDevData == NULL) {
		printk(KERN_WARNING MODDEBUGOUTTEXT" imago_spi_remove> device data is invalid!\n");
		return -ENODEV;
	}

	//IRQ zuückgeben
	if(pDevData->boIsIRQOpen) {
		AGEXDrv_SwitchInterruptOn(pDevData, FALSE);
		free_irq(spi->irq, pDevData);
	}
	
	//"... this function busy-waits until the tasklet exits..."
//	tasklet_disable(&pDevData->IRQTasklet);

	//device in der sysfs class löschen
	if (!IS_ERR(_ModuleData.pModuleClass))	
		device_destroy(_ModuleData.pModuleClass, pDevData->DeviceNumber);

	//device löschen
	if(pDevData->boIsDeviceOpen)
		cdev_del(&pDevData->DeviceCDev);
	pDevData->boIsDeviceOpen = FALSE;

	return 0;
}

struct spi_driver imago_spi_driver = {
	.driver = {
		.name	= "imago-fpga-spi",
		.of_match_table = of_match_ptr(imago_spi_of_match),
	},
	.id_table	= imago_spi_id,
	.probe		= imago_spi_probe,
	.remove		= imago_spi_remove,
};
