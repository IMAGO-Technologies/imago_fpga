/*
 * DMARead.c
 *
 * Device to PC Code
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
 *
 */

#include "AGEXDrv.h"

static void AGEXDrv_DMARead_EndDMA(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC, const bool boIsOk, const u16 BufferCounter);
static u8* AGEXDrv_DMARead_GetSGAdr(PDEVICE_DATA pDevData,const u32 iDMA, const u32 iTC);



/****************************************
 * 
 * Helpers
 *
****************************************/

//Init und mapped/pinned den "Job<>UserBuffer", struct ist beim return(min die Flags g�ltig) wickelt daher beim Fehler nichts r�ck ab (kann nicht als DPC laufen)
bool AGEXDrv_DMARead_MapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob, uintptr_t pVMUser, u64 bufferSize)
{
	u32				anzPagesToMap	= (bufferSize + PAGE_SIZE-1) / PAGE_SIZE;
	int 			pagesPinned		= -1;

	int 			anzMappedSGs	= -1;

	pr_devel(MODDEBUGOUTTEXT" MappUserBuffer> (%d[Bytes], %d[Pages] @ 0x%p)\n", (int)bufferSize, anzPagesToMap, (void*)pVMUser);


	//init der Flags(da hier kein cleanUp gemacht wird)
	pJob->pVMUser 				= (uintptr_t) pVMUser;
	pJob->bufferSize		 	= bufferSize;

	pJob->boIsOk				= false;/* don't care, da nur g�ltig wenn in .Jobs_Done */
	pJob->BufferCounter			= 0;	/* don't care, da nur g�ltig wenn in .Jobs_Done */

	pJob->boIsPageListValid 	= FALSE;
	pJob->boIsPinned			= FALSE;
	pJob->boIsSGValid			= FALSE;
	pJob->boIsSGMapped			= FALSE;

	pJob->SGItemsLeft			= 0;


	//Valid? (hier kein Test ob "Offset=0 & size n * PAGES_SIZE" nur wichtig f�rs alloc
	if( (pDevData==NULL) || (pJob==NULL) )
 		{printk(KERN_ERR MODDEBUGOUTTEXT "MappUserBuffer> invalid args!\n"); return FALSE;}
	if( 	((pJob->bufferSize & 0x3) != 0)
		|| 	(pJob->bufferSize <= 4) )
 		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> byte count is wrong!\n"); return FALSE;}
	if( (pJob->pVMUser & (PAGE_SIZE-1) ) != 0)
 		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> page pointer is wrong aligned!\n"); return FALSE;}


	//> User Buffer Pinnen
	/**********************************************************************/
	//speicher f�r die PageList
	pJob->ppPageList = kmalloc(anzPagesToMap*sizeof(struct page*), GFP_KERNEL);
	if( pJob->ppPageList == NULL )
 		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> too many pages!\n"); return FALSE;}
	pJob->boIsPageListValid = TRUE;

	//pinnen
	//muss die SEM, f�r die VMAs f�rr den aufrufenden conntext, halten
	// 'for read or write' ist eine 'rw_semaphore'
	down_read(&current->mm->mmap_sem);
//----------------------------->

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
// 4.8.17 >>> 4.9.0 (Oct 2016)
//	Linux 4.9-rc2(https://lwn.net/Articles/704380/): Lorenzo Stoakes (10): 			mm: replace get_user_pages() write/force parameters with gup_flags
//					http://lists.openwall.net/netdev/2016/10/13/1
//					https://marc.info/?l=linux-mm&m=147585445805166
//		bei 4.8.17 get_user_pages() 	> 	__get_user_pages_locked()  da wurde dann aus if(write) flags |= FOLL_WRITE
//
	pagesPinned = get_user_pages(pJob->pVMUser, anzPagesToMap, FOLL_WRITE, pJob->ppPageList, NULL);	
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
// 4.5.7 >>> 4.6.0 (Feb 2016)
//	https://github.com/torvalds/linux/commit/d4edcf0d56958db0aca0196314ca38a5e730ea92#diff-c098b65a8bd8c7db23377b90578a62c1  
//		Ingo Molnar  mm/gup: Switch all callers of get_user_pages() to not pass tsk/mm 
//
//		/mm/gup.c get_user_pages()
//			"This is the same as get_user_pages_remote(), just with a
//			 less-flexible calling convention where we assume that the task
//			 and mm being operated on are the current task's."
//	
	pagesPinned = get_user_pages(pJob->pVMUser, anzPagesToMap, TRUE, FALSE, pJob->ppPageList, NULL);
#else
	pagesPinned = get_user_pages(
		current, 		/* task_struct, wo sollen die 'page faults' hin */
		current->mm,	/* mm_struct, in welcher VMA der virtuelle Speicher zu finden ist */
		pJob->pVMUser,	/* UserMode Pointer, muss page-aligned sein */
		anzPagesToMap,	/* anz Pages */
		TRUE,			/* 1<>write&read, 0<> readOnly (f�r den fn caller [module]) */
		FALSE,			/* kein force, daher aus ein ReadOnly wird kein RW, 'LDD3 driver should always 0 here' */
		pJob->ppPageList, /* NULL, oder PointerFeld zu den Pages welches anzPages/Pointer halten kann, gef�llte anz ist result */
		NULL);			/* NULL, oder PointerFeld zu den VMAs welche anzPages/Pointer haltern kann */
#endif	
//<-----------------------------
	up_read(&current->mm->mmap_sem);

	if(pagesPinned <= 0)
 		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> get_user_pages() failed!\n"); return FALSE;}
	pJob->boIsPinned 		= TRUE;
	pJob->pagesPinned 	= pagesPinned;
	if (((u32)pagesPinned) != anzPagesToMap) {
		printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> get_user_pages() failed %d from %d pinned!\n", pagesPinned, anzPagesToMap);
		return FALSE;
	}



	//> SG(scatter/gather) Liste erzeugen
	/**********************************************************************/
	//Note:
	// bis 1.1.9.0 (Anfang 2018) wurde sg_alloc_table_from_pages() [wenn m�glich] benutzt 
	//  * das zeitliche Verhalten ist nicht vorhersagbar (von alles 4k bis eine SG mit voller BildSize)
	//  * wir haben aber nur 20Bit im FPGA f�r den DWordCount eines SGElemnts [bei der Zeile kann es knapp werden]
	//  * unter WIN32 kein Problem, da max 1MByte pro Transfer (n Transfers pro Bild) siehe WDF_DMA_ENABLER_CONFIG_INIT()
	//
	//https://www.kernel.org/doc/Documentation/DMA-API.txt
	// "... The implementation is free to merge several consecutive sglist entries ..."
	//http://www.gossamer-threads.com/lists/linux/kernel/977965
	// "scatterlist.c or dma_map_sg() should coalesce?"
	// "No, it's not the job of dma_map_sg."
	//
	//aber ab =>3.6 gibt es "sg_alloc_table_from_pages()" was es macht
	//http://lists.freedesktop.org/archives/dri-devel/2012-April/021962.html	
	// "...All contiguous chunks of the pages are merged into a single sg nodes."
	//
	//http://askubuntu.com/questions/318315/how-can-i-temporarily-disable-aslr-address-space-layout-randomization
	//"/proc/sys/kernel/randomize_va_space" kann eingestellt werden ob Speicher zusammenh�ngen darf (0=off,2=on), bei 3.16.7 (max 16k[on], ?64[off])
	//
	//- init des "Headers" <> sg_table, und alloced n pages f�r je m scatterlists 
	// wird mehr als eine page gebraucht SG_MAX_SINGLE_ALLOC wird in struct scatterlist.page_link
	// das bit 0 gesetzt, dann ist es eine Pointer auf die n�chste page 
	// daher nicht selbst durchlaufen
	//siehe: http://lwn.net/Articles/256368/ (The chained scatterlist API)
//#if LINUX_VERSION_CODE > KERNEL_VERSION(3,6,0)
//	if( 0 != sg_alloc_table_from_pages(	&pJob->SGTable,				/*header*/
//										pJob->ppPageList,			/*pointer to page array*/
//										pagesPinned,				/*number of pages in page array*/
//										0, 							/*buffer offset*/
//										pJob->bufferSize			/*buffer size [bytes]*/,
//										GFP_KERNEL))				/*wie wird gealloc*/
//		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> sg_alloc_table() failed!\n"); return FALSE;}	
//	pJob->boIsSGValid 	= TRUE;
//	pJob->pSGNext		= pJob->SGTable.sgl;
//#else
{
	struct scatterlist *ptmpSGList	= NULL;
	u32 anztmpBytes = pJob->bufferSize;
	s32	iSG							= 0;
	if(0 != sg_alloc_table(	&pJob->SGTable	/*header*/, 
							pagesPinned 	/*f�r wie viele Eintr�ge*/, 
							GFP_KERNEL))	/*wie wird die page gealloc*/
		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> sg_alloc_table() failed!\n"); return FALSE;}	
	pJob->boIsSGValid 	= TRUE;
	pJob->pSGNext		= pJob->SGTable.sgl;

	//- jetzt die Pages adden (macht nix mit den daten/pages)
	ptmpSGList = pJob->SGTable.sgl;
	for(iSG=0; iSG<pagesPinned; iSG++)
	{
		u32 tmp = MIN(anztmpBytes, PAGE_SIZE);		
		anztmpBytes -= tmp;
		sg_set_page(ptmpSGList,				/*an diese Stelle wird die Page eingetragen*/
					pJob->ppPageList[iSG]	/*Pointer zur page struct*/,
					tmp,					/*anzBytes der daten*/
					0);						/*Offset*/
		//Achtung! >nicht< einfach sgTable.sgl[Index] oder pSGList++
		//da sie gechained sein k�nnten
		ptmpSGList = sg_next(ptmpSGList);
	}
}
//#endif	


	//> in den PCI/BUS AdrRaum mappen
	/**********************************************************************/
	//macht: 	kmmemcheck_mark_initialiued() > mark_shadow() touch der Page
	//			get_dma_ops(struct device)->map_sg()
	//*_map_sg 
	//	lxr.free-electrons.com/source/drivers/iommu/amd_iommu.c 			2865 (3.8)
	//  lxr.free-electrons.com/source/drivers/iommu/intel-iommu.c			3109 (3.8)
	//	lxr.free-electrons.com/source/arch/arm/mm/dma-mapping.c?v=3.8;a=arm 1418 (3.8)
	// geben (ohne IOMMU) einfach die phys adr zur�ck, ARM k�mmert sich um CACHE (x86 nicht notwendig)
	// bei fehlern machen sie alles r�ckg�ngig
	//
	// Achtung!
	//  beim arm64 v4.14.16 (noch nicht bei v4.1.8) fasst die fn SG elemente zusammen
	// https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	// 	"The implementation is free to merge several consecutive sglist entries into one..."
	//
	// Aber das vorzeitige "Ende" markieren mit sg_mark_end() geht nicht! 
	//" The chained scatterlist API"
	//https://lwn.net/Articles/256368/
	// "...Should the I/O operation not use all of the entries which were allocated in the list, though, the driver should mark the final segment with... 
	//
	// sg_mark_end()
	//https://elixir.bootlin.com/linux/v4.16/source/include/linux/scatterlist.h
	// ".. Marks the passed in sg entry as the termination point for the sg 
	//  table. A call to sg_next() on this entry will return NULL."
	//
	//  >> weil in https://elixir.bootlin.com/linux/v4.16/source/drivers/iommu/dma-iommu.c#L792
	//   for_each_sg(sg_next(sg), tmp, nents - 1, i) ... sg_next() wird dann mit NULL aufgerufen da nents die volle l�nge ist 
	//
	anzMappedSGs = dma_map_sg(pDevData->dev,	/*struct device pointer*/	
								pJob->SGTable.sgl,		/*struct scatterlist (Anfang)*/
								pJob->SGTable.nents,	/*anz der Buffers*/
								DMA_FROM_DEVICE); 		/*die Richtung wichtig f�r cache & bounce buffer*/
	if( anzMappedSGs <=0)
 		{printk(KERN_WARNING MODDEBUGOUTTEXT "MappUserBuffer> dma_map_sg() failed!\n"); return FALSE;}
	pJob->boIsSGMapped 	= TRUE;
	pJob->SGItemsLeft	= anzMappedSGs;

	pr_devel(MODDEBUGOUTTEXT" - dma_map_sg() use %d from %d SGs\n", anzMappedSGs, pJob->SGTable.nents);
	pr_devel(MODDEBUGOUTTEXT" mapped done\n");

	return TRUE;
}



// Macht einen CleanUp des Jobs
// Achtung! 
// 	> es k�nnen nur Teile eines Jobs g�ltig sein
//  > darf nicht in einem DPC laufen! (set_page_dirty_lock())
// - unmapping/pinnen 
// - TC/Job freigeben
void AGEXDrv_DMARead_UnMapUserBuffer(PDEVICE_DATA pDevData, PDMA_READ_JOB pJob)
{

	u32 			iPage			= 0;
	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_DMARead_UnMapUserBuffer\n");
	if( (pDevData==NULL) || (pJob==NULL) )
 		{printk(KERN_ERR MODDEBUGOUTTEXT "UnMapUserBuffer> invalid args!\n"); return;}

	//unmapping/pinnen 
	/**********************************************************************/
	//- unmappen
	//https://www.kernel.org/doc/Documentation/DMA-API-HOWTO.txt
	// "PLEASE NOTE:The 'nents' argument to the dma_unmap_sg call must be
	//       		the _same_ one you passed into the dma_map_sg call,
	//	      		it should _NOT_ be the 'count' value _returned_ from the dma_map_sg call."
	//
	// " After the last DMA transfer call one of the DMA unmap routines
	//	 dma_unmap_{single,sg}. If you don't touch the data from the first dma_map_*
	//	 call till dma_unmap_*, then you don't have to call the dma_sync_* routines at all."
	//
	//lxr.free-electrons.com/source/arch/arm/mm/dma-mapping.c?v=3.8;a=arm 1500 (3.8)
	// 	__iommu_unmap_sg() k�mmert sich um den cache
	//https://www.kernel.org/doc/Documentation/DMA-API.txt
	// "All the parameters must be the same as those and passed in to the scatter/gather mapping API."
	if(pJob->boIsSGMapped)
	{
		dma_unmap_sg(pDevData->dev,				/*struct device pointer*/
				pJob->SGTable.sgl,				/*struct scatterlist (Anfang)*/
				pJob->SGTable.nents,			/*nents, items of the list*/
				DMA_FROM_DEVICE);				/*Richtung der DMA*/
	}
	pJob->boIsSGMapped = FALSE;

	//- freen der scatterlisten
	if(pJob->boIsSGValid)
		sg_free_table(&pJob->SGTable);
	pJob->boIsSGValid = FALSE;

	//- unpinn Pages
	if(pJob->boIsPinned)
	{
		//�ber alle Pages gehen	
		for(iPage=0; iPage < pJob->pagesPinned; iPage++)
		{
			//die Page als ver�ndert marken
			//https://www.kernel.org/doc/htmldocs/kernel-api/API-get-user-pages.html
			// "..If the page is written to, set_page_dirty* must be called after the page is
			// finished with, and before put_page is called.."
			//
			//http://www.gossamer-threads.com/lists/linux/kernel/596302?do=post_view_threaded#596302
			//http://comments.gmane.org/gmane.linux.kernel/275722
			// ".. If you don't have a reference on the page's inode, yes, you should use
			// set_page_dirty_lock(). If the page came from get_user_pages() then surely
			// you don't have a ref on the inode.."
			//
			//LDDv3 S.437 
			// "...Most code that performs this operation checks first to ensure that the
			// page is not in the reserved part of the memory map, 
			// which is never swapped out. Since user-space memory is not normally marked reserved,
			// this check should not strictly be necessary.."
			//
			// "... Regardless of whether the pages have been changed, they must be freed from the
			//	page cache, or they stay there forever. ..."
			//
			if(!PageReserved(pJob->ppPageList[iPage]))
			{
				//macht lock_page, set_page_dirty, unlock_page
				set_page_dirty_lock(pJob->ppPageList[iPage] );
			}	

			//page_cache_release() ist ein define auf put_page()
			put_page( pJob->ppPageList[iPage] );		
		}
	}
	pJob->boIsPinned = FALSE;


	//- freen der page liste
	if(pJob->boIsPageListValid)
		kfree( pJob->ppPageList );
	pJob->boIsPageListValid = FALSE;
}


// beendet eine DMA, egal ob mit oder ohne Fehler, ob gelaufen oder nicht (l�uft auch aus DPC)
void AGEXDrv_DMARead_EndDMA(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC, const bool boIsOk, const u16 BufferCounter)
{	
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channels[iDMA];
	PDMA_READ_TC pTC = &pDMAChannel->TCs[iTC];
	DMA_READ_JOB tmpJob;

	dev_dbg(pDevData->dev, "AGEXDrv_DMARead_EndDMA > DMA: %d, TC: %d, Res: %d, Seq: %d\n",
				   	iDMA, iTC, boIsOk, BufferCounter);
	if (pTC->boIsUsed == FALSE) {
		dev_warn(pDevData->dev, " AGEXDrv_DMARead_EndDMA > invalid TC!\n");
		return;
	}

	// copy job data from finished TC
	tmpJob = pTC->Job;
	tmpJob.boIsOk			= boIsOk;
	tmpJob.BufferCounter	= BufferCounter;

	spin_lock_bh(&pDevData->DMARead_SpinLock);

	// TC freigeben, unter lock setzen wegen race mit AGEXDrv_DMARead_Reset_DMAChannel()
	pTC->boIsUsed = FALSE;

	// add job to Jobs_Done FIFO
	if (kfifo_put(&pDMAChannel->Jobs_Done, tmpJob) == 0) {
		//sollte nie sein weil Test ob Platz ist, ist im 'AGEXDRV_IOC_DMAREAD_ADD_BUFFER'
		dev_warn(pDevData->dev, "AGEXDrv_DMARead_EndDMA > kfifo_put failed!\n");
	}
	spin_unlock_bh(&pDevData->DMARead_SpinLock);
	dev_dbg(pDevData->dev, "Jobs_Done %d (iDMA: %d)\n", kfifo_len(&pDMAChannel->Jobs_Done), iDMA);


	//SEM posten
	/**********************************************************************/
	//auch posten falls wir den buffer nicht adden konnten
	// 1. sollte nie so sein
	// 2. der buffer ist f�r immer lost
	// 3. der user thread kommt damit zurecht das es keinen buffer gibt
	up(&pDMAChannel->WaitSem);
}


//errechnet die Adr zum SG-Register (macht keine Pr�fung ob g�ltig)
static u8* AGEXDrv_DMARead_GetSGAdr(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC)
{
	u8* tmpPtr =  pDevData->pVABAR0 
				+ DMA_READ_TC_SG_OFFSET
				+ (iDMA * pDevData->DMARead_TCs * DMA_READ_TC_TC2TC_SETPBYTES)
				+ iTC * DMA_READ_TC_TC2TC_SETPBYTES;
	return tmpPtr;
}



/****************************************
 * 
 * DMARead fns
 *
****************************************/

//l�uft �ber alle DMAChannels und startet wenn freien Buffer & TC diese zu starten
void AGEXDrv_DMARead_StartDMA(PDEVICE_DATA pDevData)
{
	u8 iDMA, iTC;
	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_DMARead_StartDMA\n");


	//> �ber alle DMAs laufen
	for(iDMA = 0; iDMA < pDevData->DMARead_channels; iDMA++)
	{
		bool boHasBuffer, boHasTC, boIsTCInitDone;


		spin_lock_bh(&pDevData->DMARead_SpinLock);
//---------------------------------------------------------->
			
		//> gibt es Buffer & TC
		//noch unbenutzten Buffer?
		pr_devel(MODDEBUGOUTTEXT" - Jobs_ToDo %d (iDMA: %d)\n", kfifo_len( &pDevData->DMARead_Channels[iDMA].Jobs_ToDo), iDMA);
		boHasBuffer = ( kfifo_len( &pDevData->DMARead_Channels[iDMA].Jobs_ToDo)>=1) ? (TRUE) : (FALSE);
	
		//freien TC?
		boHasTC = FALSE;
		for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
			if (pDevData->DMARead_Channels[iDMA].TCs[iTC].boIsUsed == FALSE) {
				boHasTC = TRUE;
				break;
			}
		}

		//> wenn ja dann TC init
		boIsTCInitDone = FALSE;
		if( boHasBuffer && boHasTC ) {
			PDMA_READ_TC tmpTC 	= pDevData->DMARead_Channels[iDMA].TCs + iTC;
			DMA_READ_JOB tmpJob;

			if (kfifo_get(&pDevData->DMARead_Channels[iDMA].Jobs_ToDo, &tmpJob) == 1) {
				//struct belegen 
				tmpTC->boIsUsed	= TRUE;
				tmpTC->Job		= tmpJob;
				
				pr_devel(MODDEBUGOUTTEXT" - Job for (iDMA: %d, iTC: %d)\n", iDMA, iTC);

				boIsTCInitDone = TRUE;
			}//Job vorhanden
		}//Buffer&TC gefunden

//<----------------------------------------------------------
		spin_unlock_bh(&pDevData->DMARead_SpinLock);



		//> starten
		if (boIsTCInitDone) {
			pr_devel(MODDEBUGOUTTEXT" - try to start\n");
			spin_lock_bh(&pDevData->DMARead_SpinLock);
//---------------------------------------------------------->
			AGEXDrv_DMARead_StartNextTransfer_Locked(pDevData, iDMA, iTC);
//<----------------------------------------------------------
			spin_unlock_bh(&pDevData->DMARead_SpinLock);
		}		
	}//for DMAs

}



//startet den n�chste Transfer(SGs),
// - erkennt Fehler wenn m�glich DMA abbrechen
// - n�chste kann aber auch 1. bzw. letzte Transfer sein
// - kommt auch mit last+1(was nie sein sollte) zurecht(ist aber ein Fehler),
// - wird (auch) im DPC aufgerufen 
// - wird mit DMALock aufgerufen
void AGEXDrv_DMARead_StartNextTransfer_Locked(PDEVICE_DATA pDevData, const u32 iDMA, const u32 iTC)
{
	s32 			iSG 	= 0;
	PDMA_READ_TC 	tmpTC 	= pDevData->DMARead_Channels[iDMA].TCs + iTC;
	PDMA_READ_JOB 	pJob	= &tmpTC->Job;

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_DMARead_StartNextTransfer_Locked> DMA: %d, TC: %d\n",	iDMA, iTC);


	//> alles gut (sicher ist sicher) kann aber nichts machen
	if( (iDMA >= pDevData->DMARead_channels) || (iTC >= pDevData->DMARead_TCs) ||
	   	!tmpTC->boIsUsed || !pJob->boIsSGValid || !pJob->boIsSGMapped || (pJob->SGItemsLeft==0) || (pJob->pSGNext==NULL) ){
			printk(KERN_ERR MODDEBUGOUTTEXT" AGEXDrv_DMARead_StartNextTransfer_Locked> Invalid context!\n");return;
	}


	//> die SGs ins FPGA schreiben
	/***********************************************************************/
	//pro Transfer k�nnen max ... SGs �bertragen werden
	for (iSG=0; ((iSG < pDevData->DMARead_SGs) && (pJob->pSGNext!=NULL) && (pJob->SGItemsLeft>0) ); iSG++) 
	{
		u8* 		adrSG, WordIndex;
		u32			sg_length	= sg_dma_len(pJob->pSGNext);
		dma_addr_t  sg_address	= sg_dma_address(pJob->pSGNext);
			

		//> SG zusammenbauen
		u32 tempSG[DMA_READ_TC_TC2TC_SETPBYTES/4];

		//Flag 	(1DWord)
		tempSG[0] = 0; 
		if(pJob->pSGNext == pJob->SGTable.sgl)	//1. SG Element �ber alles
			tempSG[0] |= DMA_READ_TC_SG_FLAG_START_TRANSACTION; 

		if(iSG == 0)							//1. SG Element
			tempSG[0] |= DMA_READ_TC_SG_FLAG_START_TRANSFER; 

		if( pJob->SGItemsLeft == 1 )				//letzte SG Element �ber alles (somit auch IRQ ausl�sen)
			tempSG[0] |= DMA_READ_TC_SG_FLAG_END_TRANSACTION;

		if(iSG == (pDevData->DMARead_SGs-1))	//letzte SG Element, mehr geht nicht ins FPGA FIFO (mit dem SG darf es dann ein IRQ geben)
			tempSG[0] |= DMA_READ_TC_SG_FLAG_END_TRANSFER; 

		//Size	(1DWord)
		tempSG[1] = sg_length / 4;			// in DWORDs

		//Adr	(2DWord)
		tempSG[2] = sg_address & 0xFFFFFFFF;//LowPart;
#ifdef CONFIG_64BIT			
		tempSG[3] = sg_address >> 32;		//HighPart;
#else
		tempSG[3] = 0;						//HighPart;				
#endif		
			

		//> stimmt die Ausrichtung und die size? (adr und size) 
		if(		((sg_length & 0x3) != 0)
		 	|| 	((sg_address & (PAGE_SIZE-1)) != 0)
		 	||  (sg_length > DMA_READ_TC_SG_MAX_BYTECOUNT ) )
		{
			printk(KERN_ERR MODDEBUGOUTTEXT" AGEXDrv_DMARead_StartNextTransfer_Locked> Invalid alignment or size!\n");

			//sollt nie vorkommen und wenn ist es das letzte SG Element (size geht nicht auf)
			iSG = pDevData->DMARead_SGs;				//das Kaputte Element soll das letzte sein
			tempSG[0] |= DMA_READ_TC_SG_FLAG_ERROR; 	//Flag f�rs FPGA
		}
	

		//> n�chstes SGElement
		pJob->pSGNext =  sg_next(pJob->pSGNext);	//wenn es sg_is_last() dann gibt die fn NULL zur�ck, es m�ssen aber nicht alle Element einer SGListe benutzt sein.
		pJob->SGItemsLeft--;
	

		//> ins FPGA schreiben
		pr_devel(MODDEBUGOUTTEXT" - DMA SGs> i: %d > 0x%llx, Bytes %d\n", iSG, (u64) sg_address, sg_length);
		adrSG = AGEXDrv_DMARead_GetSGAdr(pDevData, iDMA, iTC);	
		for(WordIndex=0; WordIndex < (DMA_READ_TC_TC2TC_SETPBYTES/4); WordIndex++)
			iowrite32( tempSG[WordIndex], adrSG + WordIndex*4 );

	}//for max m�gliche SGs pro Transfer
}


//Beendet alle DMAs die durch sind (bzw. deren DMATransfer), und versucht neue zu starte
void AGEXDrv_DMARead_DPC(PDEVICE_DATA pDevData, const u32 isDoneReg, const u32 isOkReg, const u16* pBufferCounters)
{
	u8 iDMA, iTC, BitShift;

	dev_dbg(pDevData->dev, "AGEXDrv_DMARead_DPC > isDoneReg: 0x%08X, isOkReg: 0x%08X\n", isDoneReg, isOkReg);

	/* �ber alle DMAs/TCs gehen, wir gehen davon aus das es keine l�cken gibt!*/
	/**************************************************************************************/
	BitShift = 0;
	for (iDMA = 0; iDMA < pDevData->DMARead_channels; iDMA++) {
		for (iTC =0; iTC < pDevData->DMARead_TCs; iTC++) {
			bool isDone, isOk, isUsed;
		
			//> Bits sammeln (damit es besser zum lesen ist)
			isDone 		= ( ((isDoneReg >> BitShift) & 0x1) == 1) ? (TRUE) : (FALSE);
			isOk 		= ( ((isOkReg >> BitShift) & 0x1) == 1) ? (TRUE) : (FALSE);
			isUsed  	= pDevData->DMARead_Channels[iDMA].TCs[iTC].boIsUsed;

			//> FPGA sagt die DMA ist durch
			if (isDone) {
				//haben wir sie auch gestartet? (k�nnen nix machen wenn nicht)
				if (!isUsed) {
					dev_dbg(pDevData->dev, "AGEXDrv_DMARead_DPC > DMA IRQ without DMA! DMA: %d, TC: %d\n", iDMA, iTC);
				}
				else {
					//FPGA hat sie abgebrochen oder wir durch IOctrl)
					if (!isOk) {
						dev_dbg(pDevData->dev, "AgeXDMAReadDPC > DMA IRQ for a broken DMA, channel: %u, TC: %u\n", iDMA, iTC);
						AGEXDrv_DMARead_EndDMA(pDevData, iDMA, iTC, FALSE /*boIsOk*/, 0 /*don't care*/);
					}
					//isDone && isOK && isUsed
					else {
						//DMA oder nur Transfer durch?
						if (pDevData->DMARead_Channels[iDMA].TCs[iTC].Job.SGItemsLeft == 0)	//DMA & Transfer durch
							AGEXDrv_DMARead_EndDMA(pDevData, iDMA, iTC, TRUE /*boIsOk*/, pBufferCounters[BitShift]);
						else {	//nur der Transfer durch, n�chsten starten
							spin_lock_bh(&pDevData->DMARead_SpinLock);
//---------------------------------------------------------->
							AGEXDrv_DMARead_StartNextTransfer_Locked(pDevData, iDMA, iTC);
//<----------------------------------------------------------
							spin_unlock_bh(&pDevData->DMARead_SpinLock);
						}

					}//if ok
				}// if used
			}//if is done

			//> n�chster TC ist beim n�chsten Bit (auch wenn es die n�chste DMA ist)
			BitShift++;

		}//for TC
	}//for DMA



	/* versucht neue DMAs zu starten */
	/**************************************************************************************/
	AGEXDrv_DMARead_StartDMA(pDevData);

}


//bricht laufende DMAs ab sowie ungenutzte buffer
void AGEXDrv_DMARead_Abort_DMAChannel(PDEVICE_DATA pDevData, const u32 iDMA)
{
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channels[iDMA];
	DMA_READ_JOB tmpJob;
	u8 iTC;
	memset(&tmpJob, 0, sizeof(tmpJob));	/* wegen warning */
	
	dev_dbg(pDevData->dev, "AGEXDrv_DMARead_Abort_DMAChannel> DMA: %d\n", iDMA);

	spin_lock_bh(&pDevData->DMARead_SpinLock);
//----------------------------->
	//> alle Buffers aus .Jobs_ToDo() .Jobs_Done() adden mit FehlerFlag und .WaitSem posten f�r jeden verschobenen Buffer
	while( kfifo_get(&pDMAChannel->Jobs_ToDo, &tmpJob) == 1) {
		tmpJob.boIsOk	= FALSE;

		//adden (sollte immer passen)
		if (kfifo_put(&pDMAChannel->Jobs_Done, tmpJob) == 0) {
			printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_DMARead_Abort_DMAChannel> can't add Buffer to Jobs_Done, BufferLost\n");
			break;
		}

		//thread wecken
		up(&pDMAChannel->WaitSem);

		dev_dbg(pDevData->dev, "move JOB .Jobs_ToDo > Jobs_Done\n");
	}	

	//> f�r alle g�ltigen/laufenden DMAs, bei allen TCs.boIsUsed==true, 
	//> dann ein �SG� mit Bit4 (Error) schicken (wird vor dem FIFO im FPGA abgefangen)
	//> Job/Request offen lassen	
	for (iTC =0; iTC < pDevData->DMARead_TCs; iTC++) {
		//g�ltiger Eintrag mit dem gesuchten Request? (wenn boIsUsed, dann ist auch der Request g�ltig)
		if (pDMAChannel->TCs[iTC].boIsUsed) {
			u8* adrSG, WordIndex;
			u32 tempSG[DMA_READ_TC_TC2TC_SETPBYTES/4];

			dev_dbg(pDevData->dev, "abort DMA iTC: %d \n", iTC);

			//nur das Flag muss g�ltig sein
			tempSG[0] = DMA_READ_TC_SG_FLAG_ERROR;


			//dem FPGA sagen er soll die DMA abbrechen
			//(daher auf IRQ warten damit wir sicher sind das nicht mehr in den PC geschrieben wird, 
			// falls die DMA in diesem Moment durch ist, wird das IRQ "Flag" dann verworfen...)
			adrSG = AGEXDrv_DMARead_GetSGAdr(pDevData, iDMA, iTC);	
			for (WordIndex=0; WordIndex < (DMA_READ_TC_TC2TC_SETPBYTES/4); WordIndex++)
				iowrite32(tempSG[WordIndex], adrSG + WordIndex*4);
		}//if boIsUsed					
	}//for iTC
//<-----------------------------
	spin_unlock_bh(&pDevData->DMARead_SpinLock);
}



//bricht die Tasks ab, welche auf die DMAs warten (nicht 100% sicher das alle raus sind)
void AGEXDrv_DMARead_Abort_DMAWaiter(PDEVICE_DATA pDevData,  const u32 iDMA)
{
	u8 iTryLoop, iFIFOLoop;
	const u8 MaxTryLoop = 10; /*max Waiter die abgebrochen werden k�nnen*/

	//Note: - sem hat kein FIFO verhalten
	//		- der sem Z�hler gibt die anz von buffern in .Jobs_Done wieder
	//		- sem Z�hler und FIFO Z�hler, sind nicht Atomar verbunden daher
	//			1. FIFO add, 2. up(), 3. down() FIFO remove()
	//		- RaceCond, mit DPC new Image
	//		- diese fn l�uft unter dem IOCTRL Lock, daher im FIFO ist immer Platz f�r 1 Element
	//
	//==> wir m�ssen (dummy)Bilder adden und diese wieder durch ein down() raushohlen, dabei 3 M�glichkeiten
	// > TimeOut	<> User hat dummyBild weggelesenfertig(nicht 100% sicher)
	// > dummyBild  <> kein User, fertig(nicht 100% sicher)
	// > echtesBild <> wieder adden, 
	// 		Achtung! es k�nnte im FIFO nur noch echte geben (weil User dummy gelesen hat)
	// 		aber max FIFO size wiederholen, weil es k�nnte ja hintem im FIFO sein,
	// 		nach max FIFO size durchl�ufen sind wir sicher das, dass dummyBild vom User gelesen wurde
	//
	// ==> das ganze m mal wiederhohlen(f�r den Fall das Waiter aber keine Bild)

	pr_devel(MODDEBUGOUTTEXT" AGEXDrv_DMARead_Abort_DMAWaiter> DMA: %d\n",iDMA);

	for(iTryLoop=0; iTryLoop<MaxTryLoop; iTryLoop++)
	{

		//> DummyBild adden (sollte immer gehen)
		DMA_READ_JOB tmpJob;
		memset(&tmpJob, 0, sizeof(tmpJob));	/* wegen warning */
		tmpJob.pVMUser 				= (uintptr_t) NULL;
		tmpJob.boIsOk				= FALSE;
		tmpJob.boIsPageListValid 	= FALSE;
		tmpJob.boIsPinned			= FALSE;
		tmpJob.boIsSGValid			= FALSE;
		tmpJob.boIsSGMapped			= FALSE;

		//adden
		spin_lock_bh(&pDevData->DMARead_SpinLock);
//----------------------------->
		if( kfifo_put(&pDevData->DMARead_Channels[iDMA].Jobs_Done, tmpJob) == 0)
			printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_DMARead_Abort_DMAWaiter> can't add dummyBuffer(iDMA: %d, i: %d)\n", iDMA, iTryLoop);
//<-----------------------------
		spin_unlock_bh(&pDevData->DMARead_SpinLock);

		//thread wecken
		up(&pDevData->DMARead_Channels[iDMA].WaitSem);



		//> dummy wieder auslesen(wenn nicht vom User weggelesen)
		for(iFIFOLoop=0; iFIFOLoop<MAX_DMA_READ_JOBFIFO_SIZE; iFIFOLoop++)
		{
			//> etwas warten (mit schedule) damit UserThread aufwachen kann
			usleep_range(1*1000/*min us*/, 2*1000/*max us*/);


			//> versuchen (dummy/echtes)Bild zu lesen 
			if(down_trylock(&pDevData->DMARead_Channels[iDMA].WaitSem) == 0)
			{
				spin_lock_bh(&pDevData->DMARead_SpinLock);
//---------------------------------------------------------->
				if( kfifo_get(&pDevData->DMARead_Channels[iDMA].Jobs_Done, &tmpJob) == 1) 
				{
					//dummyBuffer? => fertig
					if(tmpJob.pVMUser == 0 /*NULL*/)
					{
						pr_devel(MODDEBUGOUTTEXT" - found dummyBuffer [%d:%d]\n",iTryLoop,iFIFOLoop);
						iFIFOLoop=MAX_DMA_READ_JOBFIFO_SIZE;
						iTryLoop=MaxTryLoop;
					}
					else
					{
						//echtes Bild wieder adden(sollte reinpassen)
						if( kfifo_put(&pDevData->DMARead_Channels[iDMA].Jobs_Done, tmpJob) == 0)
							printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_DMARead_Abort_DMAWaiter> can't add realBuffer\n");
						else
							up(&pDevData->DMARead_Channels[iDMA].WaitSem);

						pr_devel(MODDEBUGOUTTEXT" - found realBuffer [%d:%d]\n",iTryLoop,iFIFOLoop);
					}				
				}
				else
					printk(KERN_WARNING MODDEBUGOUTTEXT" AGEXDrv_DMARead_Abort_DMAWaiter>DMARead wake up, without buffer!\n");
//<----------------------------------------------------------
				spin_unlock_bh(&pDevData->DMARead_SpinLock);	
			}
			else
			{
				//TimeOut, FIFO leer, User hat dummyBuffer gelesen => fertig
				iFIFOLoop=MAX_DMA_READ_JOBFIFO_SIZE;
				pr_devel(MODDEBUGOUTTEXT" - Jobs_Done FIFO is empty [%d:%d]\n",iTryLoop,iFIFOLoop);
			}
		}//for FIFOSize

	}//for TryLoop
}


int AGEXDrv_DMARead_Reset_DMAChannel(PDEVICE_DATA pDevData, unsigned int dma_channel)
{
	PDMA_READ_CHANNEL pDMAChannel = &pDevData->DMARead_Channels[dma_channel];
	DMA_READ_JOB tmpJob;
	unsigned int iTC;
	int result;

	// abort running DMA transfers in FPGA and move jobs from Jobs_ToDo to Jobs_Done FIFO
	AGEXDrv_DMARead_Abort_DMAChannel(pDevData, dma_channel);

	// Wait for completion of pending transfers
	for (iTC = 0; iTC < pDevData->DMARead_TCs; iTC++) {
		spin_lock_bh(&pDevData->DMARead_SpinLock);
		if (pDMAChannel->TCs[iTC].boIsUsed) {
			// reset semaphore count, removing count associated with jobs in Jobs_Done FIFO
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
			sema_init(&pDMAChannel->WaitSem, 0);
#else
			init_MUTEX_LOCKED(&pDMAChannel->WaitSem);
#endif
			spin_unlock_bh(&pDevData->DMARead_SpinLock);

			// wait
			result = down_timeout(&pDMAChannel->WaitSem, msecs_to_jiffies(100));
			if (result == (-ETIME)) {
				dev_warn(pDevData->dev, "AGEXDrv_DMARead_Reset_DMAChannel(): reset DMA channel timeout waiting for lost image\n");
				return -EFAULT;
			}
			if (result != 0) {
				dev_warn(pDevData->dev, "AGEXDrv_DMARead_Reset_DMAChannel(): reset DMA channel down_timeout() failed\n");
				return -EFAULT;
			}
		}
		else
			spin_unlock_bh(&pDevData->DMARead_SpinLock);	
	}

	// All buffers should be in the Jobs_Done now, unmap all
	memset(&tmpJob, 0, sizeof(tmpJob));
	spin_lock_bh(&pDevData->DMARead_SpinLock);
	while (kfifo_get(&pDMAChannel->Jobs_Done, &tmpJob) == 1) {
		if (tmpJob.pVMUser != 0) {
			spin_unlock_bh(&pDevData->DMARead_SpinLock);	
			dev_info(pDevData->dev, "AGEXDrv_DMARead_Reset_DMAChannel(): unmapping lost buffer 0x%lx, %u pages\n", tmpJob.pVMUser, tmpJob.pagesPinned);
			AGEXDrv_DMARead_UnMapUserBuffer(pDevData, &tmpJob);
			spin_lock_bh(&pDevData->DMARead_SpinLock);
		}
	}
	spin_unlock_bh(&pDevData->DMARead_SpinLock);	

	// Reset semaphore
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,32)
	sema_init(&pDMAChannel->WaitSem, 0);
#else
	init_MUTEX_LOCKED(&pDMAChannel->WaitSem);
#endif

	return 0;
}
