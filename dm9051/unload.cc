//[printnb_c]

//[board_info_c]

//[spi_dm9051_c]

//[dma_spi_dm9051_c]

//[spi_user_c]

//[skb_rx_head_c]

//[skb_rx_core_c]

//[skb_rx_c]

//[sched_c]

//[int_dm9051_c]

//[sub_dm9051_c]

//[custom_gpio_dm9051_c]

//[ethtool_ops_c]

//[ethtool_ops1_c]

//[driver_c]

#if DM_CONF_APPSRC
/*
 *  init (AppSrc)
 */
 #if 0
 //[called] by [skb_rx_core_old.c] 
static void bcrdy_rx_info_clear(struct rx_ctl_mach *pbc)
{
	pbc->rxbyte_counter= 
	pbc->rxbyte_counter0= 0;
}
#endif
static void bcopen_rx_info_clear(struct rx_ctl_mach *pbc)
{
	pbc->rxbyte_counter= 
	pbc->rxbyte_counter0= 
	
	pbc->rxbyte_counter0_to_prt= 
#if 0	
	pbc->rx_brdcst_counter= 
	pbc->rx_multi_counter= 
#endif	
	pbc->rx_unicst_counter= 0;
	
	pbc->isbyte= 0xff; // Special Pattern
}
#endif

void Disp_RunningEqu(board_info_t *db)
{						
	if (db->RUNNING_rwregs[0]==db->rwregs[0] && db->RUNNING_rwregs[1]==db->rwregs[1])
	{		
		char s[50];
		if (db->bC.rxbyte_counter0_to_prt < 2)
		{
			//ISR 80 wrRd 3902/39fe (RO 98.0%) (RXB_00H Be ISR-PRS) (733 ++) @ rxb_cntr0 1
			//ISR 80 wrRd 3902/39fe (RO 98.0%) (RXB_00H No ISR-PRS) (733 ++) @ rxb_cntr0 1
			//ISR 80 wrRd 3902/39fe (RO 98.0%) Equ&(RXB_00H Be ISR-PRS)Impossible (733 ++) @ rxb_cntr0 1

			 if ((db->rwregs[0]!=db->rwregs[1]) && (db->bC.isbyte & ISR_PRS))
				sprintf(s, "(rxb00_cnt %02d Diff& Be ISR-PRS)Rare-case", db->bC.rxbyte_counter0); //printk("dm9_when: Points_diff_and_ISR-PRS \n");
			 else if (db->rwregs[0]!=db->rwregs[1])
				sprintf(s, "Diff(rxb00_cnt %02d  but no ISR-PRS)", db->bC.rxbyte_counter0); //printk("dm9_when: Points_diff only. No ISR-PRS\n");
			 else
				sprintf(s, "(rxb00_cnt %02d Equ& Be ISR-PRS)Impossible", db->bC.rxbyte_counter0); //printk("dm9_when: ISR-PRS only. Points-equ (Impossible!)\n"); //Impossible!	
				
			 printk("dm9-IOR wrRd %04x/%04x (RO %d.%d%c) ISR %02x %s (%2d ++)\n",
			   db->rwregs[0], db->rwregs[1], db->calc>>8, db->calc&0xFF, '%', db->bC.isbyte, s,
			   db->bC.rxbyte_counter0_to_prt);	
	   }
	   db->bC.rxbyte_counter0_to_prt += 1;
	}
	db->RUNNING_rwregs[0]= db->rwregs[0];
	db->RUNNING_rwregs[1]= db->rwregs[1];
}

void Disp_RunningEqu_Ending(board_info_t *db)
{
	char *s;
	if (db->bC.rxbyte_counter0_to_prt >= 2)
	{
	  if ((db->rwregs[0]!=db->rwregs[1]) && (db->bC.isbyte & ISR_PRS))
		s= "(---Accumulat times---)Rare-case";
	  else if (db->rwregs[0]!=db->rwregs[1])
		s= "(---Accumulat times---)Diff";
	  else
		s= "(---Accumulat times---)Impossible";

	  printk("dm9-IOR wrRd %04x/%04x (RO %d.%d%c) ISR %02x rxb= %02x %s (%2d ++)\n",
	    db->rwregs[0], db->rwregs[1], db->calc>>8, db->calc&0xFF, '%', db->bC.isbyte, db->bC.dummy_pad, s,
	    db->bC.rxbyte_counter0_to_prt);
	}
}

/*
 *  disp
 */
static void dm9051_fifo_reset_statistic(board_info_t *db)
{
	if (!(db->bC.DO_FIFO_RST_counter%10)) {
		rel_printk1("dm9-Mac_OvFlwEr.Rxb&LargEr RST_c %d\n", db->bC.DO_FIFO_RST_counter);
		rel_printk1(" %d %d.%d %d\n", 
			db->bC.ERRO_counter, db->bC.OvrFlw_counter, db->bC.RXBErr_counter, db->bC.LARGErr_counter);
		if (db->bC.StatErr_counter)
			rel_printk1("dm9-RareFnd StatEr %d\n", db->bC.StatErr_counter);
	}
}

// --- const, extern function and varibles---
// ---  evt-queue, event & sched  ---

#if DM9051_CONF_TX 	
static u16 check_cntStop(board_info_t *db)
{
#if 0	
	u16 cs;
	while (!spin_trylock(&db->statelock_tx1_rx1)) ; //if(!)
	cs = db->bt.prob_cntStopped;
	spin_unlock(&db->statelock_tx1_rx1); 
	return cs;
#endif	
	return (!skb_queue_empty(&db->txq));
	
}

static void opening_wake_queue1(struct net_device *dev) //( u8 flag)
{
#if 0	
	board_info_t *db= netdev_priv(dev);
	
	while (!spin_trylock(&db->statelock_tx1_rx1)) ; //if(!)
	if (db->bt.prob_cntStopped)
	{
		db->bt.prob_cntStopped= 0;
		netif_wake_queue(dev);
	}
	spin_unlock(&db->statelock_tx1_rx1);
#endif

	board_info_t *db= netdev_priv(dev);
	if (db->bt.prob_cntStopped) {
		db->bt.prob_cntStopped= 0;
		netif_wake_queue(dev);
	}
}

static void toend_stop_queue1(struct net_device *dev, u16 stop_cnt)
{
#if 0	
	board_info_t *db= netdev_priv(dev);	
	while (!spin_trylock(&db->statelock_tx1_rx1)) ; //if(!)
	switch(stop_cnt)
	{
		case 1:
		db->bt.prob_cntStopped++;
		break;
		case NUM_QUEUE_TAIL:
		default:
		db->bt.prob_cntStopped= stop_cnt;
		break;
	}
	spin_unlock(&db->statelock_tx1_rx1); 
	
	if (stop_cnt<NUM_QUEUE_TAIL)
		return; // true;
	if (stop_cnt==NUM_QUEUE_TAIL)
	{
	  	netif_stop_queue(dev);
		return; // true;
	}
	
	//.wrong path, but anyhow call stop for it
	netif_stop_queue(dev);
	printk("[.wrong path]: WARN, anyhow call stop for it .. ");
	printk("(cntStop %d)\n", db->bt.prob_cntStopped);
	driver_dtxt_disp(db); // OPTIONAL CALLED
	return; // false;
#endif	

	board_info_t *db= netdev_priv(dev);	
	switch(stop_cnt) {
		case 1:
		  db->bt.prob_cntStopped++;
		  break;
		case NUM_QUEUE_TAIL:
		  db->bt.prob_cntStopped= stop_cnt;
		  break;
	}	
	if (stop_cnt==NUM_QUEUE_TAIL)
	  	netif_stop_queue(dev);
}
#endif

//..
void rx_mutex_head(board_info_t *db)
{
  #ifdef DM_CONF_POLLALL_INTFLAG	
	  mutex_lock(&db->addr_lock);
	  //.iiow(db, DM9051._IMR, IMR._PAR); // Disable all interrupts 
  #elif DRV_POLL_1
	  mutex_lock(&db->addr_lock);
  #endif
}
//..
void rx_mutex_tail(board_info_t *db)
{
  #ifdef DM_CONF_POLLALL_INTFLAG
	//.iiow(db, DM9051._IMR, db->imr._all); // Re-enable interrupt mask 
    mutex_unlock(&db->addr_lock);
  #elif DRV_POLL_1
    mutex_unlock(&db->addr_lock);
  #endif
}
//[spi_dm9051_c]

//[Header definitions]

#ifdef RPI_CONF_SPI_DMA_YES
#define dm9051_space_alloc  dma_space_request  //#define dm9051_space_request dma_space_request
#define dm9051_space_free    dma_space_free
#endif
#ifndef RPI_CONF_SPI_DMA_YES
#define dm9051_space_alloc  std_space_request  //#define dm9051_space_request std_space_request
#define dm9051_space_free    std_space_free
#endif

//dm9051_dbg_alloc(d); 
//dm9051_dbg_free(d); 
#if DEF_SPIRW
#define dm9051_spirw_begin(d) dm9051_space_alloc(d)
#define dm9051_spirw_end(d)      dm9051_space_free(d)
#else
#define dm9051_spirw_begin(d)  // Never called, only called while define _DEF_SPIRW in above if-condition.
#define dm9051_spirw_end(d)  // Essentially called.
#endif

//[Usage definitions]
//Usage
#define dmaXFER  dma_spi_xfer_buf
#define stdXFER  std_spi_xfer_buf
//Opt
#if DMA3_P2_RSEL_1024F
#define dmaRX 	dma_read_rx_buf_1024 // reserve 1 byte in the head. // dma_ with_ ncpy_
#else
#define dmaRX 	dma_read_rx_buf_cpy // reserve 1 byte in the head. // dma_ with_ ncpy_
#endif
#if DMA3_P2_TSEL_1024F
#define dmaTX 	dma_write_tx_buf_1024
#else
#define dmaTX 	dma_write_tx_buf
#endif
#if DMA3_P2_RSEL_1024F
#define stdRX	std_read_rx_buf_1024
#else
#ifdef QCOM_RX_DWORD_BOUNDARY
#define stdRX	std_read_rx_buf_ncpy_dword_boundary
 #else
#define stdRX	std_read_rx_buf_ncpy
#endif
#endif
#if DMA3_P2_TSEL_1024F
#define stdTX 	std_write_tx_buf_1024
#else
 #ifdef QCOM_TX_DWORD_BOUNDARY
 #define stdTX 	std_write_tx_buf_dword_boundary
 #else
 #define stdTX 	std_write_tx_buf
 #endif
#endif

#if 0 // no used.
//[Functions cast definitions]
#define dm9051_read_rx_buf  dm9.inblk_defcpy_or_dm9.inblk_noncpy
#define dm9051_write_tx_buf  dm9.outblk
//[Functions cast definitions]
#define dm9051_spi_read_reg     dm9.iorb
#define dm9051_spi_write_reg     dm9.iowb

//#ifdef RPI_CONF_SPI_DMA_YES
//#define dm9051_spi_xfer_buf  dma_spi_xfer_buf
//#define dm9051_read_rx_buf  dma_read_rx_buf
//#define dm9051_write_tx_buf  dma_write_tx_buf
//#else
//#define dm9051_spi_xfer_buf  std_spi_xfer_buf
//#define dm9051_read_rx_buf  std_read_rx_buf
//#define dm9051_write_tx_buf  std_write_tx_buf
//#endif
//#ifdef RPI_CONF_SPI_DMA_YES
//#define dm9051_spi_read_reg     dma_spi_read_reg 
//#define dm9051_spi_write_reg     dma_spi_write_reg
//#else
//#define dm9051_spi_read_reg     std_spi_read_reg
//#define dm9051_spi_write_reg     std_spi_write_reg
//#endif
#endif // no used.

//[APIs definitions]
#if DEF_SPICORE
#define ior					dm9.iorb
#define iior					dm9.iorb
#define iow					dm9.iowb
#define iiow					dm9.iowb
#define dm9051_outblk				dm9.outblk
#define dm9051_inblk_rxhead(d,b,l)		dm9.inblk_defcpy(d,b,l,true)
#define dm9051_inblk_noncpy(d,b,l)		dm9.inblk_noncpy(d,b,l)
#define dm9051_inblk_dump(d,l)			dm9.inblk_defcpy(d,NULL,l,false)
#endif

//Temp
#if DEF_SPIRW
typedef struct cb_info_t {
	/*[int (*xfer)(board_info_t *db unsigned len);]*/ //by.= /'dmaXFER'
        /*[int (*xfer)(board_info_t *db, u8 *txb, u8 *rxb, unsigned len);]*/ //by.= /'stdXFER'
         u8 (*iorb)(board_info_t *db, unsigned reg);
         void (*iowb)(board_info_t *db, unsigned reg, unsigned val);
         void (*inblk_defcpy)(board_info_t *db, u8 *buff, unsigned len, bool need_read); // 1st byte is the rx data.
         void (*inblk_noncpy)(board_info_t *db, u8 *buff, unsigned len); // reserve 1 byte in the head.
         int (*outblk)(board_info_t *db, u8 *buff, unsigned len);
	 /*struct spi_transfer Tfer;
	 struct spi_message Tmsg; 
	 //struct spi_transfer *fer;
	 //struct spi_message *msg;
	 */
} cb_info;

static cb_info dm9;
#endif

void callback_setup(int dma_bff);  // Setup the call back functions.

//[Implemant code]
#if DEF_PRO & DEF_SPIRW
static int  std_alloc(struct board_info *db)
{
        #ifdef RPI_CONF_SPI_DMA_YES
        printk("[ *dm9051 DRV ] spi mode[= std] using 'enable_dma' is 0\n");
        printk("[ *dm9051 DRV ] spi mode[= dma] But using kmalloc, TxDatBuf[] or std_alloc TxDatBuf\n"); //ADD.JJ
        #else
        printk("[ *dm9051 DRV ] spi mode[= std] using kmalloc, TxDatBuf[] or std_alloc TxDatBuf\n"); //ADD.JJ
        #endif

	#if DEF_SPIRW
        db->spi_sypc_buf = kmalloc(SPI_SYNC_TRANSFER_BUF_LEN, GFP_ATOMIC);
        #if 0
        printk("[ *dm9051 DRV ] spi mode[= std] using devm_kzalloc, TxDatBuf[] or std_alloc TxDatBuf\n"); //ADD.JJ
        db->spi_sypc_buf = devm_kzalloc(&spi->dev, SPI_SYNC_TRANSFER_BUF_LEN, GFP_KERNEL);
        #endif
        
        if (!db->spi_sypc_buf)
                return -ENOMEM;
	#endif
		
        return 0; // no-Err
}

//#ifndef RPI_CONF_SPI_DMA_YES
//#endif
static int std_space_request(struct board_info *db)
{
        /* Alloc non-DMA buffers */
        callback_setup(0); // assign 0 to 'enable_dma'
        return std_alloc(db);
}
#endif

//[Implemant code]
#if DEF_REM & DEF_SPIRW
static void  std_free(struct board_info *db)
{
                printk("[dm951_u-probe].s ------- Finsih using kfree, param (db->spi_sypc_buf) -------\n");  //ADD.JJ //'TxDatBuf'
		
		#if DEF_SPIRW
                kfree(db->spi_sypc_buf);
                #if 0
                printk("[dm951_u-probe].s ------- Finsih using devm_kfree, param (&db->spidev->dev, db->spi_sypc_buf) -------\n");  //ADD.JJ //'TxDatBuf'
                devm_kfree(&db->spidev->dev, db->spi_sypc_buf); //'TxDatBuf'
                #endif
		#endif
}

//#ifndef RPI_CONF_SPI_DMA_YES
//#endif
static void std_space_free(struct board_info *db)
{
        /* Free non-DMA buffers */
         std_free(db);
}
#endif

#if DEF_SPIRW
static int std_spi_xfer_buf(board_info_t *db, u8 *txb, u8 *rxb, unsigned len)
{
        int ret;
#if 1 //'DEF_SPICORE_IMPL1'
#ifdef QCOM_BURST_MODE
	db->spi_xfer2[0].tx_buf = txb;
	db->spi_xfer2[0].rx_buf = NULL;
	db->spi_xfer2[0].len = 1;
	
	db->spi_xfer2[1].tx_buf = txb+1;
	db->spi_xfer2[1].rx_buf = NULL;
	if (rxb) //from [db->spi_xfer2[1].rx_buf = rxb+1];
		db->spi_xfer2[1].rx_buf =  rxb+1;
	db->spi_xfer2[1].len = len;
	ret = spi_sync(db->spidev, &db->spi_msg2); //[spi_msg2]
#else
	db->fer->tx_buf = txb;
	db->fer->rx_buf = rxb;
	db->fer->len = len + 1;
	db->fer->cs_change = 0;
	ret = spi_sync(db->spidev, db->msg);  //[spi_msg]
#endif
#endif
        if (ret < 0) {
                dbg_log("spi communication fail! ret=%d\n", ret);
        }
        return ret;
}
#endif

#if DEF_SPIRW

static int disp_std_spi_xfer_Reg(board_info_t *db, unsigned reg)
{
        int ret = 0;
        if (reg == DM9051_PIDL || reg == DM9051_PIDH ) {
                printk("dm905.MOSI.p.[%02x][..]\n",reg); 
        }
        if (reg == DM9051_PIDL || reg == DM9051_PIDH ) {
                printk("dm905.MISO.e.[..][%02x]\n", db->spi_sypc_buf[1]);  //'TxDatBuf'
        }
        return ret;
}

static u8 std_spi_read_reg(board_info_t *db, unsigned reg)
{
        u8 txb[2] = {0};
        u8 rxb[2] = {0};

        txb[0] = (DM_SPI_RD | reg);
        stdXFER(db, (u8 *)txb, rxb, 1); //cb.xfer_buf_cb(db, (u8 *)txb, rxb, 1); //std_spi_xfer_buf(db, (u8 *)txb, rxb, 1); //'dm9051_spi_xfer_buf'
        
  db->spi_sypc_buf[1] = rxb[1]; //.std.read_reg //'TxDatBuf'
  disp_std_spi_xfer_Reg(db, reg);
        return rxb[1];
}
#endif

#if DEF_SPIRW
static void std_spi_write_reg(board_info_t *db, unsigned reg, unsigned val)
{
        u8 txb[2] = {0};
        //if (!enable._dma) {
        //}
        txb[0] = (DM_SPI_WR | reg);
        txb[1] = val;
        stdXFER(db, (u8 *)txb, NULL, 1); //cb.xfer_buf_cb(db, (u8 *)txb, NULL, 1); //std_spi_xfer_buf(db, (u8 *)txb, NULL, 1); //'dm9051_spi_xfer_buf'
}
#endif

#if DEF_SPIRW
#if DEF_PRO 
  //&& DEF_SPIRW
  //&& DM_CONF_APPSRC
static void std_read_rx_buf(board_info_t *db, u8 *buff, unsigned len, bool need_read)
{
        //[this is for the (SPI_SYNC_TRANSFER_BUF_LEN - 1)_buf application.]
        unsigned one_pkg_len;
        unsigned remain_len = len, offset = 0;
        u8 txb[1];
        txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
        do {
                // 1 byte for cmd
                if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                        one_pkg_len = remain_len;
                        remain_len = 0;
                } else {
                        one_pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                }

                stdXFER(db, txb, db->spi_sypc_buf, one_pkg_len); //cb.xfer_buf_cb(db, txb, db->TxDatBuf, one_pkg_len); //std_spi_xfer_buf(db, txb, db->TxDatBuf, one_pkg_len); //'dm9051_spi_xfer_buf'
                if (need_read) {
                        #if 0
                        //test.ok.
                        if (one_pkg_len==4)
                        {
                                printk("Head %02x %02x %02x %02x\n", db->spi_sypc_buf[1], db->spi_sypc_buf[2], db->spi_sypc_buf[3], db->spi_sypc_buf[4]);
                        }
                        #endif
                        memcpy(buff + offset, &db->spi_sypc_buf[1], one_pkg_len); //if (!enable._dma)//.read_rx_buf //'TxDatBuf'
                        offset += one_pkg_len;
                }
        } while (remain_len > 0);
}

#if DMA3_P2_RSEL_1024F
static void std_read_rx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
        //[this is for the 1024_buf application.(with copy operations)][It's better no-copy]
	u8 txb[1];
	int const pkt_count = (len + 1) / CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1) % CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	//.if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){	
		txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
		if (pkt_count) {
			int blkLen;
			//(1)
			blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1; // minus 1, so real all is 1024 * n
			stdXFER(db, txb, db->spi_sypc_buf, /*RD_LEN_ONE +*/ blkLen);
			memcpy(&buff[1], &db->spi_sypc_buf[1], /*RD_LEN_ONE +*/ blkLen);
	        //.printk("dm9rx_EvenPar_OvLimit(%d ... \n", blkLen);
			//(1P)
			if (remainder) {
			  //.blkLen= remainder;
			  stdXFER(db, txb, db->spi_sypc_buf, /*RD_LEN_ONE +*/ remainder);
			  memcpy(buff + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count), &db->spi_sypc_buf[1], remainder);
		//.printk("dm9rx_EvenPar_OvRemainder(%d ... \n", blkLen);
			}
			return;
		}
		//(2)	
		if (remainder) {
			//stdXFER(db, txb, db->spi_sypc_buf, remainder-1);
			//memcpy(&buff[1], &db->spi_sypc_buf[1], remainder-1);
			//note: len= remainder-1
			stdXFER(db, txb, buff, len);
		}
		return;
	//.}
}
#else
#ifdef QCOM_RX_DWORD_BOUNDARY
static void std_read_rx_buf_ncpy_dword_boundary(board_info_t *db, u8 *buff, unsigned len)
{                               
									       
	unsigned pkg_len= len;
        unsigned remain_len = len;
	unsigned offset = 0;
	u8 txbf[1];
	
	if ((pkg_len+1) >= 4) {
		pkg_len = ((pkg_len + 1) >> 2) << 2; //[new for dword boundary]
		pkg_len--;
		//pkg_len = ((((pkg_len + 1) + 3) /4 )*4) - 4; //[new for dword boundary]
		//pkg_len  -= 1;
		
		//[do.here]
		txbf[0]= DM_SPI_RD | DM_SPI_MRCMD;
		stdXFER(db, txbf,& buff[offset], pkg_len);          
		
		remain_len -= pkg_len;
		offset += pkg_len;
	}
	
	while(remain_len > 0) {                                        
		#if 1
		u8 txb[2] = {0};
		u8 rxb[2] = {0};

		txb[0] = DM_SPI_MRCMD; //(DM_SPI_RD | reg);
		stdXFER(db, (u8 *)txb, rxb, 1); 
		buff[++offset] = rxb[1];
		remain_len--;
		#endif
	
		/*buff[++offset] = ior(db, DM_SPI_MRCMD);
		remain_len--;*/
	}
#if 0	                                                               
        unsigned remain_len = len;
        unsigned pkg_len, offset = 0;
        u8 txb[1];
	u8 bf1;
	do {
		pkg_len = remain_len;
	
		if ((pkg_len+1) < 4) {
			pkg_len = 1; //[when pkg_len= 1, should like the ior()]
		} else {

			pkg_len = ((pkg_len + 1) >> 2) << 2; //[new for dword boundary]
			pkg_len--;
			//pkg_len = ((((pkg_len + 1) + 3) /4 )*4) - 4; //[new for dword boundary]
			//pkg_len  -= 1;
		}
		
		bf1= buff[offset];
		txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
		stdXFER(db, txb,& buff[offset], pkg_len);
		buff[offset]= bf1;
		
		remain_len -= pkg_len;
		offset += pkg_len;
        } while (remain_len > 0);
	
	//=txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
	//=stdXFER(db, txb, buff, len);
#endif
}
#else   //QCOM_RX_DWORD_BOUNDARY
static void std_read_rx_buf_ncpy(board_info_t *db, u8 *buff, unsigned len)
{
        //[this is for the 0_buf application.][It's no-copy]
        u8 txb[1];
        txb[0] = DM_SPI_RD | DM_SPI_MRCMD;
        stdXFER(db, txb, buff, len);
}
#endif
#endif

#endif
#endif

#if DEF_SPIRW
#if DEF_PRO 
  //&& DEF_SPIRW
  //&& DM_CONF_APPSRC
  
#if DMA3_P2_TSEL_1024F
static int std_write_tx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
	int blkLen;
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	unsigned offset = 0;
	
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
		db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD;
		memcpy(&db->spi_sypc_buf[1], &buff[offset], blkLen);
                offset += blkLen;
		stdXFER(db, db->spi_sypc_buf, NULL, blkLen);
        //.printk("dm9tx_std_EvenPar_OvLimit(%d ... \n", blkLen);
		
		/*xfer->tx_buf = db->spi_sypc_buf;
		xfer->rx_buf = NULL;
		xfer->len = RD_LEN_ONE + blkLen; // minus 1, so real all is 1024 * n
		if(spi_sync(db->spidev, &db->spi_msg1))
			printk("[dm95_spi]INNO txERR1: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf);*/

		//(2)	
		blkLen= remainder;
		memcpy(&db->spi_sypc_buf[1], &buff[offset], blkLen);
                //offset += blkLen;
		stdXFER(db, db->spi_sypc_buf, NULL, blkLen);
        //.printk("dm9tx_std_EvenPar_OvRemainder(%d ... \n", blkLen);
		
		/*xfer->tx_buf = db->spi_sypc_buf;
		xfer->rx_buf = NULL; 
		xfer->len = RD_LEN_ONE + remainder; 
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO txERR2: len=%d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf);*/
	} else {
		db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD;
		memcpy(&db->spi_sypc_buf[1], buff, len);
		stdXFER(db, db->spi_sypc_buf, NULL, len);
		
		/*xfer->tx_buf = db->spi_sypc_buf;
		xfer->rx_buf = NULL; 
		xfer->len = RD_LEN_ONE + len;
		if (spi_sync(db->spidev, &db->spi_msg1))
			printk("[dm95_spi]INNO ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,xfer->tx_buf,xfer->rx_buf);*/
	}
        return 0;
}
#else
#ifdef QCOM_TX_DWORD_BOUNDARY
static int std_write_tx_buf_dword_boundary(board_info_t *db, u8 *buff, unsigned len)
{
        unsigned remain_len = len;
        unsigned pkg_len, offset = 0;
	
//.	printk("[dm9][tx %d, dword-bound] %02x %02x %02x %02x %02x %02x", 
		//remain_len, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
	
        do {
                // 1 byte for cmd
                if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
			
                        pkg_len = remain_len;
			
                } else {
			
                        pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
			
                }
		
		if ((pkg_len+1) < 4) {
			pkg_len = 1;
		} else {

			pkg_len = ((pkg_len + 1) >> 2) << 2; //[new for dword boundary]
			pkg_len--;
			//pkg_len = ((((pkg_len + 1) + 3) /4 )*4) - 4; //[new for dword boundary]
			//pkg_len  -= 1;
		}
		
		remain_len -= pkg_len;

                db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD; //if (!enable._dma) { } //'TxDatBuf'
                memcpy(&db->spi_sypc_buf[1], buff + offset, pkg_len); //'TxDatBuf'
                
    //.if (!remain_len && (pkg_len!=len)){
    	//.switch (pkg_len){
    	//.	case 3:
    	//.		printk(" %02x %02x %02x", buff[offset], buff[offset+1], buff[offset+2]);
    	//.		break;
    	//.	case 1:
    	//.		printk(" %02x", buff[offset]);
    	//.		break;
    	//.}
    	//.printk(" [end.t.xfer %d]", pkg_len);
    //.} else {
    	//.switch (pkg_len){
    	//.	case 1:
    	//.		printk(" %02x", buff[offset]);
    	//.		break;
    	//.}
			//.printk(" [t.xfer %d]", pkg_len);
		//.}
                
                offset += pkg_len;
                stdXFER(db, db->spi_sypc_buf, NULL, pkg_len); //cb.xfer_buf_cb(db, db->TxDatBuf, NULL, pkg_len); //std_spi_xfer_buf(db, db->TxDatBuf, NULL, pkg_len); //'dm9051_spi_xfer_buf'
        } while (remain_len > 0);
	//.printk("\n");
        return 0;
}
#endif
#ifndef QCOM_TX_DWORD_BOUNDARY
static int std_write_tx_buf(board_info_t *db, u8 *buff, unsigned len)
{
        unsigned remain_len = len;
        unsigned pkg_len, offset = 0;
        do {
                // 1 byte for cmd
                if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                        pkg_len = remain_len;
                        remain_len = 0;
                } else {
                        pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                }

                db->spi_sypc_buf[0] = DM_SPI_WR | DM_SPI_MWCMD; //if (!enable._dma) { } //'TxDatBuf'
                memcpy(&db->spi_sypc_buf[1], buff + offset, pkg_len); //'TxDatBuf'
                
                offset += pkg_len;
                stdXFER(db, db->spi_sypc_buf, NULL, pkg_len); //cb.xfer_buf_cb(db, db->TxDatBuf, NULL, pkg_len); //std_spi_xfer_buf(db, db->TxDatBuf, NULL, pkg_len); //'dm9051_spi_xfer_buf'

        } while (remain_len > 0);
        return 0;
}
#endif
#endif

#endif
#endif

#if 0
//static void dm9051_read_rx_buf(board_info_t *db, u8 *buff, unsigned len, bool need_read)
//{
//        if (need_read) { // need data but no buff, return
//                if (!buff) {
//                        dbg_log("rx U8* buff fail\r\n");
//                        return;
//                }
//        }
//        if (len <= 0) {
//                dbg_log("rx length fail\r\n");
//                return ;
//        }
//        do {
//        } while (remain_len > 0);
//}
#endif

#if 0
//static int dm9051_write_tx_buf(board_info_t *db, u8 *buff, unsigned len)
//{
//        if (len > DM9051_PKT_MAX) {
//                dbg_log("warning: send buffer overflow!!!\n");
//                return -1;
//        }
//        do {
//        } while (remain_len > 0);
//        return 0;
//}
#endif

#ifdef MUST_STATIC_DECL
/*static int dm9051_spi_xfer_bufXXReg(struct spi_device *spi)
{
        board_info_t *db = spi_get_drvdata(spi);
        static struct spi_transfer xfer;
        static struct spi_message msg;
        int ret;
        u8 rxb[4] = {0};

        spi_message_init(&msg);

        xfer.tx_buf = db->TxDatBuf;
        xfer.rx_buf = rxb; //[db->TxDatBuf;]
        xfer.len = 2;

        //xfer.cs_change = 0;
    
        //struct spi_transfer *xfer = &db->spi_xfer1;
        //struct spi_message *msg = &db->spi_msg1;
        //xfer->tx_buf = txb;
        //xfer->rx_buf = rxb;
        //xfer->len = len + 1;
        //xfer->cs_change = 0;
        //if (enable._dma) {
        //  xfer->tx_dma = db->spi_tx_dma;
        //  xfer->rx_dma = db->spi_rx_dma;
        //  msg->is_dma_mapped = 1;
        //}

        spi_message_add_tail(&xfer, &msg);
        ret = spi._sync(spi, &msg);
        
        db->TxDatBuf[1] = rxb[1]; //when read.
        if (ret) {
                dbg_log("spi transfer failed: ret=%d\n", ret);
        }
        return ret;
}*/
#endif //0

//static u8 dm9051_spi_read_reg(board_info_t *db, unsigned reg) {
// #if 0
        //u8 txb[4] = {0};
        //u8 rxb[4] = {0};
// db->TxDatBuf[0] = (DM_SPI_RD | reg);
// dm9051_spi_xfer_bufXXReg(db->spidev);
// return db->TxDatBuf[1];
// #endif        
//}

//static void dm9051_spi_write_reg(board_info_t *db, unsigned reg, unsigned val) {
// #if 0        
// db->TxDatBuf[0] = (DM_SPI_WR | reg);
// db->TxDatBuf[1] = val;
// dm9051_spi_xfer_bufXXReg(db->spidev);
// #endif        
//}

#if 0 //PHASE-out
//static u8 dm9051_spi_read_regXX(board_info_t *db, /*int*/ unsigned reg)
//{
//        u8 txb[4] = {0};
//        u8 rxb[4] = {0};
//        txb[0] = (DM_SPI_RD | reg);
//        dm9051._spi_xfer_buf(db, (u8 *)txb, (u8 *)rxb, 1);
//        return rxb[1];
//}
//static void dm9051_spi_write_regXX(board_info_t *db, /*int*/ unsigned reg, /*int*/ unsigned val)
//{
//        u8 txb[2] = {0};
//        txb[0] = (DM_SPI_WR | reg);
//        txb[1] = val;
//        dm9051._spi_xfer_buf(db, (u8 *)txb, NULL, 1);
//}
#endif

#if DEF_SPICORE_IMPL0
#define RD_LEN_ONE	1
//-----------------------------------------------------
// ---   io operate  (all spi read/write dm9051) ---
//-----------------------------------------------------
void wbuff(unsigned op, __le16 *txb)
{
  //op= DM_SPI_WR | reg | val
	txb[0] = cpu_to_le16(op);
}

void wbuff_u8(u8 op, u8 *txb)
{
	txb[0]= op;
}

void xrdbyte(board_info_t * db, __le16 *txb, u8 *trxb)
{
	struct spi_transfer *xfer= &db->spi_xfer1;
	struct spi_message *msg= &db->spi_msg1;
	int ret;
	
	xfer->tx_buf = txb;
	xfer->rx_buf = trxb;
	xfer->len = 2;

	ret = spi_sync(db->spidev, msg);
	if (ret < 0)
		netdev_err(db->ndev, "spi_.sync()fail (xrd.byte 0x%04x) ret= %d\n", txb[0], ret);
}

void xwrbyte(board_info_t * db, __le16 *txb)
{
	struct spi_transfer *xfer = &db->spi_xfer1;
	struct spi_message *msg = &db->spi_msg1;
	int ret;
	
  #ifdef DM_CONF_VSPI
	return;
  #endif	
	
	xfer->tx_buf = txb;
	xfer->rx_buf = NULL;
	xfer->len = 2;

	ret = spi_sync(db->spidev, msg);
	if (ret < 0)
		netdev_err(db->ndev, "spi_.sync()failed (xwrbyte 0x%04x)\n", txb[0]);
}

void xrdbuff_u8(board_info_t *db, u8 *txb, u8 *trxb, unsigned len) //xwrbuff
{
	struct spi_transfer *xfer = &db->spi_xfer1;
	struct spi_message *msg = &db->spi_msg1;
	int ret;
	
#ifdef DM_CONF_VSPI
		trxb[1]= 0;
        return;
#endif	
		//(One byte)
        xfer->tx_buf = txb;
        xfer->rx_buf = trxb;
        xfer->len = RD_LEN_ONE + len;
		ret = spi_sync(db->spidev, msg);
		if (ret < 0){
	    	printk("9051().e out.in.dump_fifo4, %d BYTES, ret=%d\n", len, ret); //"%dth byte", i
			printk(" <failed.e>\n");
		}
//u8	return trxb[1];
}

//-----------------------------------------------------
/* --- custom model --- */
/* routines for sending block to chip */
/*static int INNODev_sync(board_info_t *db)
{
	return spi_sync(db->spidev, &db->spi_msg1); //'msg'
	
	//int ret;
	//mutex_lock(&db->sublcd_mtkspi_mutex);
	//ret= spi_sync(db->spidev, &db->spi_dmsg1);
	//mutex_unlock(&db->sublcd_mtkspi_mutex);
	//if(ret)
		//printk("[dm95_spi] spi.sync fail ret= %d, should check", ret);
	//return ret;
}*/
#endif

#if DEF_SPICORE_IMPL0
void dwrite_1024_Limitation(board_info_t *db, u8 *txb, u8 *trxb, int len)
{
	int blkLen;
	
	//struct spi_transfer *xfer;
		//xfer= &db->spi_dxfer1;
		//xfer= &db->spi_dxfer1;
		//xfer= &db->spi_dxfer1;
	struct spi_transfer *xfer = &db->spi_xfer1;
	//[&db->spi_msg1]
	//struct spi_message *msg = &db->spi_msg1;

#if 0 //# ifdef "DM_CONF_ADVENCE_MEM_YES"
	// advance memory usage
	
	//int const pkt_count = (len )/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	//int const remainder = (len )% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	//int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	  //1-0.
	  //..
	    wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->TxDatBuf); //'RD_LEN_ONE'
		xfer->tx_buf = db->TxDatBuf;
		xfer->rx_buf = NULL;
		//xfer->len = RD_LEN_ONE ; 
		//if(spi_sync(db->spidev, &db->spi_msg1))
		//	printk("[dm95_spi]INNO txERR1.0: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",1,len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;
	  //1-1.
	  //...
	  if (pkt_count)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
	  else
		blkLen= len;
	  //blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count ;
	  memcpy(db->TxDatBuf+1, txb, blkLen);
	  //xfer->tx_buf = txb;
		//xfer->rx_buf = NULL;
		//xfer->len = blkLen ; 
		xfer->len = RD_LEN_ONE + blkLen ; 
		if(spi_sync(db->spidev, &db->spi_msg1))
			printk("[dm95_spi]INNO txERR1.1-1: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf); 
	  //2.
		//..
		txb[blkLen-1] = db->TxDatBuf[0];
		//...
		blkLen = len - blkLen;
		if (blkLen) {
			//.blkLen= remainder;
			xfer->tx_buf = &txb[blkLen-1];
			xfer->rx_buf = NULL;
			xfer->len = RD_LEN_ONE + blkLen ; 
			if(spi_sync(db->spidev, &db->spi_msg1))
				printk("[dm95_spi]INNO txERR1.2: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",/*remainder*/ blkLen ,len,xfer->tx_buf,xfer->rx_buf); 
		}
#else
	
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;

	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
		//[TxDatBuf]
		//.printk("9Tx tbf=0x%p,rbf=%s [len, blkLen %d= %d + %d])\n", db->TxDatBuf, "NULL", len, blkLen +1, remainder - 1); // +1 for himan-read, -1 also

	    wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->spi_sypc_buf); //'RD_LEN_ONE'

#if 1
      //memcpy(db->tmpTxPtr, txb, RD_LEN_ONE + blkLen);
        memcpy(db->spi_sypc_buf+1, txb, blkLen);
		xfer->tx_buf = db->spi_sypc_buf; //txb;
		xfer->rx_buf = NULL; //db->tmpRxPtr; //NULL; //tmpRxPtr; //trxb; ((( When DMA 'NULL' is not good~~~
#else
#endif

		xfer->len = RD_LEN_ONE + blkLen; // minus 1, so real all is 1024 * n
		//spi_message_init(&db->spi_dmsg1);
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if(spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO txERR1: len= %d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;

		//(2)	
		blkLen= remainder;
#if 1
        memcpy(db->spi_sypc_buf+1, &txb[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1], remainder);
		xfer->tx_buf = db->spi_sypc_buf; //&txb[CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1]; // has been minus 1
		xfer->rx_buf = NULL; //NULL; //tmpRxPtr; //trxb; (((  'NULL' is not good~~~
#else
#endif

		xfer->len = RD_LEN_ONE + remainder; // when calc, it plus 1
		//spi_message_init(&db->spi_dmsg1);
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO txERR2: len=%d of %d, txbuf=0x%p,rxbuf=0x%p",blkLen,len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;
	} else {
		wbuff_u8(DM_SPI_WR | DM_SPI_MWCMD, db->spi_sypc_buf);
		//spi_message_init(&db->spi_dmsg1);
#if 1
      //memcpy(db->tmpTxPtr, txb, RD_LEN_ONE + len);
        memcpy(db->spi_sypc_buf+1, txb, len);
		xfer->tx_buf = db->spi_sypc_buf; //txb;
		xfer->rx_buf = NULL; //NULL; //tmpRxPtr; //trxb; ((( again When DMA 'NULL' is not good~~~
#endif
		xfer->len = RD_LEN_ONE + len;
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,xfer->tx_buf,xfer->rx_buf); //return INNO_GENERAL_ERROR;
	}
#endif
}

void dread_1024_Limitation(board_info_t *db, u8 *trxb, int len)
{
	struct spi_transfer *xfer = &db->spi_xfer1;
	//struct spi_transfer *xfer;
	u8 txb[1];
	int const pkt_count = (len + 1) / CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1) % CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){	
		int blkLen;
		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;

	    printkr("dm9rx_EvenPar_OvLimit(%d ... ", blkLen);
	    printkr("txbf=%s,rxbf=0x%p)\n", "&txb[0]", db->spi_sypc_buf /*db->tmpRxPtr*/);
    
		//spi_message_init(&db->spi_dmsg1);
		//xfer= &db->spi_dxfer1;
        //memcpy(db->tmpTxPtr, txb, 2);
		//xfer->tx_buf = db->tmpTxPtr; //txb;
		xfer->tx_buf = txb;
		xfer->rx_buf = db->spi_sypc_buf; //[TxDatBuf].instead-rxb. //db->tmpRxPtr; //trxb;
		xfer->len = RD_LEN_ONE + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count) - 1;  // minus 1, so real all is 1024 * n
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO1 ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,trxb); //return INNO_GENERAL_ERROR;
        memcpy(trxb, db->spi_sypc_buf, RD_LEN_ONE + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count) - 1);
		//(2)	
		blkLen= remainder;
		printkr("dm9rx_EvenPar_OvRemainder(%d ... ", blkLen);
		printkr("txbf=%s,rxbf=0x%p)\n", "&txb[0]", db->spi_sypc_buf /*db->tmpRxPtr*/);

		//spi_message_init(&db->spi_dmsg1);
		//xfer= &db->spi_dxfer1;
      //memcpy(db->tmpTxPtr, txb, 2);
      //memcpy(db->tmpRxPtr, db->spi_sypc_buf, RD_LEN_ONE + remainder);
		xfer->tx_buf = txb; //NULL is also OK.. //db->tmpTxPtr; //txb;
		xfer->rx_buf = db->spi_sypc_buf; //db->TxDatBuf;
		xfer->len = RD_LEN_ONE + remainder; // when calc, it plus 1
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO2 ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,xfer->rx_buf); //return INNO_GENERAL_ERROR;

        memcpy(trxb + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count), &db->spi_sypc_buf[1], remainder);
	}
	else{
		printkr("dm9rx_smal_(%d ... ", len);
		printkr("txbf=%s,rxbf=0x%p)\n", "&txb[0]", db->spi_sypc_buf);

		wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, txb);
		//spi_message_init(&db->spi_dmsg1);
		//xfer= &db->spi_dxfer1;

#if 1
	     //wbuff_u8(DM_SPI_RD | DM_SPI_MRCMD, db->tmpTxPtr);
#else
         //memcpy(db->tmpTxPtr, txb, 2);
#endif

		xfer->tx_buf = txb; //db->tmpTxPtr; //txb;
		xfer->rx_buf = db->spi_sypc_buf; //trxb;
		xfer->len = RD_LEN_ONE + len;
		//spi_message_add_tail(&db->spi_dxfer1, &db->spi_dmsg1);
		if (spi_sync(db->spidev, &db->spi_msg1)) //(INNODev_sync(db))
			printk("[dm95_spi]INNO ERROR: len=%d, txbuf=0x%p,rxbuf=0x%p",len,txb,trxb); //return INNO_GENERAL_ERROR;

		printkr("dm9rx_smal_tx_cmd(%s) ... \n", "%txb[0]");

		memcpy(trxb, db->spi_sypc_buf, RD_LEN_ONE + len);                
		//dread_32_Limitation(db, trxb, len);
	}
} 

#if DM_CONF_APPSRC
static int Custom_SPI_Write(board_info_t *db, u8 *buff, unsigned len) 
{
#if DMA3_P2_TSEL_1024F
	dwrite_1024_Limitation(db, buff, NULL, len);
	return 1;
#elif DMA3_P2_TSEL_32F
    memcpy(&db->spi_sypc_buf[1], buff, len);
    dwrite_32_Limitation(db, db->spi_sypc_buf, NULL, len);
	return 1;
#elif DMA3_P2_TSEL_1F
	memcpy(&db->spi_sypc_buf[1], buff, len);
	dwrite_1_Limitation(db, db->spi_sypc_buf, NULL, len);
	return 1;
#else
	return 0;
#endif
}

static int Custom_SPI_Read(board_info_t *db, u8 *buff, unsigned len)
{	
#if DMA3_P2_RSEL_1024F
	dread_1024_Limitation(db, buff, (int)len);
	return 1;
#elif DMA3_P2_RSEL_32F
	dread_32_Limitation(db, buff, (int)len);
	return 1;
#elif DMA3_P2_RSEL_1F
	dread_1_Limitation(db, buff, (int)len);
	return 1;
#else
	return 0;
#endif
}
#endif

#endif
//[dma_spi_dm9051_c]

//[Classic declare definitions]
//#ifdef RPI_CONF_SPI_DMA_YES
//static int enable._dma = 1; // Default: 1 (ON)
//#else
//static int enable._dma = 0; // Default: 0 (OFF)
//#endif
//=
#ifdef RPI_CONF_SPI_DMA_YES /* [defined in "conf_ver.h"] */
static int enable_dma;
#endif

//[Implemant code]
#if DEF_PRO
#ifdef RPI_CONF_SPI_DMA_YES
//[spi_dm9051_dma.c]
static int dm9051_is_dma_param(struct board_info *db)
{
        //if (!enable._dma) return enable._dma; //int ret = 0;
        int dma_bff= 0;

#if DEF_SPIRW        
        db->spidev->dev.coherent_dma_mask = ~0;

        db->spi_tx_buf = dma_alloc_coherent(&db->spidev->dev,
                                            PAGE_SIZE,
                                            &db->spi_tx_dma,
                                            GFP_KERNEL | GFP_DMA); //GFP_KERNEL, GFP_DMA

        if (db->spi_tx_buf) {
                /* OK the buffer for DMA */
                callback_setup(1); // 1, also assigned to 'enable_dma'
                dma_bff = 1;
                
                printk("[ *dm9051 DRV ] spi mode[= dma] using db->spi_tx_buf = dma_alloc_coherent(PAGE_SIZE, &db->spi_tx_dma)\n"); //ADD.JJ
                printk("[ *dm9051 DRV ] spi mode[= dma] using 'enable_dma', enable_dma = %d\n", dma_bff); //ADD.JJ
                db->spi_rx_dma = (dma_addr_t)(db->spi_tx_dma +
                        (PAGE_SIZE / 2));
                //db->spi_rx_buf = (db->spi_tx_buf + (PAGE_SIZE / 2));
        } else {
                /* Fall back to non-DMA */
                dma_bff = 0; 
        }
#endif        
        return dma_bff; //return ret;
}

//[spi_dm9051_dma.c]
static int dma_space_request(struct board_info *db)
{
        int dma_bff;
        /* Allocate DMA buffers */
#if 0 //test 
        dma_bff = 0;
#else
        dma_bff = dm9051_is_dma_param(db);
#endif

        if (!dma_bff)
                return std_space_request(db);
        return 0; // no-Err
}
#endif
#endif

//[Implemant code]
#if DEF_REM
#ifdef RPI_CONF_SPI_DMA_YES
//[spi_dm9051_dma.c]
static void dma_space_free(struct board_info *db)
{
        /* Free DMA buffers */
        if (enable_dma) { //.space_free
                printk("[dm951_u-probe].s -Finsih using dma_free_coherent(&db->spidev->dev, PAGE_SIZE, db->spi_tx_buf, db->spi_tx_dma, ..)-\n");  //ADD.JJ
                dma_free_coherent(&db->spidev->dev, PAGE_SIZE, db->spi_tx_buf, db->spi_tx_dma);
        }
        /* Free non-DMA buffers */
        if (!enable_dma) //.space_free, ADD.JJ (JJ)
                std_space_free(db); //~std_free();
}
#endif
#endif

#if DEF_SPIRW

#ifdef RPI_CONF_SPI_DMA_YES //.(more_for_cbCode).
static int dma_spi_xfer_buf(board_info_t *db, unsigned len)
{
        int ret;
#if 0
//.static struct spi_transfer Txfer;
//.static struct spi_message Tmsg; 
//.struct spi_transfer *xfer = &Txfer;
//.struct spi_message *msg =  &Tmsg;
//.spi_message_init(msg);
        xfer->tx_buf = db->spi_tx_buf; 
        xfer->rx_buf = db->spi_tx_buf; 
        xfer->tx_dma = db->spi_tx_dma;
        xfer->rx_dma = db->spi_rx_dma;
        msg->is_dma_mapped = 1;
        xfer->len = len + 1;
        xfer->cs_change = 0;
//.spi_message_add_tail(xfer, msg);
        ret = spi_sync(db->spidev, msg);
#else
        db->fer->tx_buf = db->spi_tx_buf;
        db->fer->rx_buf = db->spi_tx_buf;
        db->fer->tx_dma = db->spi_tx_dma;
        db->fer->rx_dma = db->spi_rx_dma;
        db->msg->is_dma_mapped = 1;
        db->fer->len = len + 1;
        db->fer->cs_change = 0;
        ret = spi_sync(db->spidev, db->msg);
#endif
        if (ret < 0)
                dbg_log("spi communication fail! ret=%d\n", ret);
        return ret;
}
#endif //.(more_for_cbCode).
#endif

#if DEF_SPIRW
#ifdef RPI_CONF_SPI_DMA_YES
static int disp_dma_spi_xfer_Reg(board_info_t *db, unsigned reg)
{
        int ret = 0;
        if (reg == DM9051_PIDL || reg == DM9051_PIDH ) {
                printk("dm905.MOSI.p.[%02x][..]\n",reg); 
        }
        if (reg == DM9051_PIDL || reg == DM9051_PIDH ) {
                printk("dm905.MISO.e.[..][%02x]\n", db->spi_tx_buf[1]);  //~db->spi_sypc_buf[1], 'TxDatBuf'
        }
        return ret;
}

static u8 dma_spi_read_reg(board_info_t *db, unsigned reg)
{
        //if (!enable._dma) //.dma.read_reg
        //  return std_spi_read_reg(db, reg);
                
        db->spi_tx_buf[0] = (DM_SPI_RD | reg); //if (enable._dma) //.dma.read_reg
        dmaXFER(db, 1); //cb.xfer_buf_cb(db, NULL, NULL, 1); //dma_spi_xfer_buf(db, NULL, NULL, 1); //'dm9051_spi_xfer_buf'
        
        //db->spi_sypc_buf[1] = db->spi_tx_buf[1]; //if (enable._dma)//.dma.read_reg  //'TxDatBuf'
  disp_dma_spi_xfer_Reg(db, reg);
        return db->spi_tx_buf[1]; //'TxDatBuf'
}
#endif
#endif

#if DEF_SPIRW
#ifdef RPI_CONF_SPI_DMA_YES
static void dma_spi_write_reg(board_info_t *db, unsigned reg, unsigned val)
{
        //if (!enable._dma) //.write_reg
         //  return std_spi_write_reg(db, reg, val);
        db->spi_tx_buf[0] = (DM_SPI_WR | reg);          //if (enable._dma) {//.dma.write_reg
        db->spi_tx_buf[1] = val;                                            //if (enable._dma) {//.dma.write_reg
         dmaXFER(db, 1); //cb.xfer_buf_cb(db, NULL, NULL, 1); //dma_spi_xfer_buf(db, /*txb*/ NULL, NULL, 1); //'dm9051_spi_xfer_buf'
}
#endif
#endif

#if DEF_SPIRW
#if DEF_PRO 
#ifdef RPI_CONF_SPI_DMA_YES

static void dma_read_rx_buf(board_info_t *db, u8 *buff, unsigned len, bool need_read)
{
        
        //if (!enable._dma)//.read_rx_buf
         //       return std_read_rx_buf(db, buff, len, need_read);
        
        //[this is for the (SPI_SYNC_TRANSFER_BUF_LEN - 1)_buf application.]
        do {
                // [=dma_rdloop_tx_buf]
                unsigned one_pkg_len;
                unsigned remain_len = len, offset = 0;
                db->spi_tx_buf[0] = (DM_SPI_RD | DM_SPI_MRCMD);
                do {
                        // 1 byte for cmd
                        if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                                one_pkg_len = remain_len;
                                remain_len = 0;
                        } else {
                                one_pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                                remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        }

                        dmaXFER(db, one_pkg_len); //cb.xfer_buf_cb(db, NULL, /* NULL */ db->TxDatBuf, one_pkg_len); //dma_spi_xfer_buf(db, /* txb */ NULL,  /* NULL */ db->TxDatBuf, one_pkg_len); //'dm9051_spi_xfer_buf'
                        if (need_read) {
                                memcpy(buff + offset, &db->spi_tx_buf[1], one_pkg_len); //if (enable._dma)//.dma.read_rx_buf
                                offset += one_pkg_len;
                        }
                } while (remain_len > 0);
        } while(0);
}

#if DMA3_P2_RSEL_1024F
static void dma_read_rx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
        //[this is for the 1024_buf application.(with copy operations)][It's better no-copy]
	int const pkt_count = (len + 1) / CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1) % CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
                db->spi_tx_buf[0] = (DM_SPI_RD | DM_SPI_MRCMD);
		if (pkt_count) {
			int blkLen;
			//(1)
			blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1; // minus 1, so real all is 1024 * n
			dmaXFER(db, blkLen);
			memcpy(&buff[1], &db->spi_tx_buf[1], blkLen);
	        //.printk("dm9rx_EvenPar_OvLimit(%d ... \n", blkLen);
			//(1P)
			if (remainder) {
			  db->spi_tx_buf[0] = (DM_SPI_RD | DM_SPI_MRCMD); // dma XFER need re-fill read_cmd_field.
			  dmaXFER(db, remainder);
			  memcpy(buff + (CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count), &db->spi_tx_buf[1], remainder);
		//.printk("dm9rx_EvenPar_OvRemainder(%d ... \n", blkLen);
                        }
                        return;
                }
		//(2)	
		if (remainder) {
                        dmaXFER(db, len);
                        memcpy(&buff[1], &db->spi_tx_buf[1], len);
			//note: len= remainder-1
		}
		return;
}
#else
static void dma_read_rx_buf_cpy(board_info_t *db, u8 *buff, unsigned len)
{
        //[this is for the 0_buf application.][It's better no-copy]
                db->spi_tx_buf[0] = (DM_SPI_RD | DM_SPI_MRCMD);
                dmaXFER(db, len);
                memcpy(&buff[1], &db->spi_tx_buf[1], len);
                        //Because from: [dm9051_inblk_noncpy(db, rdptr-1, RxLen);]
}
#endif

#endif
#endif
#endif

#if DEF_SPIRW
#if DEF_PRO 
#ifdef RPI_CONF_SPI_DMA_YES

#if DMA3_P2_TSEL_1024F
static int dma_write_tx_buf_1024(board_info_t *db, u8 *buff, unsigned len)
{
	int blkLen;
	int const pkt_count = (len + 1)/ CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	int const remainder = (len + 1)% CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES;
	unsigned offset = 0;
	if((len + 1)>CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES){
		//(1)
		blkLen= CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES*pkt_count - 1;
                db->spi_tx_buf[0] = (DM_SPI_WR | DM_SPI_MWCMD);
                memcpy(&db->spi_tx_buf[1], &buff[offset], blkLen);
                offset += blkLen;
                dmaXFER(db, blkLen);
        //printk("dm9tx_dma_EvenPar_OvLimit(%d ... \n", blkLen);
		//(2)	
		blkLen= remainder;
                db->spi_tx_buf[0] = (DM_SPI_WR | DM_SPI_MWCMD); //need re-fill again.
                memcpy(&db->spi_tx_buf[1], &buff[offset], blkLen);
                //offset += blkLen;
                dmaXFER(db, blkLen);
        //printk("dm9tx_dma_EvenPar_OvRemainder(%d ... \n", blkLen);
	} else {
		db->spi_tx_buf[0] = DM_SPI_WR | DM_SPI_MWCMD;
                memcpy(&db->spi_tx_buf[1], buff, len);
                dmaXFER(db, len);
        }
        return 0;
}
#else
static int dma_write_tx_buf(board_info_t *db, u8 *buff, unsigned len)
{
        //if (!enable._dma) //.write_tx_buf
        //        return std_write_tx_buf(db, buff, len);
                
        do {
                // [= dma_wrloop_tx_buf]
                unsigned remain_len = len;
                unsigned pkg_len, offset = 0;
                do {
                        // 1 byte for cmd
                        if (remain_len <= (SPI_SYNC_TRANSFER_BUF_LEN - 1)) {
                                pkg_len = remain_len;
                                remain_len = 0;
                        } else {
                                pkg_len = (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                                remain_len -= (SPI_SYNC_TRANSFER_BUF_LEN - 1);
                        }

                        //if (enable._dma) {//.write_tx_buf
                        //}
                        db->spi_tx_buf[0] = (DM_SPI_WR | DM_SPI_MWCMD);
                        memcpy(&db->spi_tx_buf[1], buff + offset, pkg_len);
                        
                        offset += pkg_len;
                        dmaXFER(db, pkg_len); //cb.xfer_buf_cb(db, /* NULL */ db->TxDatBuf, NULL, pkg_len); //dma_spi_xfer_buf(db, /* NULL */ db->TxDatBuf, NULL, pkg_len); //'dm9051_spi_xfer_buf'
                } while (remain_len > 0);
        } while(0);
        return 0;
}
#endif

#endif
#endif
#endif

void callback_setup(int dma_bff)
{
#ifdef RPI_CONF_SPI_DMA_YES 
        enable_dma = dma_bff;
   
        #if DEF_SPIRW
        if (enable_dma) {
                dm9.iorb= dma_spi_read_reg;
                dm9.iowb= dma_spi_write_reg;
                dm9.inblk_defcpy= dma_read_rx_buf;  // 1st byte is the rx data.
                dm9.inblk_noncpy= dmaRX; // reserve 1 byte in the head. // dma_ with_ ncpy_
                dm9.outblk= dmaTX;
        } else {
                dm9.iorb= std_spi_read_reg;
                dm9.iowb= std_spi_write_reg;
                dm9.inblk_defcpy= std_read_rx_buf;  // 1st byte is the rx data.
                dm9.inblk_noncpy= stdRX;
                dm9.outblk= stdTX;
        }
        #endif
#else
        #if DEF_SPIRW
        dm9.iorb= std_spi_read_reg;
        dm9.iowb= std_spi_write_reg;
        dm9.inblk_defcpy= std_read_rx_buf;  // 1st byte is the rx data.
        dm9.inblk_noncpy= stdRX;
        dm9.outblk= stdTX;
        //#if DMA3_P2_RSEL_1024F
        //#else
        //#endif
        //#if DMA3_P2_TSEL_1024F
        //#else
        //#endif
        #endif
#endif

#if 0
#ifdef RPI_CONF_SPI_DMA_YES
        //..gXfer = (enable_dma)? dma_spi_xfer_buf :std_spi_xfer_buf; // callback.
        //..cb.xfer = (enable_dma)? dma_spi_xfer_buf :std_spi_xfer_buf; // callback.
#endif
#ifndef RPI_CONF_SPI_DMA_YES
        //.gXfer = std_spi_xfer_buf;
        //.cb.xfer = std_spi_xfer_buf;
#endif
#endif
}

void dm9051_spimsg_init(board_info_t *db)
{
        //spi_message_init(&dm9.Tmsg);
        //spi_message_add_tail(&dm9.Tfer,&dm9.Tmsg);
        #if DEF_SPICORE_IMPL1
        #ifdef QCOM_BURST_MODE
         memset(&db->spi_xfer2, 0, sizeof(struct spi_transfer)*2); //[Add.] 
         spi_message_init(&db->spi_msg2);
         spi_message_add_tail(&db->spi_xfer2[0], &db->spi_msg2);
         spi_message_add_tail(&db->spi_xfer2[1], &db->spi_msg2);
        #else
        spi_message_init(&db->Tmsg);
        spi_message_add_tail(&db->Tfer,&db->Tmsg);
        db->fer = &db->Tfer;
        db->msg = &db->Tmsg;
        #endif
        #endif
}

#if DEF_OPE | DM_CONF_APPSRC
/*
 *  INT 
 */
void int_reg_stop(board_info_t *db)
{
#if DEF_SPIRW
	iiow(db, DM9051_IMR, IMR_PAR); // Disable all interrupts 
	if (db->nSCH_INT && (db->nSCH_INT <= DM9_DBG_INT_ONOFF_COUNT))
		printk("[dm9IMR].[%02x].dis ------- nINT= %d\n",
			iior(db, DM9051_IMR), db->nSCH_INT);
#endif
}

void int_reg_start(board_info_t *db, char *headstr)
{		
#if DEF_SPIRW
	iiow(db, DM9051_IMR, db->imr_all); /*iow*/
	if (db->nSCH_INT && (db->nSCH_INT <= DM9_DBG_INT_ONOFF_COUNT))
		printk("%s.[%02x].ena ------- nINT= %d\n", headstr,
			iior(db, DM9051_IMR), db->nSCH_INT);// Re-enable by interrupt mask register
#endif
}
#endif

#if DEF_PRO | DEF_OPE | DM_CONF_APPSRC
#if DEF_SPIRW
static void
reset_rst(board_info_t * db) {	
	iiow(db, DM9051_NCR, NCR_RST);
	//= 
	//__le16 txb[2]; 
	 // wbuff(DM_SPI_WR| DM9051_NCR | NCR_RST<<8, txb); //org: wrbuff(DM9051_NCR, NCR_RST, txb)
	 // xwrbyte(db, txb);
	mdelay(1);
}
static void
reset_bnd(board_info_t * db) {
	iiow(db, DM9051_MBNDRY, MBNDRY_BYTE);
	//= 
	//.__le16 txb[2]; 
	 // wbuff(DM_SPI_WR| 0x5e | 0x80<<8, txb); //org: wrbuff(DM9051_NCR, NCR_RST, txb)
	 // xwrbyte(db, txb);
	mdelay(1);
	//printk("iow[%02x %02x]\n", 0x5e, 0x80); //iow[x].[Set.MBNDRY.BYTEBNDRY]
}
#endif

static void
dm9051_reset(board_info_t * db)
{
	  mdelay(2); // delay 2 ms any need before NCR_RST (20170510)
	  #if DEF_SPIRW
	  reset_rst(db);
	  reset_bnd(db);
	  #endif      
	  db->rwregs1 = 0x0c00;
	  //[db->validlen_for_prebuf = 0;]
}
#endif

#if DM_CONF_APPSRC
// ------------------------------------------------------------------------------
// state: 0 , fill '90 90 90 ...' e.g. dm9051_fi.fo_re.set(0, "fifo-clear0", db);
//		  1 , RST
//        2 , dump 'fifo-data...'
//		 11 , RST-Silent
// hstr:  String 'hstr' to print-out
//        NULL (no 'hstr' print)
// Tips: If (state==1 && hstr!=NULL)
//        Do increase the RST counter
// ------------------------------------------------------------------------------
static void dm9051_fifo_reset(u8 state, u8 *hstr, board_info_t *db)
{
	u8 pk;
	if (state==11)
	{
		if (hstr)
     		{
			db->rx_rst_quan = 0;
			++db->bC.DO_FIFO_RST_counter;
			Disp_RunningEqu_Ending(db);
#ifdef ON_RELEASE
			rel_printk2("dm9-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter);
#else
			printk("dm9-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter); //printlog
#endif
		}
		dm9051_reset(db);	
		#if DEF_SPIRW
		iiow(db, DM9051_FCR, FCR_FLOW_ENABLE);	/* Flow Control */
		iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
#ifdef DM_CONF_POLLALL_INTFLAG
		#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE

			//#if defined DM_CONF_INTERRUPT_TEST_DTS_FALLING
			//	iiow(db, DM9051._INTCR, 0x01); //low active
			if (db->irq_type & (IIRQ_TYPE_LEVEL_HIGH | IIRQ_TYPE_EDGE_RISING) )
				iiow(db, DM9051_INTCR, 0x00); //high active(default)
				//iiow(db, DM9051._INTCR, 0x01);  //test
			//#elif defined DM_CONF_INTERRUPT_TEST_DTS_RISING
			//	iiow(db, DM9051._INTCR, 0x00); //high active(default)
			else
				iiow(db, DM9051_INTCR, 0x01); //low active
				//iiow(db, DM9051._INTCR, 0x00); //test
			//#endif
			
		#else
			#if (DRV_IRQF_TRIGGER == IRQF_TRIGGER_LOW)
			iiow(db, DM9051_INTCR, 0x01); //low active
			#else
			iiow(db, DM9051_INTCR, 0x00); //high active(default)
			#endif
		#endif
		//.iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM); //...
#else
	     	//.iiow(db, DM9051_IMR, IMR_PAR);
#endif
		//..iiow(db, DM9051_IMR, IMR_PAR); //=int_reg_stop(db); 
	     	//iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
	     	//iiow(db, DM9051_RCR, db->rcr_all);
	     	#endif
		bcopen_rx_info_clear(&db->bC);
		DM9051_fifo_reset_flg = 1;
	     	return; 
     }
     if (state==1 || state==2 || state==3 || state==5)
     {
		if (hstr)
		{
			db->rx_rst_quan = 0;
			++db->bC.DO_FIFO_RST_counter;
			Disp_RunningEqu_Ending(db);

			if (state == 1)
#ifdef ON_RELEASE
				rel_printk2("dm9-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter);
#else
				printk("dm9-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter); //"LenNotYeh", " %d", db->bC.DO_FIFO_RST_counter
#endif
			else if (state == 3)
			{
#ifdef ON_RELEASE
				rel_printk2("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db->bC.RxLen, 
						db->bC.DO_FIFO_RST_counter, db->bC.RXBErr_counter);
#else
				if (db->mac_process)          
					printk("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db->bC.RxLen, 
						db->bC.DO_FIFO_RST_counter, db->bC.RXBErr_counter);
				else
					printlog("dm9-%s Len %03d RST_c %d (RXBErr %d)\n", hstr, db->bC.RxLen, 
						db->bC.DO_FIFO_RST_counter, db->bC.RXBErr_counter);
#endif
			}
			else  if (state == 2) // STATE 2
			{
#ifdef ON_RELEASE
				rel_printk2("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter);
#else
				if (db->mac_process)          
					printk("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter);
				else
					printlog("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter); //"Len %03d ", db->bC.RxLen
#endif
			}
			else // STATE 5
			{
#ifdef ON_RELEASE
				rel_printk2("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter);
#else
				if (db->mac_process)          
					printk("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter);
				else
					printlog("dm9-%s (YES RST)(RXBErr %d)\n", hstr, db->bC.RXBErr_counter);
#endif
			}
		}
		dm9051_reset(db);
		#if DEF_SPIRW
		iiow(db, DM9051_FCR, FCR_FLOW_ENABLE);	/* Flow Control */
		if (hstr)
			iiow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
		else
		{
			pk= ior(db, DM9051_PPCR);
			iow(db, DM9051_PPCR, PPCR_SETTING); /* Fully Pause Packet Count */
		}
#ifdef DM_CONF_POLLALL_INTFLAG	
		#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
			//#if defined DM_CONF_INTERRUPT_TEST_DTS_FALLING
			//	iiow(db, DM9051._INTCR, 0x01); //low active
			if (db->irq_type & (IIRQ_TYPE_LEVEL_HIGH | IIRQ_TYPE_EDGE_RISING) )
				iiow(db, DM9051_INTCR, 0x00); //high active(default)
				//iiow(db, DM9051_INTCR, 0x01); //test
			//#elif defined DM_CONF_INTERRUPT_TEST_DTS_RISING
			//	iiow(db, DM9051._INTCR, 0x00); //high active(default)
			else
				iiow(db, DM9051_INTCR, 0x01); //low active
				//iiow(db, DM9051_INTCR, 0x00); //test
			//#endif	
		#else
			#if (DRV_IRQF_TRIGGER == IRQF_TRIGGER_LOW)
			iiow(db, DM9051_INTCR, 0x01); //low active
			#else
			iiow(db, DM9051_INTCR, 0x00); //high active(default)
			#endif
		#endif
	     	//.iiow(db, DM9051_IMR, IMR_PAR | IMR_PTM | IMR_PRM);
#else
	     	//.iiow(db, DM9051_IMR, IMR_PAR);
#endif
			//..iiow(db, DM9051_IMR, IMR_PAR); //=int_reg_stop(db); 
	     	//iiow(db, DM9051_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);
	     	//iiow(db, DM9051_RCR, db->rcr_all);
		#endif
		bcopen_rx_info_clear(&db->bC);
		DM9051_fifo_reset_flg = 1;
	     	return; 
	 }
     //if (state==2){ 
	    //printk("------- ---end--- -------\n");
	    //printk("Hang.System.JJ..\n");
	    //while (1);
	    
	    //("------- ---NO Do_reset( xx RxbErr ) --- -------\n");
     	//	if (hstr) printk("dm9-%s Len %03d (NO RST)(RXBErr %d)\n", hstr, db->bC.RxLen, db->bC.RXBErr_counter);
	    // 	bcopen_rx_info_clear(&db->bC);
     //}
     return; 
}

// when reset: return 1
int dm9051_fifo_ERRO(int ana_test, u8 rxbyte, board_info_t *db)
{	
	char hstr[72];
	if (db->bC.rxbyte_counter==5 || /*db->bC.rxbyte_counter0==(NUMRXBYTECOUNTER-1)*/ db->bC.rxbyte_counter0==NUMRXBYTECOUNTER) {
	     
	    db->bC.RXBErr_counter++;
	    
	    #if 0 
	     //one_and_two_and_three all the same!
	     printk("RXBErr %d: %04x/%04x. rxb=%02X, rxb_cntr,cntr0 %d,%d \n", db->bC.RXBErr_counter, 
			db->rwregs[0], db->rwregs[1], rxbyte, db->bC.rxbyte_counter, db->bC.rxbyte_counter0);
	    #endif
	     
	     if (/*1*/ ana_test < 3 ) { /* tested-check-ok: if (!(db->bC.RXBErr_counter % 3)) */
	      sprintf(hstr, "dmfifo_reset( 03 RxbErr ) rxb=%02X .%04x/%04x", rxbyte, db->rwregs[0], db->rwregs[1]);
	      dm9051_fifo_reset(3, hstr, db);
	     
		  //u16 calc= dm9051_rx_cap(db);
	      //printk("( RxbErr_cnt %d ) %d ++ \n", db->bC.RXBErr_counter, db->bC.rxbyte._counter0_to_prt);
	      //printk("rxb=%02X rxWrRd .%02x/%02x (RO %d.%d%c)\n", rxbyte, 
	      //  db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
	      //if (!(db->bC.RXBErr_counter%5))
	      //{
	      //.driver_dtxt_disp(db);
	      //.driver_dloop_disp(db);
	      //}
	      //dm9051._fifo_reset(1, "dm9051._fifo_reset( RxbEr )", db);
	     
	      dm9051_fifo_reset_statistic(db);
	      return 1;
	     } 
	     else {                                                                                                                                                                                                                                                              
		if (db->mac_process)
		{
			sprintf(hstr, "Or. Do_reset( 02 RxbErr, macError-clear ) rxb=%02X .%04x/%04x", rxbyte, db->rwregs[0], db->rwregs[1]); //printk("macError {Just treat as a normal-unicast.}\n"); //, Get recover-clear
			dm9051_fifo_reset(5, hstr, db);
		}
		else
		{
			sprintf(hstr, "Or. Do_reset( 02 RxbErr ) rxb=%02X .%04x/%04x", rxbyte, db->rwregs[0], db->rwregs[1]);	
			dm9051_fifo_reset(2, hstr, db); //bcopen_rx_info_.clear(&db->bC); // as be done in 'dm9051._fifo_reset'
		}
		db->mac_process = 0;
	     }
	}
	return 0;
} 
#endif

#if 1

//#define ior		dm9.iorb
//#define iow		dm9.iowb
#define dm9051_spi_read_reg		dm9.iorb
#define dm9051_spi_write_reg	dm9.iowb                

//[return 1 ok]
static int  device_polling(board_info_t *db, u8 erre_bit, u8 expect)
{
	int i;
	u8 tmp;
	for (i=0; i< 1000; i++)
	{
		mdelay(1); //delay
		tmp = dm9051_spi_read_reg(db, DM9051_EPCR);
		if ((tmp&erre_bit) == expect) //ready
			break;
	}
	if (i==1000) {
		printk("[dm9 read.write eeprom time out] on polling bit : 0x%02x (but want 0x%02x)\n", tmp, expect);
		return 0;
	}
	//printk("[dm9 polling process done] polling bit: 0x%02x (read 0x%02x) succeed-read-times %d\n", tmp&erre_bit, expect, i);
	return 1; //OK
}

//[of spi_user.c(used by 'dm9051_ethtool_ops')]
static void dm9051_read_eeprom(board_info_t *db, int offset, u8 *to)
{
#if DEF_SPIRW        
	int pr;
        mutex_lock(&db->addr_lock);

        dm9051_spi_write_reg(db, DM9051_EPAR, offset);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_ERPRR);

        pr = device_polling(db, EPCR_ERRE, 0x00); //while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

        dm9051_spi_write_reg(db, DM9051_EPCR, 0x0);

        to[0] = dm9051_spi_read_reg(db, DM9051_EPDRL);
        to[1] = dm9051_spi_read_reg(db, DM9051_EPDRH);
if (pr) {
	printk("dm9 [read Word %d][polling OK] : %02x %02x\n", offset, to[0], to[1]);
}
        mutex_unlock(&db->addr_lock);
#endif        
}

/*
 * Write a word data to SROM
 */
static void dm9051_write_eeprom(board_info_t *db, int offset, u8 *data)
{
#if DEF_SPIRW        
	int pr;
        mutex_lock(&db->addr_lock);

        dm9051_spi_write_reg(db, DM9051_EPAR, offset);
        dm9051_spi_write_reg(db, DM9051_EPDRH, data[1]);
        dm9051_spi_write_reg(db, DM9051_EPDRL, data[0]);
        dm9051_spi_write_reg(db, DM9051_EPCR, EPCR_WEP | EPCR_ERPRW);

        pr = device_polling(db, EPCR_ERRE, 0x00); //while ( dm9051_spi_read_reg(db, DM9051_EPCR) & EPCR_ERRE) ;

        dm9051_spi_write_reg(db, DM9051_EPCR, 0);
if (pr){
	printk("dm9 [write Word %d][polling OK] : %02x %02x\n", offset, data[0], data[1]);
}

        mutex_unlock(&db->addr_lock);
        
        //[my delay]
        //printk("dm9 [write Word %d][delay task]\n",  offset);
        mdelay(1); //delay
        mdelay(2); //delay
        mdelay(3); //delay
#endif        
}
#endif

//[skb_rx_head_c]
				     
#define RWREG1_START  0x0c00
#define RWREG1_END    0x3FFF
#define RWREG1_OVSTART  0x4000

struct dm9051_rxhdr0 { //old
	u8	RxPktReady;
	u8	RxStatus;
	__le16	RxLen;
} __packed;

struct spi_rxhdr { //new
	u8	padb;
	u8	spiwb;
	struct dm9051_rxhdr {
	  u8	RxPktReady;
	  u8	RxStatus;
	  __le16	RxLen;
	} rxhdr;
} __packed;          

/* Common cap calc usage */

void read_rwr(board_info_t *db, u16 *ptrwr)
{
#if DEF_SPIRW	
	*ptrwr= ior(db, 0x24); //v.s. 'DM9051_MRRL'
	*ptrwr |= (u16)ior(db, 0x25) << 8;  //v.s. 'DM9051_MRRH'
#endif	
}
void read_mrr(board_info_t *db, u16 *ptrmrr)
{
#if DEF_SPIRW	
	*ptrmrr= ior(db, DM9051_MRRL);
	*ptrmrr |= (u16)ior(db, DM9051_MRRH) << 8; 
#endif	
}

u16 dm9051_calc(u16 rwregs0, u16 rwregs1)
{
	u32 digiCalc;
	u32 digi, dotdigi;
	u16 calc;
	
	if (rwregs0>=rwregs1)
		digiCalc= rwregs0 - rwregs1;
	else
		digiCalc= 0x3400 + rwregs0 - rwregs1; //= 0x4000 - rwregs[1] + (rwregs[0] - 0xc00)
		
	digiCalc *= 100;
	digi= digiCalc / 0x3400;
	
	dotdigi= 0;
	digiCalc -= digi * 0x3400;
	if (digiCalc>=0x1a00) dotdigi= 5;
	
	calc= ((digi << 8) + dotdigi);
	return calc;
}

u16 dm9051_rx_cap(board_info_t *db)
{
	u16 rwregs[2];
	read_rwr(db, &rwregs[0]);
	read_mrr(db, &rwregs[1]);
	db->rwregs[0]= rwregs[0]; // save in 'rx_cap'
	db->rwregs[1]= rwregs[1];
	db->calc= dm9051_calc(rwregs[0], rwregs[1]);
	return db->calc;   
}


//[skb_rx_core_c]
//#if DM_CONF_APPSRC
//#endif 

 #define DM_TYPE_ARPH	0x08
 #define DM_TYPE_ARPL	0x06
 #define DM_TYPE_IPH	0x08
 #define DM_TYPE_IPL	0x00
static bool chk_data(board_info_t *db, struct dm9051_rxhdr *prxhdr, u8 *rdptr)
{
	struct net_device *dev = db->ndev;
	u8 *phd= (u8 *) prxhdr;
	u8 flg_disp = 0;

	if ((phd[1] & 0x40) && (!(rdptr[0] &1)))
	{
		flg_disp = 1;
		//printk("\n[@dm9.multiErr start-with rxb= 0x%0x]\n", prxhdr->RxPktReady);
		dev->stats.rx_length_errors= 3;
		dev->stats.rx_crc_errors=              6;
		dev->stats.rx_fifo_errors=                 9;
		dev->stats.rx_missed_errors=                 12;
		dev->stats.rx_over_errors++;
		printk("\n[@dm9.multiErr (rxhdr %02x %02x %02x %02x)] mac %02x %02x %02x, %lu\n", phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2],
			dev->stats.rx_over_errors); //dev->stats
		#if 0
		printk("dm9.dmfifo_reset( 10 multiErr ) mac %02x %02x %02x rxhdr %02x %02x %02x %02x {multi before rst RST_c= %d}\n", 
				rdptr[0], rdptr[1], rdptr[2],
				phd[0], phd[1], phd[2], phd[3],
				db->bC.DO_FIFO_RST_counter);
		#endif
	}
	else if (rdptr[0]!=dev->dev_addr[0] || rdptr[1]!=dev->dev_addr[1] || rdptr[2]!=dev->dev_addr[2])
	{                                  
		if ((rdptr[4]==DM_TYPE_ARPH && rdptr[5]==DM_TYPE_ARPL) && (rdptr[12]!=DM_TYPE_ARPH || rdptr[13]!=DM_TYPE_ARPL)) // special in data-skip
			; // error=fail //;;[current has rdptr[12]/rdptr[13]]
		if ((rdptr[4]==DM_TYPE_IPH && rdptr[5]==DM_TYPE_IPL) && (rdptr[12]!=DM_TYPE_IPH || rdptr[13]!=DM_TYPE_IPL))  // special in data-skip
			; // error=fail //;;[current has rdptr[12]/rdptr[13]]
		else if (rdptr[0]&1) //'skb->data[0]'
			return true;
			
		#if 1
		//[01 00 9e 00] unicast and len is less 256 ;"Custom report 20210204/'lan_aging_error3.log'"
		if (phd[0]==0x01 && phd[1]==0x00 && phd[3]==0x00){
			printk("\n[@dm9.[warn] unknow uni-cast frame (hdr %02x %02x %02x %02x)] %02x %02x %02x %02x %02x %02x\n", 
				phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2], rdptr[3], rdptr[4], rdptr[5]);
			return true;
		}
			
		flg_disp = 1;
		//if (db->mac_process) { //"[ERRO.found.s]"
			//char hstr[72];
			//sprintf(hstr, "dmfifo_reset( 11 macErr ) mac %02x %02x %02x rxhdr %02x %02x %02x %02x", 
			//	rdptr[0], rdptr[1], rdptr[2],
			//	phd[0], phd[1], phd[2], phd[3]);
			//db->mac_process = 0;
			//db->bC.ERRO_counter++;
			//dm9051_fifo_reset(11, hstr, db);
			//dm9051_fifo_reset_statistic(db);
			//return false;
		//} else {
			//"[ERRO.found.s treat as no-error]"
			//printk("\n[@dm9.macErr start-with rxb= 0x%0x]\n", prxhdr->RxPktReady);
			dev->stats.rx_frame_errors++;
			printk("\n[@dm9.frame error (hdr %02x %02x %02x %02x)] [%02x %02x %02x %02x %02x %02x] %lu\n", 
				phd[0], phd[1], phd[2], phd[3], rdptr[0], rdptr[1], rdptr[2], rdptr[3], rdptr[4], rdptr[5],
				dev->stats.rx_frame_errors);
			#if 0
			printk("dm9.dmfifo_reset( 11 macErr ) mac %02x %02x %02x rxhdr %02x %02x %02x %02x {before rst RST_c= %d}\n", 
				rdptr[0], rdptr[1], rdptr[2],
				phd[0], phd[1], phd[2], phd[3],
				db->bC.DO_FIFO_RST_counter); //(quick)
			#endif
			//"(This got return true!!)"    
			//db->mac_process = 1 - db->mac_process;
		//}
		#endif    
	}
	//else
	//{
	//	if (db->mac_process) printk("macError-clear {rx-unicast %02x %02x %02x rxhdr %02x %02x %02x %02x}\n", 
	//		rdptr[0], rdptr[1], rdptr[2],
	//		phd[0], phd[1], phd[2], phd[3]);
	//	db->mac_process = 0;
	//}             
	
	if (flg_disp)
	{
		//packet ...    
		#if MSG_DBG
		u16 calc;
		calc= dm9051_rx_cap(db);   
		printk("[dm9] %02x/%02x (scan_enter)\n", db->rwregs_enter, db->rwregs1_enter); //[trick.extra]
		printk("[dm9] %02x/(wrp scan.end)\n", db->rwregs_scanend ); //[trick.extra]
		printk("[dm9] %02x/%02x (scan.end RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%');
		#endif
		//  printk("[dm9] ior-mac %02X %02X %02X %02X %02X %02X\n",
		//	    ior(db, 0x10), ior(db, 0x11), 
		//	    ior(db, 0x12), ior(db, 0x13), 
		//	    ior(db, 0x14), ior(db, 0x15));
		//  printk("[dm9] ior-RCR 0x%02X\n", ior(db, DM9051_RCR));
		#if 0
		printnb_packet(rdptr, prxhdr->RxLen-4);         
		printnb_packet(&rdptr[prxhdr->RxLen-4], 4);
		#endif
		return false;
	}
	//if (db->flg_rxhold_evt)
	//	printk("[dm9].encounter NN-cast, dm9051__chk_data, ...End...\n");
	
	db->bC.rx_unicst_counter++;
	db->nSCH_UniRX++;
	return true;
}

struct sk_buff *SKB_trans(struct net_device *dev, char * buffp, int RxLen, int SKB_size)
{
	board_info_t *db = netdev_priv(dev);     
	struct sk_buff *skb;
	u8 *rdptr;
	u8 rxbyte = db->bC.dummy_pad;
	static int asr_rsrv_test = 0;
#ifdef ASL_RSRV_RX_HDR
	if ((skb = dev_alloc_skb(SKB_size)) == NULL)  {
			printk("dm9051 [!ALLOC skb size %d fail]\n", SKB_size);
			return NULL;
	}
#else
	if ((skb = dev_alloc_skb(SKB_size - ASL_RSRV_RX_HDR_LEN)) == NULL)  {
			printk("dm9051 [!ALLOC skb size %d fail]\n", SKB_size - ASL_RSRV_RX_HDR_LEN);
			return NULL;
	}
#endif
	if (asr_rsrv_test==0) {
		//asr_rsrv_test = 1;
		printk("[dm9].peek ------------ RxLen 4 RSRV.s= %d %d %d---------\n", RxLen,
			 4, ASL_RSRV_RX_HDR_LEN);
		printk("[dm9].peek ------------ skb_len.s= %d -------------------\n", skb->len);
	}   
	skb_reserve(skb, 2);
	
	if (asr_rsrv_test==0) {
		printk("[dm9].peek ------------ skb_put.i(x), x= %d -------------------\n", RxLen - 4);
		printk("[dm9].peek ------------ skb_len.s(put)= %d -------------------\n", skb->len);
	}
	
	/* A pointer to the first byte of the packet */
	rdptr = (u8 *) skb_put(skb,  RxLen - 4);  // [save 4] SKB_size - 4 - ASL_RSRV_RX_HDR_LEN - 4
	memcpy(rdptr, buffp, RxLen - 4); // [save 4] &sbufp[p]
	
	if (asr_rsrv_test==0) {
		printk("[dm9].peek ------------ skb_len.e(put)= %d -------------------\n", skb->len);
	}                   
	// ?? if (!dm9051_chk_data(db, rdptr, RxLen))
	//	return nRx; ? rwregs1; ?          
	dev->stats.rx_bytes += RxLen;
	skb->protocol = eth_type_trans(skb, dev);  // [JJ found: skb->len -= 14]
		 
	if (asr_rsrv_test==0) {
		asr_rsrv_test = 1;
		printk("[dm9].peek --- due to eth_type_trans(skb, dev), skb->len -= 14 ---\n");
		printk("[dm9].peek ------------ skb_len.e= %d -------------------\n", skb->len);
		printk("[dm9].peek ------------ skb_alloc.is= %d -------------------\n", RxLen + 4 + ASL_RSRV_RX_HDR_LEN);
	}
	
	if (dev->features & NETIF_F_RXCSUM) {
		if ((((rxbyte & 0x1c) << 3) & rxbyte) == 0)
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb_checksum_none_assert(skb);
	}
	
	rel_printk6("[DM9].netif_rx: skb->len %d\n", skb->len);
	printnb_packet6(skb->data, 32); 
	
	if (in_interrupt())
		netif_rx(skb);
	else
		netif_rx_ni(skb);

	dev->stats.rx_packets++;  
	return skb;
}

u8 sb_scan_enter = 0;
u8 sb_scan_enter1 = 0;
u16 SB_skbing(struct net_device *dev, char *sbufp, u16  rwregs1)
{
	board_info_t *db = netdev_priv(dev);     
	struct dm9051_rxhdr rxhdr;   
	//[u8 rxbyte;]
	int prevl = 0, RxLen;
	int prevp = 0, p = 1;
	static int diff = 0;

	db->validlen_for_prebuf = 0; //init.
	while(sbufp[p] == DM9051_PKT_RDY)
	{                                                                     
		db->bC.dummy_pad = sbufp[p];
		memcpy((char *)&rxhdr, &sbufp[p], sizeof(rxhdr));
		RxLen = rxhdr.RxLen;
		
		//[read.mem.buf's]
		if ((p+sizeof(rxhdr)) <= (SCAN_LEN_HALF+1)) //RWREG1_END
		{
			p += sizeof(rxhdr);
			db->validlen_for_prebuf = p;
		} 
		else {
			rel_printk6("[trace.end1 reg1 %x]\n", rwregs1);
			goto skbing_end; //return rwregs1;
		}

		//[read.mem.buf's]
		if ((p+RxLen ) <= (SCAN_LEN_HALF+1)) //RWREG1_END
			;  //[go skb-trans data and then move pointer 'p']
		else {
			rel_printk6("[trace.end2 reg1 %x]\n", rwregs1);
			goto skbing_end; //return rwregs1;
		}

#if 0
		//_check_data
#else
		if (!chk_data(db, (struct dm9051_rxhdr *) &sbufp[p-sizeof(rxhdr)], &sbufp[p])) //&sbufp[p] = skb->data
		{
			//do {       
			//[simple NCR reset]   
				//"[ERRO.found.s]"
				#if MSG_DBG
				  struct dm9051_rxhdr *prxhdr = (struct dm9051_rxhdr *) &sbufp[p-sizeof(rxhdr)];
				#endif
				
				#ifdef ON_RELEASE
				#if MSG_DBG
				  dm9051_rx_cap(db);
				  printk("[dm9] %02x/%02x (again scan.end RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], db->calc>>8, db->calc&0xFF, '%'); 
				  printk("[dm9] skbing.on %02x/%02x while.acc.nRx %d macErr.RxLen %d\n", 
					db->rwregs[0], rwregs1,
					db->nRx, prxhdr->RxLen);
				#endif
				#endif
				
				do {         
					  char hstr[72];
					  sprintf(hstr, "dmfifo_reset( 11 macErr ) mac %02x %02x %02x %02x rxhdr %02x %02x %02x %02x (quick)", 
						sbufp[p], sbufp[p+1], sbufp[p+2], sbufp[p+3],
						sbufp[p-4], sbufp[p-3], sbufp[p-2], sbufp[p-1]);
					//.printk("dm9.s-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter);
					db->bC.ERRO_counter++;
					dm9051_fifo_reset(11, hstr, db);
					dm9051_fifo_reset_statistic(db);
				} while (0);
				
				#ifdef ON_RELEASE
				  dm9051_rx_cap(db);
				#if MSG_DBG
				  printk("dm9.e-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter);
				  printk("[dm9] %02x/%02x (end RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], 0, 0, '%'); 
				#endif
				#if MSG_REL
				  printk("[dm9] %02x/%02x (r%lu o%lu f%lu RST_c %d)\n", db->rwregs[0], db->rwregs[1], 
					dev->stats.rx_errors, dev->stats.rx_over_errors, dev->stats.rx_frame_errors, db->bC.DO_FIFO_RST_counter);  //"end RO %d.%d%c", 0, 0, '%', 
				#endif
				#endif
				return 0x0c00;
			//} while(0);
		}
#endif
		prevp = p;
		prevl= RxLen;
		if (!SKB_trans(dev, &sbufp[p], RxLen, RxLen + 4 + ASL_RSRV_RX_HDR_LEN)) {  //[or compact as (RxLen - 4 + ASL_RSRV_RX_HDR_LEN)]
				printk("[trace.end3 reg1 %x]\n", rwregs1); // not 'rel_printk6', wait happen to do analysis
				goto skbing_end; //return rwregs1;
		}
		p += RxLen ;
		db->validlen_for_prebuf = p;
		db->nRx++;    
		//[db->rx_count++;]
		//[copying skb..]

		//{chip's}
		rwregs1 += sizeof(rxhdr) + RxLen;
		rel_printk6("[trace.on reg1 %x]\n", rwregs1);
		if (rwregs1 > RWREG1_END)
		{
			rwregs1 -= RWREG1_OVSTART;
			rwregs1 += RWREG1_START;
		}

		//[read.mem.buf's]
		if (p > (SCAN_LEN_HALF+1)) //'RWREG1_END'
		{
			printk("[trace.end4 reg1 %x]\n", rwregs1); // not 'rel_printk6', wait happen to do analysis
			goto skbing_end; //return rwregs1;
		}
	}
	rel_printk_last("[trace.end5 reg1 %x]\n", rwregs1); // not 'rel_printk6', wait happen to do analysis
skbing_end:
	if (db->nRx != diff)
	{
		if (sb_scan_enter && db->nRx > 32)
		{
			sb_scan_enter++;
			if (sb_scan_enter > 1) //2
				sb_scan_enter = 0;
			printk("        ---------------------------------- [%d PKs dm9LastRxb] %02x\n", db->nRx, db->bC.last_ornext_pad);
		}
		diff = db->nRx;
	}
	return rwregs1;
}         

char *SB_scan0(board_info_t *db, char *sbuff)
{
	u8 rxbyte;        
	int RxLen;
	struct dm9051_rxhdr *prxhdr;
	char bf1;
	//char *inblkSTARTP = NULL;
	u16 calc;
	int p = 1;

	db->validlen_for_prebuf = 0; //init.
	/*
	 * If this spi read xfer, over-read than rx_sram_writte_in is not allowed~
	 * MOdify to be spi-read-xfer by sizeof(rxhdr) and then rxhdr.RxLen,
	 * Repeat untill no rxbyte(0x01) or the todo spi-read-xfer will 
	 * over-than totally SCAN_LEN_HALF bytes.
	 */

	db->bC.isbyte = ior(db, DM9051_ISR);
	calc= dm9051_rx_cap(db);
db->bC.isr_clear = 0;	// 20210201.RX_Pointer_Protect

	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
	if (rxbyte != DM9051_PKT_RDY)
	{                                                                                                                                                                                                                                                                                                                                                           
		db->bC.last_ornext_pad = rxbyte; // We also think below line, that sbuff[p] = rxbyte, is also OK.
		sbuff[p] = 0xcc; // We like to make completely, NOT 0x01, so everything after this could not accident find the dummy-0x01 head-rxb
		db->rwregs_scanend = db->rwregs[0]; //[trick.extra]
		if (!db->bC.isr_clear)
			iow(db, DM9051_ISR, 0xff);
		return NULL; //inblkSTARTP;
	//inblkSTARTP = sbuff;
	}

	rel_printk5("(dm9.rxbyte [step %02d]) %02x, rxptrs %x/%x isr %02x\n", 101 , rxbyte, db->rwregs[0], db->rwregs[1], db->bC.isbyte);

	do 
	{
		rel_printk5("(dm9.rxbyte [step %02d]) p= %d {%d, %d}\n", 111 , p, p+sizeof(struct dm9051_rxhdr), RWREG1_OVSTART);

		if ((p+sizeof(struct dm9051_rxhdr)) > (SCAN_LEN_HALF+1)) //[not 'RWREG1_OVSTART']
		{                   
			if (sb_scan_enter1)
			{
				sb_scan_enter1 = 0;
				printk("        ---------------------------------- [scan.rxhdr exaust-sbuf %d + %u] > %d\n", p, (unsigned int) sizeof(struct dm9051_rxhdr), (SCAN_LEN_HALF+1));
			}
			break;
		}

		dm9051_inblk_noncpy(db, &sbuff[p-1], sizeof(struct dm9051_rxhdr)); /*rdptr-1*/    /* destroy previous pkt's crc's 4th byte.*/
		sbuff[p-1] = 0xcc; // Be Destroied, mark to be 0xcc */
	if (!db->bC.isr_clear){
		iow(db, DM9051_ISR, 0xff);
		db->bC.isr_clear = 1;
	}

		rel_printk5("(dm9.rxbyte [step %02d]) %02x, %02x %02x %02x %02x\n", 112 , sbuff[p-1], 
			sbuff[p], sbuff[p+1], sbuff[p+2], sbuff[p+3]);
		
		prxhdr = (struct dm9051_rxhdr*) &sbuff[p];
		RxLen = prxhdr->RxLen;
		p += sizeof(struct dm9051_rxhdr);  
		db->validlen_for_prebuf = p; //[scan0]

		if ((p+RxLen) > (SCAN_LEN_HALF+1)) //[not 'RWREG1_OVSTART']
		{
			if (sb_scan_enter1)
			{
				sb_scan_enter1 = 0;
				printk("        ---------------------------------- [scan.payld exaust-sbuf %d + %d] > %d\n", p, RxLen, (SCAN_LEN_HALF+1));
			}
			break;
		}

		db->bC.isr_clear = 0;
		#ifdef DM_CONF_ANY_BUF_CASE_SKB_RX_CORE
		  do {
			  int n = RxLen;
			  while (n > ANY_BUF_NUM){
				  bf1 = sbuff[p-1];
				  dm9051_inblk_noncpy(db, &sbuff[p-1], ANY_BUF_NUM); /*rdptr-1*/
				  sbuff[p-1] = bf1;
				  p += ANY_BUF_NUM;
				  n -= ANY_BUF_NUM;
			  }
			  if (n){
				  bf1 = sbuff[p-1];
				  dm9051_inblk_noncpy(db, &sbuff[p-1], n); /*rdptr-1*/
				  sbuff[p-1] = bf1;
				  p += n;
			  }
		  } while (0);
		#else
		  bf1 = sbuff[p-1];
		  dm9051_inblk_noncpy(db, &sbuff[p-1], RxLen); /*rdptr-1*/
		  sbuff[p-1] = bf1;
		  p += RxLen;
		#endif
	//.if (!db->bC.isr_clear){
	iow(db, DM9051_ISR, 0xff);
	db->bC.isr_clear = 1;
	//.}

db->bC.isbyte = ior(db, DM9051_ISR); //calc= dm9051_rx_cap(db);
db->bC.isr_clear = 0;	

		db->validlen_for_prebuf = p; //[scan0]

		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
		if (rxbyte != DM9051_PKT_RDY)
		{
			db->bC.last_ornext_pad = rxbyte; // We also think below line, that sbuff[p] = rxbyte, is also OK.
			sbuff[p] = 0xcc; // We like to make completely, NOT 0x01, so everything after this could not accident find the dummy-0x01 head-rxb
			break;
		}
	} while (1);
	sbuff[p] = 0; // in case, to not 0x01.
if (!db->bC.isr_clear)
	iow(db, DM9051_ISR, 0xff);

	/*
	//.db->bC.isbyte = ior(db, DM9051_ISR);
	*/

	calc= dm9051_rx_cap(db);
	db->rwregs_scanend = db->rwregs[0]; //[trick.extra]
	
	/*
	//.rel_printk5("(dm9.rxbyte [step %02d]) %02x, rxptrs %x/%x isr %02x\n", 102 , rxbyte, db->rwregs[0], db->rwregs[1], db->bC.isbyte);
	*/

	#if 0
	/*
	 * This spi read xfer, EVER to large length, encounter fail result! 
	 */
	dm9051_inblk_noncpy(db, sbuff, SCAN_LEN_HALF); /*rdptr-1*/
	#endif

	return sbuff; //'inblkSTARTP'
}           

void SB_scan0_enter(board_info_t *db, u16 rwregs1)
{
	static u32 disp_tot_quan = 0;
	u32 m, sm;
	int mf, smf;
	if (db->rwtrace1 < rwregs1)
	{
		if (db->rwtrace1)
		{                                                             
			db->rx_rst_quan += 13;
			db->rx_tot_quan += 13;
		}
		
		if (db->rx_tot_quan > disp_tot_quan)
		{
			sb_scan_enter = 1;
			sb_scan_enter1 = 1;
			disp_tot_quan += 35840; // 35 m
				//102400; // 100 M
				//35840; // 35 m
				//512; // 0.5 M
			m = db->rx_tot_quan / 1024;
			sm = db->rx_rst_quan / 1024;
			mf = smf= 0;
			if ((db->rx_tot_quan - (m * 1024)) >= 512)  mf= 5;
			if ((db->rx_rst_quan - (sm * 1024)) >= 512)  smf= 5;
			
			#ifdef DM_CONF_ANY_BUF_CASE_SKB_RX_CORE
			printk("\n");
			printk("               <dm9 note: Testing %d bytes ANY_BUF_CASE_SKB_RX_CORE> \n", ANY_BUF_NUM);
			#endif
			
			printk("        ---------------------------------- <dm9 %04x [%d.%d M]/ %d.%d M> \n", 
				rwregs1, sm, smf, m, mf);		// "(rst %d)", db->bC.DO_FIFO_RST_counter
		}
	}
	db->rwtrace1 = rwregs1;
}

static int dm9000_rx(struct net_device *dev)  //.....
{
	board_info_t *db = netdev_priv(dev);
	char *sbufp;   

//static int dbg_nstrip = 6; 	
	//if (dbg_nstrip==0)  //.
	//	return 0; //.
	//dbg_nstrip--; //.
	
	db->nRx = 0;
	SB_scan0_enter(db, db->rwregs1);
	sbufp = SB_scan0(db, db->sbuf); //= SB_scan(db, db->sbuf, db->scanmem_sta);
	if (sbufp)
	{
		db->scan_end_reg0= db->rwregs[0]; // Save wr on (Last.ScanEnd)
		rel_printk5("[Scan Len : 0x%x (%d)]\n", db->validlen_for_prebuf, db->validlen_for_prebuf);
		rel_printk_last(" wt.s rwregs1 0x%x\n", db->rwregs1); //"%02x %02x" db->rwregs1 & 0xff, (db->rwregs1 >> 8) & 0xff
		printnb_packet5(sbufp, 17); // 32, min(db->validlen_for_prebuf - 4, 32)
		printnb_packet0(&sbufp[db->validlen_for_prebuf-4], 4);
		db->rwregs1 = SB_skbing(dev, sbufp, db->rwregs1);
		db->last_nRx = db->nRx;
		db->nSCH_GoodRX += db->nRx;
		
		read_rwr(db, &db->last_rwregs[0]); // Save every wr (Last.SkbingEnd)
		db->last_rwregs[1] = db->rwregs1;     // Save every rd
		db->last_calc = dm9051_calc(db->last_rwregs[0], db->last_rwregs[1]);
		
		if (db->rwregs1==0x0c00)
		{
			return db->nRx;
		}
	#ifndef FREE_NO_DOUBLE_MEM_MAX	// [do not copy for better performance, also with saving memory!]
		memcpy(db->prebuf, sbufp, db->validlen_for_prebuf); // not 'SCAN_LEN_HALF+1'
	#endif
		rel_printk_last(" [skbing : 0x%x (%d)]\n", db->validlen_for_prebuf, db->validlen_for_prebuf);
		
		iow(db, DM9051_MRRL, db->rwregs1 & 0xff);
		iow(db, DM9051_MRRH, (db->rwregs1 >> 8) & 0xff);
		rel_printk_last(" wt.e rwregs1 0x%x\n\n", db->rwregs1); //"%02x %02x" db->rwregs1 & 0xff, (db->rwregs1 >> 8) & 0xff
	}
	return db->nRx;
}

//[skb_rx_c]
// delayed_wrok_rx.c <----- [appsrc_user.c]

//#if DM_CONF_APPSRC

// -------------- rx operation skel --------------------

//static void dm9051_disp_hdr_s(board_info_t *db)
static void dm9051_disp_hdr_s_new(board_info_t *db)
{
	u16 calc= dm9051_rx_cap(db);
	//printk("dm9.hdrRd.s.%02x/%02x(RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%'); //hdrWrRd
	db->DERFER_rwregs[RXM_WrtPTR]= db->rwregs[0];
	db->DERFER_rwregs[MD_ReadPTR]= db->rwregs[1];
	db->DERFER_calc= calc;
}
//static void dm9051_disp_hdr_e(board_info_t *db, int rxlen)
static void dm9051_disp_hdr_e_new(board_info_t *db)
{
	//u16 calc= dm9051_rx_cap(db);
	//printk("hdrWrRd.e.%02x/%02x(RO %d.%d%c) rxLen(0x%x)= %d\n", db->rwregs[0], db->rwregs[1], calc>>8, calc&0xFF, '%', 
	//  rxlen, rxlen);
	u16 calc= dm9051_rx_cap(db);
	db->DERFER_rwregs1[RXM_WrtPTR]= db->rwregs[0];
	db->DERFER_rwregs1[MD_ReadPTR]= db->rwregs[1];
	db->DERFER_calc1= calc;
} 

static int /*irqreturn_t */
dm9051_isr_ext2(void *dev_id) //dm9051_isr_ext1(int irq, void *dev_id, int flag)
{
	struct net_device *dev = dev_id;
	board_info_t *db = netdev_priv(dev);
	unsigned int_status = 0;
	int nRx= 0;
	u8 flg_rx_cap =  0;
	static int nRx_counter0= 0;
	//static u8 eqEnter_count = 0;
	//static u8 neqEnter_count = 0;
	static int nNT_section_count = 0;
	//static u16 rwreg_equ = 0;

	#if 1
	//Add TX cap. (to be tested.)
	//dm9051_tx_irx(db);
	//dm9051_tx_work_disp(db);
	#endif

	// Received the coming packet 
#if 1
	//-----------------------------------
	//needed especially, for polling mode
	//-----------------------------------
	#if DRV_POLL_1
	u16 calc;
	flg_rx_cap = 1;
	calc= dm9051_rx_cap(db);
	if (db->rwregs[0]==db->rwregs[1])
	{
	  // [disp] [todo] Will make it only re-cycle is happen
	}
	else
	{
		//[trick.extra]
		if (flg_rx_cap )
			db->rwregs_enter = db->rwregs[0];
		else
			read_rwr(db, &db->rwregs_enter);
		//[trick.extra1]
		db->rwregs1_enter = db->rwregs1;
	#endif
#endif
		//u16 regs[2];
		//regs[0]= db->rwregs[0];
		//regs[1]= db->rwregs[1];
		//if (int_status & ISR_PRS)
		//	dm9000_rx(dev);
		//=
		#if 0
		u8 rxbyte;
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */
		rxbyte= ior(db, DM_SPI_MRCMDX);	/* Dummy read */	
		if ( rxbyte == DM9051_PKT_RDY)
		{
		#endif

		// Get DM9051 interrupt status 
#if DEF_SPIRW
		db->bC.isbyte= ior(db, DM9051_ISR);	// Got ISR
		#if !defined WR_ISR_ENDOF_RXWORK_ONLY	// need written at end of dm9051 _rx _work
			iiow(db, DM9051._ISR, db->bC.isbyte); //..tgbd,. // Clear ISR status
		#endif
#endif
			int_status = db->bC.isbyte;

			if (int_status & ISR_PRS)
			{
				nRx= dm9000_rx(dev);
				if (nRx==0) 
				{

				  nRx_counter0++;
				  if (!(nRx_counter0 % 25) ) {

				  	nNT_section_count++;
				  	if (nNT_section_count <= 5)
				  	{
						//printk("\n[@dm9.RxbErr start-with rxb= 0x%0x]\n", db->bC.last_ornext_pad);
						dev->stats.rx_errors++;
						printk("\n[@dm9.RX errors (rxb= 0x%0x)] %lu\n", db->bC.last_ornext_pad, dev->stats.rx_errors);
						#if MSG_DBG
						  printk("dm9.Tell-Last.ScanEnd %02x\n", db->scan_end_reg0);
						  printk("dm9.Tell-Last.SkbingEnd %02x/%02x (tod RO %d.%d%c), Last.nRx %d\n", 
							db->last_rwregs[0], db->last_rwregs[1], db->last_calc>>8, db->last_calc&0xFF, '%',
							db->last_nRx);
						  printk("Warn-(nRx 0), rxb_pad %02x, NT %d: When traffic, No dm9051-fifo-reset(11) nRx_counter0= %d\n", 
							db->bC.last_ornext_pad , nNT_section_count, nRx_counter0);
						#endif
					}
				  	if (nNT_section_count == 5)
						printk("Warn-(nRx 0), rxb_pad %02x ........... and more ............\n", db->bC.last_ornext_pad);

					#if 1
					do {       
					//[simple NCR reset]   
						//"[ERRO.found.s]"
						char hstr[82];
						db->bC.RXBErr_counter++; //db->bC.ERRO_counter++;
						
						if (!flg_rx_cap)
							dm9051_rx_cap(db);
						
						#ifdef ON_RELEASE    
						#if MSG_DBG
						  printk("[dm9] %02x/%02x (tod RO %d.%d%c), now.dm9051_rx.return.nRx %d\n", // since (nRx==0)
							db->rwregs[0], db->rwregs[1], db->calc>>8, db->calc&0xFF, '%', nRx); 
						#endif
						#endif
						
						sprintf(hstr, "dmfifo_reset( 02 RxbErr ) ISR %02x %04x/%04x (at outside-caller nRx 0)", 
							db->bC.isbyte, 
							db->rwregs[0], 
							db->rwregs[1]);
						dm9051_fifo_reset(2, hstr, db);
						dm9051_fifo_reset_statistic(db);
						
						#ifdef ON_RELEASE
						  dm9051_rx_cap(db);
						#if MSG_DBG                                      
						  printk("dm9.e-%s Len %03d RST_c %d\n", hstr, db->bC.RxLen, db->bC.DO_FIFO_RST_counter);
						  printk("[dm9] %02x/%02x (end RO %d.%d%c)\n", db->rwregs[0], db->rwregs[1], 0, 0, '%'); 
						#endif
						#if MSG_REL
						  printk("[dm9] %02x/%02x (r%lu o%lu f%lu RST_c %d)\n", db->rwregs[0], db->rwregs[1], 
							dev->stats.rx_errors, dev->stats.rx_over_errors, dev->stats.rx_frame_errors, db->bC.DO_FIFO_RST_counter);  //"end RO %d.%d%c", 0, 0, '%', 
						#endif
						#endif
						return 0x0c00;
					} while(0);
					#endif
			
					  //"[ERRO.found.s]"
					 //..dm9051._fifo_reset(11, "dmfifo._reset( 11 )", db); //~= dm9051._fifo_reset(11, ...)
					  //.dm9051._fifo_reset_statistic(db);
					  //.int._status= ISR_CLR_STATUS;
				  }

				  goto end_of_dm9051_isr_ext;
				}
			}
			else
			{
				nRx= dm9000_rx(dev);
				#if 0
				//<No need>
				if (nRx==0) {
				  //"[ERRO.found.s]"
				  if (int_status & ISR_PRS)
				   printk("[NOT DO RST.. (!ISR_PRS)] ISR 0x%02x and-Zero-pkt ( nRx %d, totalRx %lu)\n", 
				     int_status, nRx, dev->stats.rx_packets);
				  //.int_status= ISR_CLR_STATUS; //return ISR_CLR_STATUS;
				  //.goto end_of_dm9051_isr_ext;
				}
				#endif
				#if 0
				//"[Special.found.s]", NT= NoteTemp
				//<Many case that get 1 frame below disp.>
				if (nRx) {
				   printk("[NT: SPECIAL.FND.. ISR 0x%02x (!ISR_PRS)] nRx= %d ( totalRx %lu)\n", 
				     int_status, nRx, dev->stats.rx_packets);
				}
				#endif
			}

			if (nRx) {
				nRx_counter0= 0;
				nNT_section_count = 0;
			}

		#if 0
		}
		#endif
		//.else
		//.{
		//.	if (int_status & ISR_PRS)
		//.	{
		//.		dm9000_rx(dev);
		//.		printk(ISRByte 0x%02x warnning rxbyt=0x%02x (add %d to %lu)\n", 
		//.		  int_status, rxbyte, nRx, dev->stats.rx_packets);
		//.	}
		//.}
#if 1
	#if DRV_POLL_1
	}
	#endif
#endif

	/* Receive Overflow */
	if (int_status & ISR_ROS) {

		if (nRx) 
				; //printk(" [ERR-ISR] ISR_ROS(rx-overflow), nRX= %d, NOT DO RST..\n", nRx);
		else {		
				char hstr[60];
				db->bC.OvrFlw_counter++;
		#if 0
				printk("( Rxb %d ) %d %d ++ \n", db->bC.isbyte, db->bC.OvrFlw_counter, db->bC.rxbyte._counter0_to_prt);
				
				if (!(db->bC.OvrFlw_counter%5))
				 {
				 //driver_dtxt_disp(db);
				 //driver_dloop_disp(db);
				 }
		#endif
		#if 0
				printk(" db_isbyte 0x%02x (%d ++)\n", db->bC.isbyte, db->bC.rxbyte._counter0_to_prt);
				printk(" int_status 0x%02x", int_status);
		#endif
				sprintf(hstr, "dmfifo_reset( rcv ovflw ) ERR-ISR= %02x", int_status);
				dm9051_fifo_reset(1, hstr, db);
				//printk(" [ERR-ISR= %02x] (overflow)", int_status); //dm9051_fifo_show_flatrx(" [ERR-ISR] (recieve overflow)"); // (--, db)
				//dm9051._fifo_reset(1, "dmfifo._reset(recieve overflow )", db); //early fifo_reset
				dm9051_fifo_reset_statistic(db);
		}

		//int_status= ISR_CLR_STATUS;
		goto end_of_dm9051_isr_ext;
	}
end_of_dm9051_isr_ext:

	//return int_status; //[Here! 'void' is OK]
	return nRx;
}
//#endif

//[sched_c]
/* [Schedule Work to operate SPI rw] */

/*(or DM9051_Schedule.c)*/
/*static int dm9051_sch_cnt(u16 ChkD) // large equ +50 or less -3
{
	static u16 SavC= 0;
	
	if (SavC != ChkD) {
		
		u16 LessC= 0;
		if (SavC > 3)
			LessC= SavC - 3;
		
		if (ChkD < LessC) { //SavC
			SavC= ChkD;
			return 1; //less and reduce
		}
		
		if (ChkD >= (SavC+50)) {
			SavC= ChkD;
			return 1;
		}
	}
	return 0;
}*/
static int dm9051_sch_cnt1(u16 getVAL, u16 offset) // large equ +1 (as increasment)
{
	static u16 Sav1= 0;
	
	if (!offset) // an option that always no need printed, so return 0. 
		return 0;
	
	//offset default is as 1
	//if (Sav1 != getVAL) {
		if (getVAL >= (Sav1+offset)) {
			Sav1= getVAL;
			return 1;
		}
	//}
	return 0;
}

//Testing...JJ5_DTS
#define GLUE_LICENSE_PHYPOLL		(3+2)
#define GLUE_LICENSE_INT			(3+1)
#define GLUE_LICENSE_LE_EXPIRE		(3-1)

static void 
dm9051_INTPschedule_isr(board_info_t *db, int sch_cause)
{
	//spin_lock(&db->statelock_tx_rx);//mutex_lock(&db->addr_lock);
	
	//.printk("R_SCH_XMIT %d (=%d) dm9051_start_xmit, Need skb = skb_dequeue(&db->txq) to get tx-data\n", R_SCH_XMIT, sch_cause);
	if (dm9051_sch_cnt1(db->nSCH_XMIT, 0)) //1500, 5500, 0
		printk("dm9-INFO TX %02d, sched R_SCH_XMIT %d (=%d) send skb_dequeue(txq)..\n", db->nSCH_XMIT, R_SCH_XMIT, sch_cause); //, db->rx_count
	
	db->sch_cause = sch_cause;
	
	#ifdef DM_CONF_POLLALL_INTFLAG
	if (sch_cause== R_SCH_INIT)
		return;
	if (sch_cause== R_SCH_INT_GLUE)
		return;
	//if (sch_cause== R_SCH._INFINI) 
	//	return;
	#endif
	
	switch (sch_cause)
	{
		case R_SCH_INIT:
			db->nSCH_INIT++; // if (m<250) m++; 
		//	schedule_delayed_work(&db->rx._work, 0); //dm9051_continue_poll
			break;
		case R_SCH_INFINI:
			db->nSCH_INFINI++;
		//	schedule_delayed_work(&db->rx._work, 0);  //dm9051_continue_poll
			break;
		//case R_SCH_LINK:
			//break;
		case R_SCH_PHYPOLL:
			break;
		case R_SCH_INT:
			db->nSCH_INT++;
		//	schedule_delayed_work(&db->rx._work, 0); //dm9051_continue_poll 
			break;
		case R_SCH_INT_GLUE:
			db->nSCH_INT_Glue++;
	#ifdef DM_CONF_POLLALL_INTFLAG		
			DM9051_int_token++; //.DM9051_int_token++;
	#endif	
			break;
		case R_SCH_XMIT:
	        //db->nSCH_XMIT++;
	#ifdef DM_CONF_POLLALL_INTFLAG	
			DM9051_int_token++;
	#endif			
			//.printk("(%d)dm9051_start_xmit, Need skb = skb_dequeue(&db->txq) to get tx-data\n", db->nSCH_XMIT);
			break;
	}
	
	#ifdef DM_CONF_TASKLET
	switch (sch_cause)
	{
		case R_SCH_INIT:
		case R_SCH_INFINI:
		//case R_SCH_LINK:
		case R_SCH_INT:
			tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0);
			break;
		case R_SCH_INT_GLUE:
			   tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0); 
			break;		
		case R_SCH_PHYPOLL:
#ifdef MORE_DM9051_MUTEX
			   tasklet_schedule(&db->phypoll_tl); //schedule_.delayed_work(&db->x, 0);
#else
			   tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0); 
#endif			   
			break;		
	#ifdef DM_CONF_POLLALL_INTFLAG		
		case R_SCH_XMIT:
#ifdef MORE_DM9051_MUTEX
			tasklet_schedule(&db->xmit_tl); //schedule_.delayed_work(&db->y, 0);
#else
			tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0);
#endif			   
			break;
	#endif			 	
	}
	#else //~DM_CONF_TASKLET
#if 1	
	//spin_lock(&db->statelock_tx_rx);//mutex_lock(&db->addr_lock);
	switch (sch_cause)
	{
  #ifdef DM_CONF_INTERRUPT
		#ifndef DM_CONF_THREAD_IRQ
		case R_SCH_INT:
			schedule_delayed_work(&db->rx_work, 0); 
			break;
		#endif
  #else
		case R_SCH_INIT:
		case R_SCH_INFINI: //'POLLING'
		case R_SCH_INT_GLUE:
		//case R_SCH_LINK:
			//schedule_delayed_work(&db->rx_work, 0); //dm9051_continue_poll
			//break;
			 schedule_delayed_work(&db->rx_work, 0); 
			break;		
  #endif
  
		case R_SCH_PHYPOLL:
#ifdef MORE_DM9051_MUTEX
			   schedule_delayed_work(&db->phypoll_work, 0);
#else
			   schedule_delayed_work(&db->rx_work, 0); 
#endif			   
			break;		
			
		#ifdef DM_CONF_INTERRUPT
		case R_SCH_XMIT:
		#ifdef MORE_DM9051_MUTEX
			schedule_delayed_work(&db->xmit_work, 0);
		#else
			schedule_delayed_work(&db->rx_work, 0); //dm9051_continue_poll
			   //[OR] schedule_delayed_work(&db->tx_work, 0); //(dm9051_tx_work) This which need tryLOck() or Mutex() design.
		#endif			   
		break;
		#endif			 	
		/* 0, Because @delay: number of jiffies to wait or 0 for immediate execution */
	}
	//spin_unlock(&db->statelock_tx_rx);//mutex_unlock(&db->addr_lock);
#endif	
	//spin_unlock(&db->statelock_tx_rx);//mutex_unlock(&db->addr_lock);
	#endif
}

 #ifdef DM_CONF_POLLALL_INTFLAG
 #elif DRV_POLL_1
 
#if !defined DM_EXTREME_CPU_MODE 
static void
dm9051_INTPschedule_weight(board_info_t *db, unsigned long delay)
{
	static int sd_weight = 0;
	
	#ifdef DM_CONF_TASKLET
	if (db->DERFER_rwregs[MD_ReadPTR] != db->DERFER_rwregs1[MD_ReadPTR]) {
		tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0); 
		return;
	}
	
	if (db->DERFER_rwregs1[RXM_WrtPTR] == db->DERFER_rwregs1[MD_ReadPTR]) {
		tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, delay); 
		return;
	}
	
	sd_weight++;
	if (!(sd_weight%3)) {
		if (sd_weight>=6000) /*(sd_weight>=5000) in disp no adj*/ 
			sd_weight = 0;
			
		if ( sd_weight == 0 && (db->DERFER_calc1>>8 )!= 0) // fewer disp
			printk("-[dm9 SaveCPU for: MDWA 0x%x (RO %d.%d%c)]-\n", db->DERFER_rwregs1[RXM_WrtPTR], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
		
		tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, delay);  // slower ,
		return;
	}
	tasklet_schedule(&db->rx_tl); //schedule_.delayed_work(&db->r, 0); 
	#else //~DM_CONF_TASKLET
	
	if (db->DERFER_rwregs[MD_ReadPTR] != db->DERFER_rwregs1[MD_ReadPTR]) {
		schedule_delayed_work(&db->rx_work, 0); 
		return;
	}
		
	//good.all.readout
	if (db->DERFER_rwregs1[RXM_WrtPTR] == db->DERFER_rwregs1[MD_ReadPTR]) { //THis is also 'db->DERFER_calc1>>8 == 0'
		//mdwa = 0;
		schedule_delayed_work(&db->rx_work, delay); 
		return;
	}
			
	sd_weight++;
	if (!(sd_weight%3)) /* "slower" by more delay_work(delay) */
	/*if (!(sd_weight%5)) */
	/*if (!(sd_weight%6)) */ {
		
		//warn.(NoWrPkt)But_Read_CutCut (too slow read or rx-pointer.Err)
		/*if ((db->DERFER_calc1>>8) > 5) {
			sd_weight = 0;
			dm9051_fifo_reset(1, "dm9 (RxPoint.Err)", db);
			dm9051_fifo_reset_statistic(db);
			schedule_..delayed_work(&db->rx_work, 1);  //or 'delay'
			return;
		}*/
	
		//normal
		if (sd_weight>=6000) /*(sd_weight>=5000) in disp no adj*/ 
			sd_weight = 0;
			
		if ( sd_weight == 0 && (db->DERFER_calc1>>8 )!= 0) // fewer disp
			printk("-[dm9 SaveCPU for: MDWA 0x%x (RO %d.%d%c)]-\n", db->DERFER_rwregs1[RXM_WrtPTR], db->DERFER_calc1>>8, db->DERFER_calc1&0xff, '%');
		
		schedule_delayed_work(&db->rx_work, delay);  // slower ,
		return;
	}
	schedule_delayed_work(&db->rx_work, 0); 
	#endif
}
#endif

#endif

//[int_dm9051_c]
/*
 *  INT 
 */

#ifdef DM_CONF_POLLALL_INTFLAG	
void int_en(struct net_device *dev)
{
    DM9051_int_en= 1;
    enable_irq(dev->irq); // V1.69.6p2 enable INT 
	//board_info_t *db = netdev_priv(dev);
    //if (db->nSCH_INT < 8)
	//	printk("[dm951_ int_en].e ------- enable_irq ( %d ) statis nSCH_INT= %d -------\n", dev->irq, db->nSCH_INT);
}
void int_dis(struct net_device *dev)
{
	disable_irq_nosync(dev->irq); // V1.69.6p2 disable INT 
	DM9051_int_en= 0;
}

#define S_DISP_BOOTING(endN)					((db->nSCH_INT < endN))
#define	S_DISP_DISABLE_IRQ(str)  if (S_DISP_BOOTING(DM_CONF_SPI_DBG_INT_NUM))  printk("[%s][-].disable_irq\n", str  /*, db->nSCH_INT*/ ); 
void IRQ_DISABLE(char *head, board_info_t *db)
{
//#ifdef DM_CONF_POLLALL_INTFLAG	
    if (DM9051_int_en){
		int_dis(db->ndev); //disable_irq_nosync(irq);DM9051._int_en= 0;
		
		//if ((db->nSCH_INT < DM_CONF_SPI_DBG_INT_NUM))  
			//printk("[%s][%d].disable_irq\n", head, db->nSCH_INT);
		S_DISP_DISABLE_IRQ(head)
			
		//in int_dis() DM9051_int_en= 0;
		//it like a structure with one field.DM9051_int_en is as "field.DM9051_int_enter_tobe_IMR_service_etc"
		//that every thing in the following (rx_work,dm9051_rx_work) will depend on "field.DM9051_int_enter_tobe_IMR_service_etc"
		//and finally clear it -- the "field.DM9051_int_enter_tobe_IMR_service_etc".
		//  int_reg_stop(db); .To DO FUNC.
		db->has_do_disable = true;
		//return true;
    }
//#endif
	else{
		db->has_do_disable = false;
		//return false;
	}
}

#define S_DISP_IRQ(staRANG, endN)	((db->nSCH_INT < staRANG) || (db->nSCH_INT == endN)) //[0~7, endN]
#define E_DISP_IRQ(staRANG, endN)	((db->nSCH_INT < (staRANG+1)) || (db->nSCH_INT == endN+1)) //[1~8, 25]
#define S_DISP_FIRST if (S_DISP_BOOTING(DM_CONF_SPI_DBG_INT_NUM)) {  \
	printk("\n"); \
	if (db->nSCH_INT==0) \
		printk("[dm9]\n"); \
}
#define S_DISP_FUNC	if (S_DISP_IRQ(DM9_DBG_INT_ONOFF_COUNT, DM_CONF_SPI_DBG_INT_NUM-1)) printk("[dm951_irq][-].in\n" /*, db->nSCH_INT*/ );  //(8,24) ,(5,10)
#define E_DISP_FUNC	if (E_DISP_IRQ(DM9_DBG_INT_ONOFF_COUNT ,DM_CONF_SPI_DBG_INT_NUM-1)) printk("[dm951_irq][%d].out\n", db->nSCH_INT); //(8,24) ,(5,10)
#define HAS_DISABLE	db->has_do_disable
//************************************************************//
/* Not used interrupt-related function: */
/* This function make it less efficient for performance */
/* Since the ISR not RX direct, but use a schedule-work */
/* So that polling is better in using its poll schedule-work */
#ifdef DM_CONF_THREAD_IRQ

static void dm9051_rx_work_proc(board_info_t *db);

static irqreturn_t dm9051_rx_work_irq(int irq, void *pw)
{
	board_info_t *db = pw;
	db->nSCH_INT++;
  	S_DISP_FIRST
  	S_DISP_FUNC
	IRQ_DISABLE("dm951_irq", db);
	if (HAS_DISABLE) db->nSCH_INT_NUm_A++;
	db->nSCH_INT_NUm++;
	
	dm9051_rx_work_proc(db); //proc
	
    E_DISP_FUNC
	return IRQ_HANDLED;
}
#else
static irqreturn_t dm951_irq(int irq, void *pw)
{
	board_info_t *db = pw;
	
	//if ((db->nSCH_INT < DM_CONF_SPI_DBG_INT_NUM))  // || (db->nSCH_INT == 24)
		//printk("\n");
	//if ((db->nSCH_INT < 8) || (db->nSCH_INT == 24)) //[0~7, 24]
	//{
		//if (db->nSCH_INT==0) {
			//printk("[dm9]\n");
		//}
		//printk("[dm951_irq][%d].in\n", db->nSCH_INT);
  	//}
  	S_DISP_FIRST
  	S_DISP_FUNC
  	
    //.DM9051_int_flag = 1;	

	//.#ifdef DM_CONF_POLLALL_INTFLAG	
    //.if (DM9051_int_en){
	//.	db->nSCH_INT_NUm_A++;
	//.	int_dis(db->ndev); //disable_irq_nosync(irq);DM9051._int_en= 0;
		//in int_dis() DM9051_int_en= 0;
		//it like a structure with one field.DM9051_int_en is as "field.DM9051_int_enter_tobe_IMR_service_etc"
		//that every thing in the following (rx_work,dm9051_rx_work) will depend on "field.DM9051_int_enter_tobe_IMR_service_etc"
		//and finally clear it -- the "field.DM9051_int_enter_tobe_IMR_service_etc".
		//  int_reg_stop(db); .To DO FUNC.
    //.}
	//.#endif
	//=
	//if (IRQ_DISABLE("dm951_irq", db))
		//db->nSCH_INT_NUm_A++;
	IRQ_DISABLE("dm951_irq", db);
	if (HAS_DISABLE) db->nSCH_INT_NUm_A++;
  
	db->nSCH_INT_NUm++;
    dm9051_INTPschedule_isr(db, R_SCH_INT); //schedule_work(&db->rx._work); //new 'dm9051_INTPschedule_isr'
    
    E_DISP_FUNC
	//if (db->nSCH_INT && ((db->nSCH_INT <= 8) || (db->nSCH_INT == 25))) //[1~8, 25]
		//printk("[dm951_irq][%d].out\n", db->nSCH_INT);
		
	return IRQ_HANDLED;
}
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
u8 read_DTS_IrqType(struct device_node *nc)
{
	u8 val;
	int rc;
	u32 value;
		val = IIRQ_TYPE_NONE;
		if (nc) {
			rc = of_property_read_u32_index(nc, "interrupts", 1, &value); // read to DTS index
			if (!rc) {
				printk("[ *dm9051 dts ] %s interrupts irq-type[= %d]\n", nc->full_name, value);
				val = (int) value;
			}
		}
		else
			printk("[ *dm9051 dts ] interrupts irq-type<= %d (default)>\n", val); //=value
			
		if (val) {
			if (val & IIRQ_TYPE_EDGE_RISING)
				printk("[ *dm9051 dts ] interrupts IRQ Trigger Type[= %d] (EDGE RISING)\n", val);
			if (val & IIRQ_TYPE_EDGE_FALLING)
				printk("[ *dm9051 dts ] interrupts IRQ Trigger Type[= %d] (EDGE FALLING)\n", val);
			if (val & IIRQ_TYPE_LEVEL_HIGH)
				printk("[ *dm9051 dys ] interrupts IRQ Trigger Type[= %d] (LEVEL HIGH)\n", val);
			if (val & IIRQ_TYPE_LEVEL_LOW)
				printk("[ *dm9051 dts ] interrupts IRQ Trigger Type[= %d] (LEVEL LOW)\n", val);
		}
		else
			printk("[ *dm9051 dts ] interrupts IRQ Trigger Type{= %d} (TRIGGER NONE)\n", val);
		return val;
}
void Set_IrqType(board_info_t *db, struct device_node *nc)
{
	db->irq_type = read_DTS_IrqType(nc);
}	
u8 Get_TrqType(board_info_t *db)
{
	return db->irq_type;
}
#endif

// return: 0, ATTRI_OK 
// return: 1, ATTRI_ERR 
int int_get_attribute(struct spi_device *spi, struct net_device *ndev)
{
	int ret = 0;
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	board_info_t *db = netdev_priv(ndev);
#else
	char *Str[5][2]= {
		{"IRQF_TRIGGER_NONE", "LEVEL_FLOAT"},
		{"IRQF_TRIGGER_RISING", "LEVEL_HIGH"},
		{"IRQF_TRIGGER_FALLING", "LEVEL_LOW"},
		{"IRQF_TRIGGER_HIGH", "LEVEL_HIGH"},  // domain-field
		{"IRQF_TRIGGER_LOW", "LEVEL_LOW"},  // domain-field
	};  //(DRV_IRQF_TRIGGER == IRQF_TRIGGER_LOW)? "(LEVEL LOW)": "(LEVEL HIGH)"
	int str_idx;
	u16 bbit= 0x01;
	str_idx= 0;
	while(bbit<=0x08) {
	 str_idx++;
	 if (DRV_IRQF_TRIGGER & bbit) {
		break;
	 }
	 bbit<<= 1;
   }
	if (bbit > 0x08) 
		str_idx= 0;
#endif	
	printk("dm9.[int_get_attri].s\n");
//#ifdef DM_CONF_POLLALL_INTFLAG
	//---------------------------------------------------------
	//[Raspberry Pi 2, GPIO_17 (pin 11) on Pi connection slot.]
	//[Raspberry Pi 2, GPIO_26 (pin 37) on Pi connection slot.]
	//---------------------------------------------------------
//#endif

//#ifdef DM_CONF_POLLALL_INTFLAG
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	//Jerry add. and. if. spi->irq = dm9051_get_irq_num();
	do {
		int rc;
		struct device_node *nc;
		u32 value;
		int gpio_int_pin; //Get gpio pin from DTS.
		/* DTS, this GPIO-INT-PIN setting is in the dts file, not in the driver ! */
		gpio_int_pin = 0; // 0 means null (empty) //DM_CONF_INTERRUPT_IRQ; //=value
		nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE);
		//read-interrupt-pin-num
		if (nc) {
			//rc = of_property_read_u32(nc, "interrupts", &value); // read to DTS 
			rc = of_property_read_u32_index(nc, "interrupts", 0, &value); // read to DTS index
			if (!rc) {
				printk("[ *dm9051 dts ] %s interrupts gpio-pin[= %d]\n", nc->full_name, value);
				gpio_int_pin = (int) value;
			}
		}
		else
			printk("[ *dm9051 dts ] interrupts gpio-pin= %d (read fail)>\n", gpio_int_pin); //=value
			
		printk("[ *dm9051 spi_device * ] %s value[= %d]\n", "spi->irq", spi->irq);
		
		//read-interrupt-trigger-irq-type	
		Set_IrqType(db, nc); //Or only call: read_DTS_IrqType(nc), is most pure simple.

		//dm9051 gpio_request.
		/* DTS, this request is of the dts's domination, not in the driver ! 	
		if (gpio_request(gpio_int_pin, GPIO_ANY_GPIO_DESC)) {
			printk("--------------------------------------------------------\n");
			printk("ERROR! dm9051 Mapped gpio_to_irq() IRQ no : %d\n", spi->irq);
			printk("--------------------------------------------------------\n");
			printk("dm951 gpio_request ERROR! pin %d, desc %s\n", DM_CONF_INTERRUPT_IRQ, GPIO_ANY_GPIO_DESC);
			printk("--------------------------------------------------------\n");
			goto err_irq;
		}*/ 
	} while(0);
#else
	printk("[ *dm9051 CONST ] dm9051 gpio_request: pin %d, desc %s\n", DM_CONF_INTERRUPT_IRQ, GPIO_ANY_GPIO_DESC);
	if (gpio_request(DM_CONF_INTERRUPT_IRQ, GPIO_ANY_GPIO_DESC)) {
		printk("---------------------------------------------\n");
		printk("Error! dm9051 Mapped gpio_to_irq() IRQ no : %d\n", spi->irq);
		printk("---------------------------------------------\n");
		printk("dm9051 gpio_request Error! pin %d, desc %s\n", DM_CONF_INTERRUPT_IRQ, GPIO_ANY_GPIO_DESC);
		printk("---------------------------------------------\n");
		ret = 1; 
		goto err_attri;
	}
	printk("[ *dm9051 CONST ] interrupts gpio-pin= %d\n", DM_CONF_INTERRUPT_IRQ);
	gpio_direction_input(DM_CONF_INTERRUPT_IRQ);
	spi->irq = gpio_to_irq(DM_CONF_INTERRUPT_IRQ);  //exp: enum gpio_to_irq( 17) = 187
	if (spi->irq <= 0) { // jj:enum
		printk("dm9051 failed to get irq_no, %d\n", spi->irq);
		ret = 1;
		goto err_attri;
	}
	printk("[ *dm9051 spi_device * ] gpio_to_irq()= %d\n", spi->irq);	
	
	printk("[ *dm9051 CONST ] interrupts IRQ Trigger Type[=%s] %s\n", 
		Str[str_idx][0],  Str[str_idx][1]); 
err_attri:		
#endif
//#endif
	printk("dm9.[int_get_attri].e\n");
	return ret;
}

void int_get_begin(struct spi_device *spi, struct net_device *ndev)   
{
	board_info_t *db = netdev_priv(ndev);
	int ret; // = 0;
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE	
	#ifdef DTS_AUTO_TRIGGER 
	#else
	unsigned long val_trigger;
	u8 val_type;
	#endif
#endif	
	
	printk("dm9.[int_get_begin].s\n");

//#ifdef DM_CONF_POLLALL_INTFLAG

	//"int_get_attribute()"

	ndev->irq = spi->irq;	
	//ndev->if_port = IF_PORT_100BASET;
				
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE	
	printk("dm9051 Mapped DTS's IRQ no : %d\n", spi->irq);
	
	//db->req_irq_flags = IRQF_TRIGGER_NONE;
	#ifdef DTS_AUTO_TRIGGER //(if 1)
	printk("dm9051 request irq for DTS's IRQ Trigger Type with IRQF_TRIGGER_NONE : %d (TRIGGER NONE)\n", IRQF_TRIGGER_NONE);
	printk("Hence registering with IRQF_TRIGGER_NONE does NOT modify the existing configuration of the IRQ and IRQ no.\n");
	#ifdef DM_CONF_THREAD_IRQ
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_work_irq,
				IRQF_TRIGGER_NONE, ndev->name, db);
	#else
	ret = request_irq(spi->irq, dm951_irq, IRQF_TRIGGER_NONE, /*db->req_irq_flags*/
				ndev->name, db);
	#endif
	#else			
	//if (Get_TrqType(db) & (IIRQ_TYPE_EDGE_RISING | IIRQ_TYPE_LEVEL_HIGH)) {
		//printk("dm9051 request irq for DTS's IRQ Trigger Type with IRQF_TRIGGER_HIGH : 0x%d (IRQF_TRIGGER_HIGH)\n", IRQF_TRIGGER_HIGH);
		//ret = request_irq(spi->irq, dm951_irq, IRQF_TRIGGER_HIGH,
		//		ndev->name, db);
	//} else {
		//printk("dm9051 request irq for DTS's IRQ Trigger Type with IRQF_TRIGGER_LOW : 0x%d (IRQF_TRIGGER_LOW)\n", IRQF_TRIGGER_LOW);
		//ret = request_irq(spi->irq, dm951_irq, IRQF_TRIGGER_LOW,
		//		ndev->name, db);
	//}
	val_type = Get_TrqType(db);
	if (val_type & IIRQ_TYPE_EDGE_RISING) {
		printk("dm9051 do request irq, IRQ Trigger Type[= %d] (EDGE RISING)\n", val_type);
		val_trigger = IRQF_TRIGGER_RISING;
	} else if (val_type & IIRQ_TYPE_EDGE_FALLING) {
		printk("dm9051 do request irq, IRQ Trigger Type[= %d] (EDGE FALLING)\n", val_type);
		val_trigger = IRQF_TRIGGER_FALLING;
	} else if (val_type & IIRQ_TYPE_LEVEL_HIGH){
		printk("dm9051 do request irq, IRQ Trigger Type[= %d] (LEVEL HIGH)\n", val_type);
		val_trigger = IRQF_TRIGGER_HIGH;
	} else if (val_type & IIRQ_TYPE_LEVEL_LOW) {
		printk("dm9051 do request irq, IRQ Trigger Type[= %d] (LEVEL LOW)\n", val_type);
		val_trigger = IRQF_TRIGGER_LOW;
	} else {
		printk("dm9051 do request irq, IRQ Trigger Type[= %d] Unknow to (LEVEL LOW)\n", val_type);
		val_trigger = IRQF_TRIGGER_LOW; // or 'IRQF_TRIGGER_NONE'
	}
	#ifdef DM_CONF_THREAD_IRQ
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_work_irq,
			val_trigger, ndev->name, db);
	#else
	ret = request_irq(spi->irq, dm951_irq, val_trigger,
			ndev->name, db);
	#endif
	#endif
#else		
	printk("(dm9051 Mapped gpio_to_irq() IRQ no : %d)\n", spi->irq);
	#ifdef DM_CONF_THREAD_IRQ
	ret = request_threaded_irq(spi->irq, NULL, dm9051_rx_work_irq,
				DRV_IRQF_TRIGGER | IRQF_ONESHOT,
				ndev->name, db);
	#else
	ret = request_irq(spi->irq, dm951_irq, DRV_IRQF_TRIGGER | IRQF_ONESHOT, //DRV_IRQF_TRIGGER for IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW
				ndev->name, db); //or IRQF_TRIGGER_NONE, //or IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING 
	#endif
#endif			

	disable_irq_nosync(/*spi->irq*/ndev->irq);//int_d.i.s
	DM9051_int_en= 0;//int_d.i.s
	printk("[DBG] dm9051 request irq then immediately, disable_irq_nosync()\n");
			
	if (!ret)
		printk("[DBG] dm9051 request irq (%d), ret= %d (must be zero to succeed)\n", spi->irq, ret);
	else
		printk("[DBG] dm9051 request irq (%d), ret= %d (must be zero to succeed, BUT not) ERROR!\n", spi->irq, ret);
	if (ret < 0) {
		printk("dm9051 failed to (get irq) ERROR!\n");
		goto err_irq;
	}
//#endif

	printk("%s: dm9051spi at isNO_IRQ %d MAC: %pM\n", // (%s)
		   ndev->name,
		   ndev->irq,
		   ndev->dev_addr);
		   
//#ifdef DM_CONF_POLLALL_INTFLAG
err_irq:
//#endif

	//#ifdef DM_CONF_POLLALL_INTFLAG
	//free_irq(spi->irq, db); 
	//#endif
	
	printk("dm9.[int_get_begin].e\n");
	
	return;
}

void int_end(struct spi_device *spi, board_info_t *db)    
{
  //#ifdef DM_CONF_POLLALL_INTFLAG
  
	#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	/* DTS, this free is of the dts's domination, not in the driver ! */ //Jerry add
	#else
	printk("[dm9051 gpio_free: pin %d (desc %s)]\n", DM_CONF_INTERRUPT_IRQ, GPIO_ANY_GPIO_DESC);
	gpio_free(DM_CONF_INTERRUPT_IRQ);
	#endif
	
  //#endif
  
  //#ifdef DM_CONF_POLLALL_INTFLAG
	printk("[dm9051 free_irq: irq no %d]\n", spi->irq);
	free_irq(spi->irq, db);
  //#endif
} 
#endif

//[sub_dm9051_c]
/* ops */
/* event: play a schedule starter in condition */
//static netdev_tx_t 
//DM9051_START_XMIT(struct sk_buff *skb, struct net_device *dev) //void sta_xmit_sched_delay_work(board_info_t * db)
//{
//	return NETDEV_TX_OK;
//}

//..ok //*************************************************************//
/*  Set DM9051 multicast address */
static void
dm_hash_table_unlocked(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
#ifdef JABBER_PACKET_SUPPORT
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN | RCR_DIS_WATCHDOG_TIMER;
#else	
	u8 rcr = RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
#endif	
#if DEF_SPIRW
	struct netdev_hw_addr *ha;
	int i, oft;
	u32 hash_val;
	u16 hash_table[4];
	for (i = 0, oft = DM9051_PAR; i < 6; i++, oft++)
		iiow(db, oft, dev->dev_addr[i]);

	/* Clear Hash Table */
	for (i = 0; i < 4; i++)
		hash_table[i] = 0x0;

	/* broadcast address */
	hash_table[3] = 0x8000;

	if (dev->flags & IFF_PROMISC)
		rcr |= RCR_PRMSC;

	if (dev->flags & IFF_ALLMULTI)
		rcr |= RCR_ALL;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, dev) {
		hash_val = ether_crc_le(6, ha->addr) & 0x3f;
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	}

	/* Write the hash table */
	for (i = 0, oft = DM9051_MAR; i < 4; i++) {
		iiow(db, oft++, hash_table[i]);
		iiow(db, oft++, hash_table[i] >> 8);
	}

	iow(db, DM9051_RCR, rcr);
#endif	
	db->rcr_all= rcr;
/*
//TEST
	db->rcr_all |= RCR_PRMSC | IFF_ALLMULTI;
	printk("Test db->rcr_all from %02x to %02x\n", rcr, db->rcr_all);
*/	
}
static void 
dm_hash_table(board_info_t *db)
{
	struct net_device *dev = db->ndev; //board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	dm_hash_table_unlocked(dev);	
    mutex_unlock(&db->addr_lock);
}

static void 
rx_mutex_hash_table(board_info_t *db)
{
	if (db->Enter_hash)	
	{
		dm_hash_table(db);
		db->Enter_hash= 0;
	}
}

	#ifdef DM_CONF_TASKLET
	static void 
	dm_hash_table_task(unsigned long data)
	{
		board_info_t *db = (board_info_t *) data;
		db->Enter_hash= 1;
	}
	#else //~DM_CONF_TASKLET
	static void 
	dm_hash_table_work(struct work_struct *work)
	{
		board_info_t *db = container_of(work, board_info_t, rxctrl_work);
		db->Enter_hash= 1;
		//board_info_t *db = container_of(work, board_info_t, rxctrl_work);
		//dm_hash_table(db); //struct net_device *dev = db->ndev;
	}
	#endif

void sched_work(board_info_t *db)
{
	#ifdef DM_CONF_TASKLET
	tasklet_schedule(&db->rxctrl_tl);
	#else //~DM_CONF_TASKLET
	#if 1
	/*[DM9051_Schedule.c]*/
	/* spin_lock/spin_unlock(&db->statelock); no need */
	schedule_work(&db->rxctrl_work);
	#endif
	#endif
}

//..ok //*************************************************************//
#ifdef DM_CONF_PHYPOLL
#if 0
void dm_netdevice_carrier(board_info_t *db)
{
	struct net_device *dev = db->ndev;
	u8 nsr;
	int link;	
	mutex_lock(&db->addr_lock);
	nsr= iior(db, DM9051_NSR); 
	mutex_unlock(&db->addr_lock);	
	link= !!(nsr & NSR_LINKST);
	db->link= link;       //JJ-Add, Rasp-save
	if (netif_carrier_ok(dev) != link) {
		if (link)
		  netif_carrier_on(dev);
		else
		  netif_carrier_off(dev);
		printk("[DM9051.continue_poll] Link Status is: %d\n", link);
	}
}
#endif

int db_phy = 0;
int nAll_run_gap = 0;

void dm_schedule_phy(board_info_t *db)
{  
	#ifdef DM_CONF_TASKLET
	tasklet_schedule(&db->phy_poll_tl);
	#else //~DM_CONF_TASKLET
  //schedule_delayed_work(&db->phy._poll, HZ * 2); to be 3 seconds instead
  //schedule_delayed_work(&db->phy._poll, HZ * 3);
	schedule_delayed_work(&db->phy_poll, HZ * 2);
	#endif
}

	#ifdef DM_CONF_TASKLET
	static void 
	dm_phy_poll_task(unsigned long data)
	{
		board_info_t *db = (board_info_t *) data;
		int a, b;
		
#ifdef DM_CONF_POLLALL_INTFLAG 
  #if defined MORE_DM9051_MUTEX && defined  MORE_DM9051_MUTEX_EXT
	mutex_lock(&db->spi_lock);
	if (!DM9051_int_en) {
		mutex_unlock(&db->spi_lock);
		goto sched_phy;
	}
	mutex_unlock(&db->spi_lock);
  #else
	if (!DM9051_int_en)
		goto sched_phy;
  #endif
#else		
	//if (!DM9051_int_en_OF_poll) goto sched_phy;
#endif	

	//debug.NOT.in_rx_work.s!
	a = (int) db->nSCH_INT_NUm;
	b = (int) db->nSCH_INT_B;
	if (a != (b + nAll_run_gap)) { 
		nAll_run_gap = a - b; // record the diff.
	}
	db_phy++; 
	//debug.NOT.in_rx_work.e!
	
	dm9051_INTPschedule_isr(db, R_SCH_PHYPOLL);  //extended-add
	
#ifdef DM_CONF_POLLALL_INTFLAG 
sched_phy:
#else
//sched_phy:
#endif
	if (netif_running(db->ndev))
	  dm_schedule_phy(db);
	}
	#else //~DM_CONF_TASKLET
	
static void 
dm_phy_poll(struct work_struct *w)
{ 
//#ifdef DM_CONF_PHYPOLL
	struct delayed_work *dw = to_delayed_work(w);
	board_info_t *db = container_of(dw, board_info_t, phy_poll);
	int a, b;
	
	//if.in.rx_work.procedure.s!
#ifdef DM_CONF_POLLALL_INTFLAG 
  #if defined MORE_DM9051_MUTEX && defined  MORE_DM9051_MUTEX_EXT
	mutex_lock(&db->spi_lock);
	if (!DM9051_int_en) {
		mutex_unlock(&db->spi_lock);
		goto sched_phy;
	}
	mutex_unlock(&db->spi_lock);
  #else
	if (!DM9051_int_en)
		goto sched_phy;
  #endif
#else		
	//if (!DM9051_int_en_OF_poll) goto sched_phy;
#endif	
	//if.in.rx_work.procedure.e!
	
	//debug.NOT.in_rx_work.s!
	a = (int) db->nSCH_INT_NUm;
	b = (int) db->nSCH_INT_B;
	if (a != (b + nAll_run_gap)) { 
		nAll_run_gap = a - b; // record the diff.
		//.printk("dm_phypol %d[run-gap %d][PHY-SCHED-rx-work-OUT_OF-INT].CHK. INT.Num %5d(dis %5d), INT.Sch= %5d(en %d).\n",
		//.	db_phy, nAll_run_gap, db->nSCH_INT_NUm, db->nSCH_INT_NUm_A, db->nSCH_INT, db->nSCH_INT_B);
	}
	db_phy++; 
	//debug.NOT.in_rx_work.e!
	
	//dm_netdevice_carrier(db);
	dm9051_INTPschedule_isr(db, R_SCH_PHYPOLL);  //extended-add
	
#ifdef DM_CONF_POLLALL_INTFLAG 
sched_phy:
#else
//sched_phy:
#endif
	if (netif_running(db->ndev))
	  dm_schedule_phy(db);
//#endif
}
  #endif
#endif

//..ok //*************************************************************//
#if DM9051_CONF_TX
static void dm9051_tx_chk(struct net_device *dev, u8 *wrptr)
{
#if 0
    printk("dm9.tx_packets %lu ", dev->stats.tx_packets);
    printk("tx(%02x %02x %02x %02x %02x %02x ", wrptr[0], wrptr[1],wrptr[2],wrptr[3],wrptr[4],wrptr[5]);
    printk("%02x %02x   %02x %02x %02x %02x ", wrptr[6], wrptr[7],wrptr[8],wrptr[9],wrptr[10],wrptr[11]);
    printk("%02x %02x\n", wrptr[12],wrptr[13]);
#endif
}
#endif

#if DM9051_CONF_TX
static int
dm9051_continue_xmit_inRX(board_info_t *db) //dm9051_continue_poll_xmit
{
		    struct net_device *dev = db->ndev;
		    int nTx= 0;

		    db->bt.local_cntTXREQ= 0;
		    db->bt.local_cntLOOP= 0;
			while(!skb_queue_empty(&db->txq)) // JJ-20140225, When '!empty'
			{
				  struct sk_buff *tx_skb;
				  int nWait = 0;
				  tx_skb = skb_dequeue(&db->txq);
				  if (tx_skb != NULL) {
					  	
#if DEF_SPIRW
					        while( (ior(db, DM9051_TCR) & TCR_TXREQ) && (++nWait < 20)) 
								;
#endif
					        if (nWait ==20)
								printk("[dm9] tx_step timeout B\n");
					        
					        //while( ior(db, DM9051_TCR) & TCR_TXREQ ) 
							//	; //driver_dtxt_step(db, 'B');
				    
					        if(db->bt.local_cntTXREQ==2)
					        {
					           //while( ior(db, DM9051_TCR) & TCR_TXREQ ) 
					            // ; //driver_dtxt_step(db, 'Z');
					           db->bt.local_cntTXREQ= 0;
					        }

						    nTx++;

#if DEF_SPIRW
							#if 0	
							if (1)
 								{
 									int i;
 									char *pb = (char *) tx_skb->data;
 									for (i=0; i<tx_skb->len; i++ )
 										*pb++ = i;
 								}
							#endif	
						    dm9051_outblk(db, tx_skb->data, tx_skb->len);
						    iow(db, DM9051_TXPLL, tx_skb->len);
						    iow(db, DM9051_TXPLH, tx_skb->len >> 8);
#ifdef JABBER_PACKET_SUPPORT						    
						    iow(db, DM9051_TCR, TCR_TXREQ | TCR_DIS_JABBER_TIMER);
#else
						    iow(db, DM9051_TCR, TCR_TXREQ);
#endif
#endif
						    dev->stats.tx_bytes += tx_skb->len;
						    dev->stats.tx_packets++;
						    /* done tx */
					        #if 1
						    dm9051_tx_chk(dev, tx_skb->data);
					        #endif
						    dev_kfree_skb(tx_skb);
				            db->bt.local_cntTXREQ++;
				            db->bt.local_cntLOOP++;
							#if 0
							  {   
					            u16 mdwr;
					            u16 txmr;
						        while (ior(db, DM9051_TCR) & TCR_TXREQ) 
									;
								
								mdwr= ior(db, 0x7a);
								mdwr |= (u16)ior(db, 0x7b) << 8;
								txmr= ior(db, 0x22);
								txmr |= (u16)ior(db, 0x23) << 8;
								printk("TX.e [txmr %03x mdwr %03x]\n", txmr, mdwr);
							}
							#endif 
				  } //if
			} //while

			#if 0 //checked ok!!
			if (db->nSCH_INT_NUm >= db->nSCH_INT_Num_Disp) { //( /*nTx>1 || */ !(db->nSCH_INT_NUm%50))
			  char *jmp_mark= " ";
			  u16 update_num_calc = ((db->nSCH_INT_NUm/100)*100) + 100;
			  db->nSCH_INT_Num_Disp += 100;
			  if (db->nSCH_INT_Num_Disp != update_num_calc) { //i.e. (db->nSCH_INT_Num_Disp < update_num_calc)
				  jmp_mark= "*";
				  db->nSCH_INT_Num_Disp = update_num_calc;
			  }
			}
			#endif
			
		    return nTx;
}
#endif

static int dm9051_tx_irx(board_info_t *db)
{
	struct net_device *dev = db->ndev;
	int nTx = 0;
#if DM9051_CONF_TX

	if (check_cntStop(db)) 
	  //if (db->bt.prob_cntStopped)  
	  // This is more exactly right!!
	{
		  #if LOOP_XMIT
		    //mutex_lock(&db->addr_lock);
		    nTx= dm9051_continue_xmit_inRX(db); //=dm9051_continue_poll_xmit
		    opening_wake_queue1(dev); 
		    //mutex_unlock(&db->addr_lock);
		  #endif //LOOP_XMIT
	}
	
#endif //DM9051_CONF_TX
	return nTx;
}

//..ok //*************************************************************//
static int dm9051_sch_cnt_chang(u16 nEnter) // large equ +1 (as increasment)
{
	static u16 nSAVE= 0xffff;
	if (nEnter != nSAVE) {
		nSAVE= nEnter;
		return 1;
	}
	return 0;
}
int rx_work_carrier(board_info_t *db)
{
	struct net_device *dev = db->ndev;
	unsigned nsr;
	int link;	
	static int try= 0;
	//ststic int ng_found = 0
	
	//if (1) {
	//[here!]
	//do {
	mutex_lock(&db->addr_lock);
#if DEF_SPIRW
		nsr= iior(db, DM9051_NSR);
#endif
		link= !!(nsr & 0x40); //& NSR_LINKST
			
		if (!link && try && !(try%250) && try<=750)
		  printk("[DM9051.carrier] nsr %02x, link= %d (try %d)\n", nsr, link, try);
		
		if (link) {
			if (db->linkA<3) 
			  db->linkA++;
		} else {
			if (db->linkA)
			  db->linkA--;
		}

		//db->linkBool= db->linkA ? 1 : 0;  //Rasp-save
		if (db->linkA) {
			db->linkBool= 1;
			try= 0; //ng_found= 0;
		} else {
			db->linkBool= 0;
			try++; //ng_found= 1;
		}

	if (db->linkBool) //(netif_carrier_ok(dev))
	{
		if (dm9051_sch_cnt_chang(db->nSCH_LINK))
		  printk("\n[DM9051.carrier] Link Status is: %d nsr %02x [SCHEDU cnt %d. try %d]\n", link, nsr, db->nSCH_LINK, try);
	}
	else
	{
		db->nSCH_LINK++;
		if (db->nSCH_LINK<3)
		  printk("[DM9051.carrier] Link Status is: %d\n", link); //"nsr %02x", nsr
	}
	
		
		if (netif_carrier_ok(dev) != db->linkBool) { //Add	
			if (db->linkBool) {
			  netif_carrier_on(dev); //db->nSCH_LINK= 0;
			} else
			  netif_carrier_off(dev);
			printk("[DM9051.phypoll] Link Status is: %d\n", link);
		}
		
	mutex_unlock(&db->addr_lock);  
	return link;
	//} while ((++try < 8) && !db->linkBool);
	//}
}

void rx_work_cyclekeep(board_info_t *db, int has_txrx) // link_sched_delay_work, INT_glue_sched_delay_work, and infini_sched_delay_work
{
	//struct net_device *dev = db->ndev;
    //if (!netif_carrier_ok(dev) && db->nSCH_LINK < 65500) 	//new-add
	//	dm9051_INTPschedule_isr(db, R_SCH_LINK);         	//new-add
  #ifdef DM_CONF_POLLALL_INTFLAG
	static u32 SSave_Num = 0;
	static u32 SSave_INT_B = 0;
	char *jmp_mark= "*";
  #endif
   
  #ifdef DM_CONF_POLLALL_INTFLAG
	if (DM9051_int_token) DM9051_int_token--;
	if (DM9051_int_token)
		dm9051_INTPschedule_isr(db, R_SCH_INT_GLUE); //again (opt-0)
		
	if (has_txrx)	
		dm9051_INTPschedule_isr(db, R_SCH_INFINI); //again (opt-0)
		
	if (db->nSCH_INT_NUm != db->nSCH_INT_B) {
		if ((SSave_Num != db->nSCH_INT_NUm) || (SSave_INT_B != db->nSCH_INT_B)) {
			#if 0
			
			//.Check ok.
			//.printk("[DM9_cyclekeep Check] INT.Num %5d(dis %5d), INT.Sch= %5d(en %d)%s\n",
			//.	db->nSCH_INT_NUm, db->nSCH_INT_NUm_A, db->nSCH_INT, db->nSCH_INT_B, jmp_mark);
				
			#endif	
			SSave_Num = db->nSCH_INT_NUm;
			SSave_INT_B= db->nSCH_INT_B;
			
			if (db->nSCH_INT_NUm > (db->nSCH_INT_B+10)) {
				jmp_mark = "**";
				db->nSCH_INT_NUm_A= db->nSCH_INT= db->nSCH_INT_B= db->nSCH_INT_NUm;
				printk("[DM9_cyclekeep ALL-SYNC-EQUAL] INT.Num %5d(dis %5d), INT.Sch= %5d(en %d)%s\n",
				  db->nSCH_INT_NUm, db->nSCH_INT_NUm_A, db->nSCH_INT, db->nSCH_INT_B, jmp_mark);
			}
		}
	}
		
  #elif DRV_POLL_1
  
	//dm9051_INTPschedule_isr(db, R_SCH_INFINI);
	//=
	// schedule_delayed_work(&db->rx_work, 0); //dm9051_rx_work

#ifdef DM_EXTREME_CPU_MODE //20210204	
  //(4.14.79-KT.POLL-2.2zcd.xTsklet.savecpu_5pt_JabberP.202002_nosave_20210204)
  //(lnx_dm9051_dts_Ver2.2zcd_R2_b2_savecpu5i2p_Tasklet5p_JabberP_pm_NEW2.0_extreme)
  schedule_delayed_work(&db->rx_work, 0);
#else //20210204	  
	#define DM_TIMER_EXPIRE1    1  //15
	#define DM_TIMER_EXPIRE2    0  //10
	#define DM_TIMER_EXPIRE3    0
	
	if (db->DERFER_rwregs[RXM_WrtPTR] == db->DERFER_rwregs1[RXM_WrtPTR])
		dm9051_INTPschedule_weight(db, DM_TIMER_EXPIRE1);
	else {
		//if ((db->DERFER_calc1>>8) < 50)
		//	schedule_delayed_work(&db->rx_work, DM_TIMER_EXPIRE2); // slow ,
		//else
	#ifdef DM_CONF_TASKLET
			tasklet_schedule(&db->rx_tl);
	#else //~DM_CONF_TASKLET
			schedule_delayed_work(&db->rx_work,  DM_TIMER_EXPIRE3); // faster ,
	#endif
	}
#endif //20210204	
	
  #endif
}

void IMR_DISABLE(board_info_t *db)
{
#ifdef DM_CONF_POLLALL_INTFLAG	
	if (!DM9051_int_en) { // Note.ok. 
		
	 //if (db->sch_cause!=R_SCH_INT) {
	//	printk("[Dbg condition: CASE-IS-IMPOSSIBLE] check (db->sch_cause!=R_SCH_INT) INTO rx-work~]\n");
	//	printk("[Dbg condition: CASE-IS-IMPOSSIBLE] list ([SCH_INIT,1][XMIT,2][INT,3][INFINI,4][GLUE,5][PHYPOLL,6]) db->sch_cause= %d\n", db->sch_cause);
	 //}
		
	 //if (db->sch_cause==R_SCH_INT) {
	 mutex_lock(&db->addr_lock);
	 int_reg_stop(db);
	 mutex_unlock(&db->addr_lock);  
	 //}
	}
#endif
}

bool ISR_RE_STORE(board_info_t *db)
{
#ifdef DM_CONF_POLLALL_INTFLAG	
	static unsigned short ctrl_rduce = 0;
	if (!DM9051_int_en)  // Note that: Come-in with 'if (db->sch_cause==R_SCH_INT)' TRUE.
	{

		#if defined WR_ISR_ENDOF_RXWORK_ONLY //to-do-check-how-to...
		mutex_lock(&db->addr_lock);
		db->bC.isbyte= ior(db, DM9051_ISR); // Got ISR
		if (db->bC.isbyte & 0x7e) 
		{

			//if (db->bC.isbyte == 0x82) ; // [only 'PT']

			if (db->bC.isbyte & 0x03) //(db->bC.isbyte & 0x01)
				ctrl_rduce++; // [with 'PT' or 'PR']
			else // somethings, BUT without PT or PR
				printk("[isr_reg] ISR= 0x%02x (somethings, BUT without PT or PR) Warn-Rare: overflow suspected\n", db->bC.isbyte);
				
			iiow(db, DM9051_ISR, db->bC.isbyte); // Clear ISR status
		}
		else 
		{
			if (db->bC.isbyte & 0x01)
				iiow(db, DM9051_ISR, db->bC.isbyte); // Clear ISR status //printk("[int_reg].e WITH PR: Wr ISR= 0x%02x\n", db->bC.isbyte);
		}
		mutex_unlock(&db->addr_lock);  
		#endif

		return true;
	}
#endif
	return false;
}

void IMR_ENABLE(board_info_t *db, int with_enable_irq)
{
	mutex_lock(&db->addr_lock);	
#ifdef DM_CONF_POLLALL_INTFLAG	
	if (!DM9051_int_en) { // Note that: Come-in with 'if (db->sch_cause==R_SCH_INT)' TRUE.
	 
      if (with_enable_irq)  {
		  
		if ((db->nSCH_INT <= DM_CONF_SPI_DBG_INT_NUM))  // || (db->nSCH_INT == 24)
			printk("[%s][%d].enable_irq\n", "dm951_irq", db->nSCH_INT); //from-"dm9051_rx_work"
		int_en(db->ndev);
      }
		
	  int_reg_start(db, "[dm9IMR]"); // "dm9IMR_irx_work", rxp no chg, if ncr-rst then rxp 0xc00 
	}
	if (DM9051_fifo_reset_flg) {
#if DEF_SPIRW
	  iiow(db, DM9051_RCR, db->rcr_all); // if ncr-rst then rx enable
#endif
	  DM9051_fifo_reset_flg = 0;
	}
#else	
	if (DM9051_fifo_reset_flg) {
	  int_reg_start(db, "[dmIMR_poll_rx_work]"); // exactly ncr-rst then rxp to 0xc00
#if DEF_SPIRW
	  iiow(db, DM9051_RCR, db->rcr_all); // exactly ncr-rst then rx enable
#endif
	  DM9051_fifo_reset_flg = 0;
	}
#endif
    mutex_unlock(&db->addr_lock);
}

//..ok //*************************************************************//

int rx_tx_isr0(board_info_t *db)
{
		struct net_device *dev = db->ndev;
		int nTX= 0, nRx= 0, n_tx_rx= 0;
		do {
			
			#ifdef DM_CONF_POLLALL_INTFLAG
			if (DM9051_int_en)  {
				nTX= dm9051_tx_irx(db);
				n_tx_rx += nTX;
			}
			else
			#else
			   //if (DM9051_int_en_OF_poll) {
			   // ...
			  //}
			  //else
			#endif
			{
				do {
				  nTX= dm9051_tx_irx(db);
				  nRx= dm9051_isr_ext2(dev); //dm9051_continue_poll_rx(db);
				  n_tx_rx += nTX;
				  n_tx_rx += nRx;
				} while(nRx);
			}
		} while (nTX || nRx);	
		return n_tx_rx;
}

static void dm9051_mutex_dm9051(board_info_t *db)
{
	//int link; link= 
	//printk("[dm9051.isr extend.s:\n");
	int	has_tx_rx= 0;
	static int dbg_first_in = 1;
	
	IMR_DISABLE(db);
	
	if (dbg_first_in) {
	  dbg_first_in = 0;
	  printk("[dm9051_rx_work] ------- 03.s.first in. ------\n");
	  //rx_mutex_head(db);
	  //dm9051_rx_cap(db); // get db->rwregs[0] & db->rwregs[1]
	  //rx_mutex_tail(db);
	  printk("[dm9051_rx_work] ------- 03.s. %x/%x. ------\n", db->rwregs[0], db->rwregs[1]);
	}

	rx_mutex_head(db);
	dm9051_disp_hdr_s_new(db);
	rx_mutex_tail(db);
	
	/* [dm9051_simple_mutex_dm9051].s.e */
	rx_work_carrier(db);
  #if 1	
	if (netif_carrier_ok(db->ndev)) {
  #endif
	do {
      rx_mutex_hash_table(db);
    
      //rx_tx_isr(db);=
      rx_mutex_head(db);
      has_tx_rx = rx_tx_isr0(db); // e.g. has_tx_rx = 0;
      rx_mutex_tail(db);
    
	} while(0);
  #if 1	  
	}
  #endif
  
	if (ISR_RE_STORE(db)) //if (IMR._ENABLE(db, 1))
		db->nSCH_INT_B++;

	rx_mutex_head(db);
    dm9051_disp_hdr_e_new(db);
	rx_mutex_tail(db);
    
	rx_work_cyclekeep(db, has_tx_rx); //[CYCLE-KEEP]
	IMR_ENABLE(db, 1);
}

static void dm9051_simple_mutex_dm9051(board_info_t *db)
{
	rx_work_carrier(db);
  #if 1	
	if (netif_carrier_ok(db->ndev)) {
  #endif
	do {
      rx_mutex_hash_table(db);
    
      //rx_tx_isr(db);=
      rx_mutex_head(db);
      /*has_tx_rx= */ rx_tx_isr0(db); // has_tx_rx = NOUSED.
      rx_mutex_tail(db);
    
	} while(0);
  #if 1	  
	}
  #endif
  
  #ifdef DM_CONF_POLLALL_INTFLAG 
  //[ASR gpio only (high trigger) raising trigger].s
  #ifdef MORE_DM9051_INT_BACK_TO_STANDBY 
   //#ifdef DM_CONF_POLLALL_INTFLAG 
	if (DM9051_int_en )
	{
  //#endif
	     mutex_lock(&db->addr_lock);	
         db->bC.isbyte= ior(db, DM9051_ISR); // Got ISR
      
		if (db->bC.isbyte & 0x01)
		{
		  iiow(db, DM9051_ISR, db->bC.isbyte); //~bhdbd~~ // Clear ISR status
		  //;printk("--- dm9 check DM9051_INT_BACK_TO_STANDBY [%d]--- \n", db->nSCH_INT);
		}      
        mutex_unlock(&db->addr_lock);  
		
   //#ifdef DM_CONF_POLLALL_INTFLAG 
	}
  //#endif
 #endif	
  //[ASR gpio only (high trigger) raising trigger].e
 #endif
}

	#ifdef DM_CONF_TASKLET
	static void dm9051_rx_task(unsigned long data) {
		board_info_t *db = (board_info_t *) data;
		#ifdef MORE_DM9051_MUTEX
		mutex_lock(&db->spi_lock);
		#endif

		dm9051_mutex_dm9051(db);
		
		#ifdef MORE_DM9051_MUTEX
		mutex_unlock(&db->spi_lock);
		#endif
	}
	#else //~DM_CONF_TASKLET
	#ifdef DM_CONF_THREAD_IRQ
	static void dm9051_rx_work_proc(board_info_t *db) {
		#ifdef MORE_DM9051_MUTEX
		mutex_lock(&db->spi_lock);
		#endif

		dm9051_mutex_dm9051(db);
		
		#ifdef MORE_DM9051_MUTEX
		mutex_unlock(&db->spi_lock);
		#endif
	}
	#else
  static void dm9051_rx_work(struct work_struct *work) { //TODO. (over-night ? result)
	struct delayed_work *dw = to_delayed_work(work);
	board_info_t *db = container_of(dw, board_info_t, rx_work);
	
#ifdef MORE_DM9051_MUTEX
	mutex_lock(&db->spi_lock);
#endif

	dm9051_mutex_dm9051(db);
	
#ifdef MORE_DM9051_MUTEX
	mutex_unlock(&db->spi_lock);
#endif
  }
  #endif
  #endif

#ifdef MORE_DM9051_MUTEX
	#ifdef DM_CONF_TASKLET
	static void dm9051_phypoll_tasklet(unsigned long data){
		board_info_t *db = (board_info_t *) data;
		mutex_lock(&db->spi_lock);
		dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
		mutex_unlock(&db->spi_lock);
	}
	static void dm9051_xmit_tasklet(unsigned long data){
		//[or by spin_lock_irq(&db->hwlock)/spin_unlock_irq(&db->hwlock)]
		board_info_t *db = (board_info_t *) data;
		mutex_lock(&db->spi_lock);
		dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
		mutex_unlock(&db->spi_lock);
	}
	#else //~DM_CONF_TASKLET
static void dm9051_phypoll_work(struct work_struct *work) {
	struct delayed_work *dw = to_delayed_work(work);
	board_info_t *db = container_of(dw, board_info_t, phypoll_work);
	mutex_lock(&db->spi_lock);
	dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
	mutex_unlock(&db->spi_lock);
}
static void dm9051_xmit_work(struct work_struct *work) {
	struct delayed_work *dw = to_delayed_work(work);
	board_info_t *db = container_of(dw, board_info_t, xmit_work);
	mutex_lock(&db->spi_lock);
	dm9051_simple_mutex_dm9051(db); //dm9051_mutex_dm9051(db);
	mutex_unlock(&db->spi_lock);
}
		#endif
#endif

/*[DM9051_Device_Ops.c]*/
void dm_sched_start_rx(board_info_t *db) // ==> OPEN_init_sched_delay_work
{
	if (db->driver_state!=DS_POLL) {
	   db->driver_state= DS_POLL;
	   dm9051_INTPschedule_isr(db, R_SCH_INIT); //#ifndef DM_CONF_POLLALL_INTFLAG, #endif
	}
}

//..ok //*************************************************************//

void define_delay_work(board_info_t * db)
{
	#ifdef DM_CONF_TASKLET
	tasklet_init(&db->rxctrl_tl, dm_hash_table_task,(unsigned long) db);
#ifdef DM_CONF_PHYPOLL	
	tasklet_init(&db->phy_poll_tl, dm_phy_poll_task,(unsigned long) db);
#endif
	tasklet_init(&db->rx_tl, dm9051_rx_task, (unsigned long) db);

#ifdef MORE_DM9051_MUTEX
	tasklet_init(&db->phypoll_tl, dm9051_phypoll_tasklet, (unsigned long) db);
	tasklet_init(&db->xmit_tl, dm9051_xmit_tasklet, (unsigned long) db);
#endif
	#else //~DM_CONF_TASKLET
	
	INIT_WORK(&db->rxctrl_work, dm_hash_table_work);
#ifdef DM_CONF_PHYPOLL	
	INIT_DELAYED_WORK(&db->phy_poll, dm_phy_poll);
#endif
    
    #ifndef DM_CONF_THREAD_IRQ
    INIT_DELAYED_WORK(&db->rx_work, dm9051_rx_work); //(dm9051_continue_poll); // old. 'dm9051_INTP_isr()' by "INIT_WORK"
    #endif
    
#ifdef MORE_DM9051_MUTEX
    INIT_DELAYED_WORK(&db->phypoll_work, dm9051_phypoll_work);
    INIT_DELAYED_WORK(&db->xmit_work, dm9051_xmit_work);
#endif
  #endif
}

//void sched_delay_work(board_info_t * db)  
//{  --for OPEN_init_sched_delay_work
//   --for hash(= sched_work)
//   --for sta_xmit_sched_delay_work
//}  --for link_sched_delay_work(= dm_schedule._phy), 
//  --INT_glue_sched_delay_work, and 
//  --infini_sched_delay_work

void sched_delay_work_cancel(board_info_t * db)
{
	#ifdef DM_CONF_TASKLET
	#ifdef DM_CONF_PHYPOLL
	tasklet_kill(&db->phy_poll_tl);
	#endif
	tasklet_kill(&db->rxctrl_tl);
	tasklet_kill(&db->rx_tl);
	#ifdef MORE_DM9051_MUTEX
	tasklet_kill(&db->phypoll_tl);
	tasklet_kill(&db->xmit_tl);
	#endif
	#else //~DM_CONF_TASKLET
	
	#ifdef DM_CONF_PHYPOLL
	cancel_delayed_work_sync(&db->phy_poll);
	#endif
	
//.flush_work(&db->rxctrl_work); /* stop any outstanding work */
  #ifndef DM_CONF_THREAD_IRQ
	cancel_delayed_work_sync(&db->rx_work); //flush_work(&db->rx_work);
	#endif
	
#ifdef MORE_DM9051_MUTEX
	cancel_delayed_work_sync(&db->phypoll_work);
	cancel_delayed_work_sync(&db->xmit_work);
#endif
  #endif
}

//void scded_work_flush(board_info_t * db)
//{
//}

//void temp_suspend_delay_work(board_info_t * db){}
//void temp_resume_delay_work(board_info_t * db){}

#if defined MTK_CONF_XY6762TEMP /* || defined QCOM_CONF_BOARD_YES */

#define ETH_NODE_NAME "davicom,dm9051"

struct pin_ctl{
	unsigned int gpio_power;
	//unsigned int gpio_SPI_CS;
	unsigned int gpio_rst;
unsigned int gpio_rst_rtl;		
};

static struct pin_ctl *dts_pin;

static int ctl_pwr_gpio(void) {  //xx_gpio(int power, int power_en ,int cs, int cs_en)
	int ret = 0;
	//if(power == POWER){
		if(/*power_en &&*/ dts_pin->gpio_power != 0){
			ret = gpio_request(dts_pin->gpio_power,"power");
			if (ret) {
				printk("error: dm9051 gpio_request\n");
				return -1;
			}
			ret = gpio_direction_output(dts_pin->gpio_power, 1);
			if (ret) {
				printk("error %s : dm9051 reset gpio_direction_output failed\n",__FILE__);
				ret = -1;
			}
			mdelay(5);
		}//else if((!power_en) && dts_pin->gpio_power != 0){
			//ret = gpio_request(dts_pin->gpio_power,"power");
			//if (ret) {
			//	printk("error: dm9051 gpio_request\n");
			//	return -1;
			//}
			//ret = gpio_direction_output(dts_pin->gpio_power, 0);
			//if (ret) {
			//	printk("error %s : dm9051 reset gpio_direction_output failed\n",__FILE__);
			//	ret = -1;
			//}
			//mdelay(5);
		//}
	//}

	return 0;
} 

static int ctl_rst_gpio(void)
{
	int ret;
	if(dts_pin->gpio_rst != 0){
	  ret = gpio_request(dts_pin->gpio_rst,"spi_rst");
	  ret = gpio_direction_output(dts_pin->gpio_rst, 0);
	  msleep(100);
	  ret = gpio_direction_output(dts_pin->gpio_rst, 1);
	  msleep(10);		
	}
	  msleep(100);
	if(dts_pin->gpio_rst_rtl != 0){
	  ret = gpio_request(dts_pin->gpio_rst_rtl,"spi_rst_rtl");
	  ret = gpio_direction_output(dts_pin->gpio_rst_rtl, 0);
	  msleep(100);
	  ret = gpio_direction_output(dts_pin->gpio_rst_rtl, 1);
	  msleep(10);		
	}
	return ret;
}

int SPI_GPIO_Set(int enable)
{
	 struct device_node *node;
		printk("SPI_GPIO_Set\n");
	 node = of_find_compatible_node(NULL, NULL, ETH_NODE_NAME);
	if (!node) {
		printk("[ *dm9051 READ-quacom dts WARN ] {%s} is not found in the dts data-grouping-set\n", ETH_NODE_NAME);
		return -1;
	}
	 
     dts_pin->gpio_power = of_get_named_gpio(node, "en-vdd-lan ", 0);
     dts_pin->gpio_rst = of_get_named_gpio(node, "reset-gpio-dm9051", 0);
	 dts_pin->gpio_rst_rtl = of_get_named_gpio(node, "reset-gpio-rtl8305", 0);
			  
	if (!dts_pin->gpio_power)
			  printk("[ *dm9051 READ-MTK dts WARN ] {%s} is not found in the dts data-grouping-set\n", "gpio-power");
	if (!dts_pin->gpio_rst)
			  printk("[ *dm9051 READ-MTK dts WARN ] {%s} is found in the dts data-grouping-set\n", "gpio-rst-dm9051");
	if (!dts_pin->gpio_rst_rtl)
			  printk("[ *dm9051 READ-MTK dts WARN ] {%s} is found in the dts data-grouping-set00\n", "gpio-rst-rtl8305");
			  
	if (dts_pin->gpio_power)
	   printk("[ *dm9051 READ-MTK dts INFO ] {%s} is not found in the dts, Do ctl_pwr_gpio()\n", "gpio-power");
	ctl_pwr_gpio(); /*ret =*/  //xx_gpio(POWER,HIGH,CS,HIGH);
	if (dts_pin->gpio_rst)
		printk("[ *dm9051 READ-MTK dts INFO ] {%s} is found in the dts, Do ctl_rst_qpio()\n", "gpio-rst-dm9051");
	ctl_rst_gpio(); /*ret =*/ 
	return 0;
}

static
void SPI_GPIO_SetupPwrOn(struct board_info *db)
{
	 dts_pin = kmalloc(sizeof(struct pin_ctl), GFP_KERNEL);
	 if(NULL == dts_pin){
			  printk("error : dm9051 malloc pin_ctl failed\n");
			  return;
	 }
	SPI_GPIO_Set(1);
//#ifdef DM_CONF_INTERRUPT	
//	mt_dm9051_irq_no(NULL);
//#endif 
}
#endif //defined MTK_CONF_XY6762TEMP //"|| defined QCOM_CONF_BOARD_YES"

//[ethtool_ops_c]

//[ethtool_ops1_c]

//[driver_c]

/* ops: driver main procedures */
//.void read_isr_print(board_info_t *db)
//.{
	//.#if 0
	//.unsigned  dat= ior(db, DM9051._ISR);
	//.printk("[dm9051.probe.ISR.MBNDRY_STS] data[= 0x%02x]\n", dat);
	//.#endif
//.}

/* event: play a schedule starter in condition */
static netdev_tx_t 
DM9051_START_XMIT(struct sk_buff *skb, struct net_device *dev) //void sta_xmit_sched_delay_work(board_info_t * db)
{
#if DM_CONF_APPSRC
	board_info_t *db = netdev_priv(dev);
#ifdef DM_CONF_POLLALL_INTFLAG	// ONly interrupt mode can has countted 'db->nSCH_INT'
	if (/*db->nSCH_INT &&*/ (db->nSCH_INT <= DM_CONF_SPI_DBG_INT_NUM))
		printk("[DM9051_START_XMIT %d][%d]/[%d].s\n", db->nSCH_XMIT, db->nSCH_INT, DM_CONF_SPI_DBG_INT_NUM);
#endif

  //.#if defined MORE_DM9051_MUTEX && defined  MORE_DM9051_MUTEX_EXT
  //.mutex_lock(&db->spi_lock);
  //.#endif 	
	#if DM_CONF_APPSRC & DM9051_CONF_TX
	toend_stop_queue1(dev, 1 );
	// dm9051_tx
	// Need "skb = skb_dequeue(&db->txq)" to get tx-data
	// JJ: a skb add to the tail of the list '&db->txq'
	skb_queue_tail(&db->txq, skb); 
	#endif

	//#if 0
	//driver_dtxt_step(db, '0');  // driver_dtxt_step(db, 'q'); // Normal
	//#endif
	
	db->nSCH_XMIT++;
  //.#if defined MORE_DM9051_MUTEX && defined  MORE_DM9051_MUTEX_EXT
  //.mutex_unlock(&db->spi_lock);
  //.#endif	
  
  #ifdef DM_CONF_POLLALL_INTFLAG
	dm9051_INTPschedule_isr(db, R_SCH_XMIT); // in 'dm9051_start_xmit'
  #endif

#ifdef DM_CONF_POLLALL_INTFLAG	// ONly interrupt mode can has countted 'db->nSCH_INT'
	if (/*db->nSCH_INT &&*/ (db->nSCH_INT <= DM_CONF_SPI_DBG_INT_NUM))
		printk("[DM9051_START_XMIT %d][%d]/[%d].e\n", db->nSCH_XMIT, db->nSCH_INT, DM_CONF_SPI_DBG_INT_NUM);
#endif		
#endif
	return NETDEV_TX_OK;
}

/* ops */
/* play with a schedule starter */
static void 
dm9051_set_multicast_list_schedule(struct net_device *dev)
{
#if DEF_PRO & DM_CONF_APPSRC
	board_info_t *db = netdev_priv(dev);
	sched_work(db);
#endif
}   

void
dm9051_reload_eeprom(board_info_t *db)
{
#ifdef DM_RELOAD_EEPROM	
	iow(db, DM9051_EPCR, 1 << 5); //EPCR_REEP= 1 << 5, EPCR_ERPRR/EPCR_WEP
	printk("dm951: reload EEPROM (Reloading)\n");
	mdelay(1); //delay (Driver needs to clear it up after the operation completes)
	iow(db, DM9051_EPCR, 0x0);	/* Clear phyxcer write command */
#endif	
}

void 
dm9051_show_eeprom_mac(board_info_t *db)
{
	int i;
	int offset = 0;
	u8 rmac[6];
	for (i = 0; i < 6; i += 2)
		dm9051_read_eeprom(db, (offset + i) / 2, &rmac[i]);
	//printk("read_eeprom mac dm9051 %02x %02x %02x  %02x %02x %02x\n", rmac[0],rmac[1],
	//	rmac[2],rmac[3],rmac[4],rmac[5]);
	printk("dm951: read eeprom MAC: %pM (%s)\n", rmac, "Reading");
}           

void 
dm9051_set_mac_ops(struct net_device *ndev, void *p)
{
	board_info_t *db = netdev_priv(ndev);
	u8 *s = p;
	int offset = 0;
	//u8 rmac[6];
	int i;
	//[param check]
	printk("dm9 [write mac permanently]\n");
	printk("set param mac dm9051 %02x %02x %02x  %02x %02x %02x\n", s[0],s[1],
		s[2],s[3],s[4],s[5]);
	//[dm9]				    
	iow(db, DM9051_PAR+0, s[0]);
	iow(db, DM9051_PAR+1, s[1]);
	iow(db, DM9051_PAR+2, s[2]);
	iow(db, DM9051_PAR+3, s[3]);
	iow(db, DM9051_PAR+4, s[4]);
	iow(db, DM9051_PAR+5, s[5]);
	//for (i = 0; i < 6; i++)
	//ndev->dev_addr[i]= s[i];
	for (i = 0; i < 6; i++) {
	  ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
	}
	  //[mac reg]
	  for (i = 0; i < 6; i++) {
		  if (ndev->dev_addr[i] != s[i]){
			  break;
		  }
	  }
	  if (i!=6) {
	     printk("dm9 set mac(but not as parameters) chip mac %02x %02x %02x  %02x %02x %02x [Can't write]\n", ndev->dev_addr[0],ndev->dev_addr[1],
		ndev->dev_addr[2],ndev->dev_addr[3],ndev->dev_addr[4],ndev->dev_addr[5]);
		return;
	  }
	//[eeprom]
#if 1	
	printk("write eeprom mac dm9051 %02x %02x %02x  %02x %02x %02x\n", s[0],s[1],s[2],s[3],s[4],s[5]);
		
	for (i = 0; i < 6; i += 2)
		dm9051_write_eeprom(db, (offset + i) / 2, s + i);
#endif

	printk("[dm9 write and then read]\n");

	dm9051_show_eeprom_mac(db);
}

int
dm9051_set_mac_address(struct net_device *dev, void *p)
{
	char *s = p;
	//printk("dm9051_set_mac_address %02x %02x %02x  %02x %02x %02x\n", s[0],s[1],s[2],s[3],s[4],s[5]);
	printk("dm9051_set_mac_address (%02x %02x)  %02x %02x %02x  %02x %02x %02x\n", s[0],s[1], s[2],s[3],s[4],s[5],s[6],s[7]);
	
	dm9051_set_mac_ops(dev, s+2);
	return eth_mac_addr(dev, p);
}
    
