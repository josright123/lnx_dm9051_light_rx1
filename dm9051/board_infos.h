#define	NUMRXBYTEABNORMAL	10
#define	NUMBOOTRPUPDATE	1	//when boot (driver load) link state change once with boot-rp-update number be 1

struct rx_ctl_mach {
	__le16	RxLen;
	u16		OvrFlw_counter;
	u16		ERRO_counter; /*cycFOUND_counter*/
	u16		RXBErr_counter;
	u16		LARGErr_counter;
	u16		StatErr_counter; /* new */
    u16		DO_FIFO_RST_counter; /*(preRST_counter) / cycRST_counter*/
    
    u16		rxbyte_regs[NUMRXBYTEABNORMAL];
    u8		rxbyte_pad[NUMRXBYTEABNORMAL];
	u16		rxbyte_counter;
	u16		rxbyte_counter0;
	
	u16		rxbyte_counter0_to_prt;
#if 0	
	u16		rx_brdcst_counter; 
	u16		rx_multi_counter;
#endif 	
	u16		rx_unicst_counter;
	u8		isbyte; // ISR Register data
	u8		isr_clear; // 20210201.RX_Pointer_Protect
	//u8	dummy_pad, last_ornext_pad; // dummypad for parsing, lastor for scaning
	char	*dummy_pad_pointer;
	
	int		nRRMAX;
	int		sScanSizeMax;
	u16		DO_RP_Update_counter;
};

struct tx_state_mach {
	u16		prob_cntStopped;
	char	local_cntTXREQ;
	char	pad_0;
	u16		local_cntLOOP; // for trace looptrace_rec[]
#if DRV_TRACE_XLOOP
	#define NUMLPREC  16
	struct loop_tx {
	  u16 	looptrace_rec[NUMLPREC];  	// 20140522
	  int	loopidx;					// 20140522
	} dloop_tx;
#endif
};

typedef struct board_info {
#if DEF_SPICORE_IMPL1
	#ifdef QCOM_BURST_MODE
	 struct spi_transfer spi_xfer2[2] ____cacheline_aligned;
	 struct spi_message spi_msg2 ____cacheline_aligned;
	#else
	 struct spi_transfer Tfer ____cacheline_aligned;
	 struct spi_transfer *fer ____cacheline_aligned;
	 struct spi_message Tmsg ____cacheline_aligned; 
	 struct spi_message *msg ____cacheline_aligned;
	#endif	 
#endif	 
#if DEF_SPICORE_IMPL0	
	struct spi_message	spi_msg1;
	struct spi_transfer	spi_xfer1;
#endif
#if DEF_SPIRW	
        /* DMA buffer */
//.(more_for_cb).#ifdef RPI_CONF_SPI_DMA_YES
        u8 *spi_tx_buf; //(can be = dma_data_buf;) // org = spi_tx_buf
        dma_addr_t spi_tx_dma;
        dma_addr_t spi_rx_dma;
        //u8 *spi_rx_buf;
//.(more_for_cb).#endif
        /* Non-DMA buffer */
        //u8 *TxDatBuf;
//u8 TxDatBuf[SPI_SYNC_TRANSFER_BUF_LEN];  //ADD.JJ
	u8		*spi_sypc_buf;
#ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT	
	u8		*blkin; // also buffer len of 'DM_CONF_SPI_TEST_BLKLEN'
#endif
	char	*prebuf; 
	char	*sbuf; 
	int	validlen_for_prebuf;
	int rx_scan_packets; //new
	int	nRx;
	int	sScanSize;
#endif
	u16 			len_rxhdr_pkt[NUM_SCANX+1] ____cacheline_aligned;
	u16 			mdra_regs[NUM_SCANX+1] ____cacheline_aligned; // one more full-condition room
	u16 			mdra_reg_end ____cacheline_aligned;
	u16				mdwr_reg_end ____cacheline_aligned;
	u16				RdForLen;
	u16 			rwregs[2];
	//u16 		rx_count;
	
	struct sk_buff_head	txq;
	struct rx_ctl_mach	  bC;
	struct tx_state_mach  bt; // .prob_cntStopped
	struct spi_device	*spidev;
	struct net_device   *ndev; /* dm9051's netdev */
	struct mii_if_info 	mii;
	
#ifdef MORE_DM9051_MUTEX
	struct mutex	 	spi_lock;
	#ifdef DM_CONF_TASKLET
	struct tasklet_struct phypoll_tl;
	struct tasklet_struct xmit_tl;
	#else //~DM_CONF_TASKLET
	struct delayed_work	phypoll_work;
	struct delayed_work	xmit_work;
	#endif
#endif

	struct mutex	 	addr_lock;	/* dm9051's lock;*/
	spinlock_t		statelock_tx1_rx1; /* state lock;*/
	
	#ifdef DM_CONF_PHYPOLL	
	#ifdef DM_CONF_TASKLET
	struct tasklet_struct phy_poll_tl;
	#else //~DM_CONF_TASKLET
	struct delayed_work	phy_poll;
	#endif
	#endif
	
	#ifdef DM_CONF_TASKLET
	struct tasklet_struct rxctrl_tl;
	struct tasklet_struct rx_tl;
	#else  //~DM_CONF_TASKLET
	struct work_struct	rxctrl_work;
	#ifndef DM_CONF_THREAD_IRQ
	struct delayed_work	rx_work; //"INT_or_poll_Work;"
	#endif
	#endif
		
	//[int	scanmem;]
#if DM_RX_HEAVY_MODE
	u16		rwtrace1;
	u16		rwregs1;
	u8			rxd[8] ____cacheline_aligned; //aS R1 for R2.R3
	u8			txd[8];
#endif
	
	u32		msg_enable ____cacheline_aligned;

	u8					imr_all;
	u8					rcr_all;
	u8					driver_state;
	u8					chip_code_state;
	int				linkBool;
	char				linkA;
	
	u8  				Enter_hash;
	//u8  			Enter_count;
	u8 					sch_cause;
	u8 					nSCH_INIT;
	u32				nSCH_LINK;
  /*u16*/ u32 			nSCH_INT_NUm, nSCH_INT_NUm_A; //+(nSCH_INT_NUm_A)
	u32 				nSCH_INT, nSCH_INT_B; //+(nSCH_INT_B)
	u32 				nSCH_INT_Glue;
	u16				nSCH_INT_Num_Disp;
	u16 				nSCH_INFINI; // loop to large count, and few looping
	u16 				nSCH_XMIT;
	u16					nSCH_XMIT_WAVE_PDL;
	u16 				nSCH_GoodRX;
	u16 				nSCH_UniRX;
#if DM_RX_HEAVY_MODE
	u16 				DERFER_rwregs[2];
	u16 				DERFER_calc;
	u16 				DERFER_rwregs1[2];
	u16 				DERFER_calc1;
	
	u16 				RUNNING_rwregs[2];
	u16				rwregs_enter, rwregs1_enter; //[trick.extra]
	u16				rwregs_scanend; //[trick.extra]
	u16				scan_end_reg0;
	u16				last_rwregs[2];
	u16				last_calc;
	int				last_nRx;
	u16				calc;
#endif	

	u8					mac_process;
	u8					flg_rxhold_evt;
#if DM_RX_HEAVY_MODE	
	u32				rx_rst_quan;
	u32				rx_tot_quan;
#endif	
	bool				has_do_disable;
	
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE	
	//unsigned long	req_irq_flags;
	u8					irq_type;
#endif
} board_info_t;

#if 0
/*struct dm9000_rxhdr {
	u8	RxPktReady;
	u8	RxStatus;
	__le16	RxLen;
} __packed;*/
#endif

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

#define	RXHDR_SIZE	sizeof(struct dm9051_rxhdr)

static void dm9051_read_eeprom(board_info_t *db, int offset, u8 *to);
static void dm9051_write_eeprom(board_info_t *db, int offset, u8 *data);

// "sub.h" [previous name : "sub_dm9051.h"]

// --- struct declaration ---
// ---  const definition  ---

#define WR_ISR_ENDOF_RXWORK_ONLY //JJ_20190813

#define IIRQ_TYPE_NONE			0
#define IIRQ_TYPE_EDGE_RISING	1
#define IIRQ_TYPE_EDGE_FALLING	2
#define IIRQ_TYPE_LEVEL_HIGH	4
#define IIRQ_TYPE_LEVEL_LOW		8

//[SPI_BUF_LEN]
#define SPI_SYNC_TRANSFER_BUF_LEN (4 + DM9051_PKT_MAX)
//[CUSTOM_BUF_LEN]
#define CMMB_SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES (0x400)

#define DS_NUL							0
#define DS_POLL							1
#define DS_IDLE							2
#define CCS_NUL							0
//#define CCS_PROBE						1

#define	R_SCH_INIT		1
#define R_SCH_XMIT		2
#define R_SCH_INT		3
#define R_SCH_INFINI	4
#define R_SCH_INT_GLUE	5  // vs.R_SCH_.INT
//#define R_SCH_LINK	6
#define R_SCH_PHYPOLL	6  // extended

/* 3p6s.s */
asmlinkage __visible int printkr(const char *fmt, ...){
  return 0; 
}
EXPORT_SYMBOL(printkr);
/* 3p6s.e */
