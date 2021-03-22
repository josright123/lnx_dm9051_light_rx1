#define LNX_KERNEL_v58	0 //0

// To be complete for customize purpose
#ifdef QCOM_CONF_BOARD_YES
#define QCOM_RX_DWORD_BOUNDARY
#define QCOM_TX_DWORD_BOUNDARY
//#define QCOM_BURST_MODE
#endif
#ifndef QCOM_CONF_BOARD_YES
#define QCOM_RX_DWORD_BOUNDARY //[TEMP] //#define QCOM_RX_DWORD_BOUNDARY   //RX NOT simulate 
#define QCOM_TX_DWORD_BOUNDARY
//#define QCOM_BURST_MODE
#endif

 /* ----------------------------- */
 /* {Easy Engineer selection}     */
 /* ----------------------------- */
//.#define MACRO_ASR_QUECTEL
       
 /* ----------------------------- */
 /* {Easy customization config}   */
 /* ----------------------------- */
// To be complete for everyone who is to make experiment (put at below section area)
#ifndef MACRO_ASR_QUECTEL
#if 1

	//
	//  Davicom-RPI-test
	//
	#undef MTK_CONF_XY6762TEMP
	
//#undef DM_CONF_MAX_SPEED_HZ
//#define DM_CONF_MAX_SPEED_HZ	15600000

//#define DM_CONF_ANY_BUF_CASE_SKB_RX_CORE
#define ANY_BUF_NUM	768 //[64 (MacErr early, With Request time out), 128][512, 640, 768, ...]

#endif
#endif

/**  Lab: Options control flags (The usage for the customer who have especial requirments) */
#define DM_CONF_PHYPOLL
#define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE				0
#define DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE		0
//#define DM_DM_CONF_TESTMODE 
//#define DM_CONF_ADVENCE_MEM_YES 
 //#define DM_CONF_SPI_CHIP_SELECT 1 //(#define DM_CONF_SPI_CS1_CASE_YES)

#ifdef SUPP_CONF_DMA_SYNC_YES
#define RPI_CONF_SPI_DMA_YES
#endif

#ifdef DM_DM_CONF_TESTMODE
//This is the test mode 1

 #undef DTS_CONF_YES
 
 #undef DM_CONF_MAX_SPEED_HZ
 #define DM_CONF_MAX_SPEED_HZ 15600000
 #undef DM_CONF_INTERRUPT_IRQ
 #define DM_CONF_INTERRUPT_IRQ	26
 
//Here make it polling driver.
 #undef DM_CONF_INTERRUPT
 #undef DM_CONF_INTERRUPT_IRQ
 #undef DM_CONF_INTERRUPT_LOW_ACTIVE

//Optional..purpose... (e.g. NOT normal case to use tx_1024/rx_1024)
 #undef DM_CONF_MODULE
 #define DM_CONF_MODULE
 #undef DM_CONF_SPI_CHIP_SELECT
 #define DM_CONF_SPI_CHIP_SELECT  1

#endif

#ifdef DTS_CONF_YES

 #define DM_DM_CONF_DTS_COMPATIBLE_USAGE "davicom,dm9051"
 #undef DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
 #define DM_DM_CONF_RARE_PROJECTS_DTS_USAGE 1

#endif

#if DM_DM_CONF_INSTEAD_OF_DTS_AND_BE_DRVMODULE

 #undef DM_CONF_MAX_SPEED_HZ
 #define DM_CONF_MAX_SPEED_HZ (15 * 1000 * 1000 + 600 * 1000)
 #undef DM_CONF_MODULE
 #define DM_CONF_MODULE
 #undef MTK_CONF_YES

#endif

/* -------------- */
/* Generation: dm */
/* -------------- */

#ifndef DM_CONF_SPI_BUS_NUMBER
 #define DRV_SPI_BUS_NUMBER   0
#else
 #define DRV_SPI_BUS_NUMBER   DM_CONF_SPI_BUS_NUMBER
#endif

#ifndef DM_CONF_SPI_CHIP_SELECT
 #define DRV_SPI_CHIP_SELECT   0
#else
 #define DRV_SPI_CHIP_SELECT   DM_CONF_SPI_CHIP_SELECT
#endif

/**  max spi speed: 20MHz [default] */
/**  speed update: chnage with new define const */
#ifndef DM_CONF_MAX_SPEED_HZ
 #define DRV_MAX_SPEED_HZ (20 * 1000 *1000)  // (20MHz)
#else /* DM_CONF_MAX_SPEED_HZ */
 #define DRV_MAX_SPEED_HZ DM_CONF_MAX_SPEED_HZ
#endif /* DM_CONF_MAX_SPEED_HZ */

/**  process: polling [default] */
#define DRV_POLL_1         1   /* This DRV_ POLL_ 1 is always true in design. Because we let INT plays everything POL did. */

#ifdef DM_CONF_INTERRUPT
 #define DM_CONF_POLLALL_INTFLAG  //(Interrupt support, when begin by "DM_CONF_INTERRUPT _1" v.s. "_DRV_INTERRUPT_1")
#endif

#ifdef DM_CONF_INTERRUPT
  //#define _DRV_INTERRUPT_1    1
  //#define DRV_POLL_0  0
#else /* DM_CONF_INTERRUPT */
  //#define _DRV_INTERRUPT_1    0
  //#define DRV_POLL_0  1
#endif /* DM_CONF_INTERRUPT */

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE

//#define DM_CONF_INTERRUPT_TEST_DTS_FALLING  //(Test.First)
//#define DM_CONF_INTERRUPT_TEST_DTS_RISING //(Others)

#else

#ifdef DM_CONF_INTERRUPT_LOW_ACTIVE
#define DRV_IRQF_TRIGGER	IRQF_TRIGGER_LOW
#else /* DM_CONF_INTERRUPT_LOW_ACTIVE */
#define DRV_IRQF_TRIGGER	IRQF_TRIGGER_HIGH
#endif /* DM_CONF_INTERRUPT_LOW_ACTIVE */

#endif

/**  0: dm9051.ko (module) */
/**  1: dm9051.o (static) [default] */
#ifndef DM_CONF_MODULE
 #define DMA3_P6_DRVSTATIC  1  
#else /* DM_CONF_MODULE */
 #define DMA3_P6_DRVSTATIC  0  
#endif /* DM_CONF_MODULE */

#ifdef DM_CONF_INTERRUPT
#define DRV_VERSION	\
	"4.14.79-KT.INT-2.2zcd.light.rx1.20200201_nosave_20210309"
#else
#define DRV_VERSION	\
	"4.14.79-KT.POLL-2.2zcd.light.rx1.20200201_nosave_20210309"
#endif

#define DRV_TRACE_XLOOP						1
#define LOOP_XMIT						1
#define SCH_XMIT						0 

#if LOOP_XMIT
//static char *str_drv_xmit_type= "LOOP_XMIT";
#endif
#if SCH_XMIT
//static char *str_drv_xmit_type= "sch_xmit";
#endif

#define GPIO_ANY_GPIO_DESC  "processer_int_pin" /*"DM9051_INT"*/

#ifdef ENABLE_PRINTLOG_DEBUG
#define printlog(format, args...)     printk(format, ## args)
#else
#define printlog(format, args...)     ((void)0)
#endif

/* --------------- */
/* Generation: mtk */
/* --------------- */

#ifdef MTK_CONF_YES

 #ifdef MTK_CONF_SPI_DMA_YES
   #define DMA3_P0_MTKHDR  		1
   #define DMA3_P1_MTKGPIO  	1
   #define DMA3_P1_MTKSETUP		1

   #define DMA3_P2_MSEL_MOD		1 // SPI_MODE, 0: FIFO, 1: DMA (conjunction with 1024,32,1 bytes or no-limitation)
  #define DMA3_P2_RSEL_1024F	1 // RD_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_RSEL_32F		0 // RD_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
  #define DMA3_P2_TSEL_1024F	1 // TX_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_TSEL_32F		0 // TX_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #else
   #define DMA3_P0_MTKHDR  		1
   #define DMA3_P1_MTKGPIO  	1
   #define DMA3_P1_MTKSETUP		1

   #define DMA3_P2_MSEL_MOD		0
  #define DMA3_P2_RSEL_1024F	0 // RD_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_RSEL_32F		1 // RD_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
  #define DMA3_P2_TSEL_1024F	0 // TX_MACH: FIFO model (1024 bytes-limitation)
  #define DMA3_P2_TSEL_32F		1 // TX_MACH: FIFO model (32 bytes-limitation)
  #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #endif
 
#else /* MTK_CONF_YES */

 #ifdef DM_CONF_1024_BUF_CASE_YES
  #define DMA3_P0_MTKHDR  		0
  #define DMA3_P1_MTKGPIO  		0
  #define DMA3_P1_MTKSETUP		0
  
  #define DMA3_P2_MSEL_MOD		0 // SPI_MODE, 0: FIFO
 #define DMA3_P2_RSEL_1024F		1
 #define DMA3_P2_RSEL_32F		0 // RD_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
 #define DMA3_P2_TSEL_1024F	1
 #define DMA3_P2_TSEL_32F		0 // TX_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #else /* DM_CONF_1024_BUF_CASE_YES */
  #define DMA3_P0_MTKHDR  		0
  #define DMA3_P1_MTKGPIO  		0
  #define DMA3_P1_MTKSETUP		0

  #define DMA3_P2_MSEL_MOD		0 // SPI_MODE, 0: FIFO, 1: DMA (conjunction with 1024,32,1 bytes or no-limitation)
 #define DMA3_P2_RSEL_1024F		0 // RD_MACH: FIFO model (1024 bytes-limitation)
 #define DMA3_P2_RSEL_32F		0 // RD_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_RSEL_1F		0 // RD_MACH: FIFO model (1 byte-limitation)
 #define DMA3_P2_TSEL_1024F		0 // TX_MACH: FIFO model (1024 bytes-limitation)
 #define DMA3_P2_TSEL_32F		0 // TX_MACH: FIFO model (32 bytes-limitation)
 #define DMA3_P2_TSEL_1F		0 // TX_MACH: FIFO model (1 byte-limitation)
 #endif /* DM_CONF_1024_BUF_CASE_YES */
 
#endif /* MTK_CONF_YES */

/* SE - System Environment setup (P0/P1/P3/P4/P6) */
/* PA - Performance Adjustment setup (P5) */
/* -------------------------------------- */
 #define DMA3_P3_KT		  	0 // keep 0 is OK
 #define DMA3_P4_KT		  	0 // keep 0 is OK           
 #define DMA3_P4N_KT		  	1 // keep 0 is OK (new suspend/resume FOR Linux 3.18.19)

/* ------------------------------- */
/* - ReadWrite.configuration.table */
/* ------------------------------- */

//DTS_MODEL
#ifdef DTS_CONF_YES
 #define MSTR_DTS_VERSION		"[ *conf_dts: yes]"   //new-model, 
#else
 #define MSTR_DTS_VERSION		"[ *conf_dts: no]"  //def, 
#endif    

#ifdef DM_CONF_MODULE
#define MSTR_MOD_VERSION		"[ *conf_mod: yes]"
#else
#define MSTR_MOD_VERSION		"[ *conf_mod: no]"
#endif

#ifdef DM_CONF_INTERRUPT
#define MSTR_INT_VERSION		"[ *conf_INT: yes]"
#else
#define MSTR_INT_VERSION		"[ *conf_INT: no]"
#endif

//SPI_MODE.mtk
#if DMA3_P2_MSEL_MOD
 #define MSTR_MTKDMA_VERSION		"[ *SUPP MTK_DMASetup mode: yes]"   //enhance
#else
 //#define MSTR_MTKDMA_VERSION		"[ *SUPP MTK_DMASetup mode: no]"  //def(no printed)
#endif
//SPI_MODE.rpi
#ifdef RPI_CONF_SPI_DMA_YES
 #define MSTR_DMA_SYNC_VERSION		"[ *SUPP conf_dma_sync: yes]"   //rpi.enhance, SYNC_DMA
#else
 #define MSTR_DMA_SYNC_VERSION		"[ *SUPP conf_dma_sync: no]"  //def, SYNC_FIFO
#endif

#ifdef DM_EXTREME_CPU_MODE
 #define MSTR_EXTREME_VERSION		"[ *SUPP dm9_extreme_ver: yes]"
#else
 #define MSTR_EXTREME_VERSION		"[ *SUPP dm9_extreme_ver: no]"
#endif

#ifdef DM_LIGHT_RX
 #define MSTR_LIGHT_RX_VERSION		"[ *SUPP dm9_light_rx: yes]"
#else
 #define MSTR_LIGHT_RX_VERSION		"[ *SUPP dm9_light_rx: no]"
#endif

//RD_MACH
#if DMA3_P2_RSEL_1024F
  #define RD_MODEL_VERSION		"[ *Rx func: 1024-buf]"
#elif DMA3_P2_RSEL_32F
  #define RD_MODEL_VERSION		"[ *Rx func: 32-buf]"	   //test.OK
#elif DMA3_P2_RSEL_1F
  #define RD_MODEL_VERSION		"[ *Rx func: 1-buf]"
#else
  #define RD_MODEL_VERSION		"[ *Rx func: 0-buf]"		   //best
#endif
//TX_MACH
#if DMA3_P2_TSEL_1024F
  #define WR_MODEL_VERSION		"[ *Tx func: 1024-buf]"
#elif DMA3_P2_TSEL_32F
  #define WR_MODEL_VERSION		"[ *Tx func: 32-buf]"	   //tobe.test.again
#elif DMA3_P2_TSEL_1F
  #define WR_MODEL_VERSION		"[ *Tx func: 1-buf]"	   //tobe.test.again
#else
  #define WR_MODEL_VERSION		"[ *Tx func: 0-buf]"		   //best
#endif

// Local compiler-option
/*  RX is basic essential */
/*  TX is made optional('DM9051_CONF_TX') */

#define DM9051_CONF_TX   				1

#if DM9051_CONF_TX
#define NUM_QUEUE_TAIL					0xFFFE   //(2) //(5)//(65534= 0xFFFE)MAX_QUEUE_TAIL  
#endif

#define NUM_SCH_XMIT_WAVE                               5
#define NUM_TRIPS_OF_WAVE                               5
#define NUM_TOTAL_ALL                                   (NUM_SCH_XMIT_WAVE * NUM_TRIPS_OF_WAVE) //is originally defined as 5*5

#define EN_DEBUG

#ifdef EN_DEBUG
#define dbg_log(format, args...)                                                                                        \
                do{                                                                                                                                         \
                                printk(KERN_ERR DRV_NAME": %s() _%d_: " format                       \
                                                                , __FUNCTION__                                                                          \
                                                                , __LINE__                                                                                      \
                                                                , ## args);                                                                                     \
                }while(0)
#else
#define dbg_log(format, args...)     ((void)0)
#endif //EN_DEBUG

