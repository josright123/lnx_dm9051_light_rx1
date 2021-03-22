//----------------------------------------------------------------------------------------
// (customization code)

//[custom_gpio_dm9051_c]

#ifdef QCOM_CONF_BOARD_YES
static irqreturn_t realtek_plug_irq_l(int irq, void *dev_id)
{
/*davicom add begin*/	
	
/*davicom add end*/		
	return IRQ_HANDLED;
}

static irqreturn_t realtek_plug_irq_p(int irq, void *dev_id)
{
/*davicom add begin*/	
	
/*davicom add end*/		
	return IRQ_HANDLED;
}
#endif

void Custom_Board_Init(struct spi_device *spi)
{
#ifdef QCOM_CONF_BOARD_YES
	struct pinctrl *phandle=NULL;
   // struct device_node * np=NULL;
	unsigned int gpio_rst;
	unsigned int gpio_power; 
	//unsigned int gpio_realtek_irq_l; 
	//unsigned int gpio_realtek_irq_p; 
	//unsigned int realtek_irq_no_l= 0;
	//unsigned int realtek_irq_no_p= 0;
	////gary add interrupt for realtek begin 
	
		//gpio_rst = of_get_named_gpio(spi->dev.of_node, "reset-gpio-dm9051", 0);
   //printk("[ *dm9051 probe of_get_named_gpio gpio_rst is %d\n",gpio_rst);
		//gpio_power= of_get_named_gpio(spi->dev.of_node, "en-vdd-lan", 0);
	// printk("[ *dm9051 probe of_get_named_gpio gpio_power is %d\n",gpio_power);	
		//gpio_power= of_get_named_gpio(spi->dev.of_node, "reset-gpio-rtl8305", 0);
	// printk("[ *dm9051 probe of_get_named_gpio 8305reset is %d\n",gpio_power);
/*	
	gpio_realtek_irq_l= of_get_named_gpio(spi->dev.of_node, "8305-irq1-gpio ", 0);
	gpio_realtek_irq_p= of_get_named_gpio(spi->dev.of_node, "8305-irq2-gpio ", 0);
	if (gpio_is_valid(gpio_realtek_irq_l)) {			
			ret = gpio_request(gpio_realtek_irq_l,"realtek_plug1_irq");	
			if (ret) {
			printk("[ *dm9051 realtek irq1 request failed \n");  }	
			realtek_irq_no_l = gpio_to_irq(gpio_realtek_irq_l);
			if (gpio_realtek_irq_l) {
			ret = request_irq(realtek_irq_no_l, realtek_plug_irq_l, IRQF_TRIGGER_RISING, "realtekirql", db->spidev);}
			if (ret) {
			printk("[ *dm9051 realtek irq1 request_irq failed \n"); }
		}

	if (gpio_is_valid(gpio_realtek_irq_p)) {			
			ret = gpio_request(gpio_realtek_irq_p,"realtek_plugp_irq");	
			if (ret) {
			printk("[ *dm9051 realtek irqp request failed \n");  }	
			realtek_irq_no_p = gpio_to_irq(gpio_realtek_irq_p);
			if (gpio_realtek_irq_p) {
			ret = request_irq(realtek_irq_no_p, realtek_plug_irq_p, IRQF_TRIGGER_RISING, "realtekirq2", db->spidev);}
			if (ret) {
			printk("[ *dm9051 realtek irq2 request_irq failed \n"); }
		}	*/	
	//////gary add interrupt for realtek end 
		phandle=devm_pinctrl_get(spi);
		if(IS_ERR_OR_NULL(phandle))
			  printk("[ *dm9051 probe devm_pinctrl_get failed\n");
		  else
			 printk("[ *dm9051 probe devm_pinctrl_get sucess\n");  
		struct pinctrl_state *turnon_reset=pinctrl_lookup_state(phandle,"dm9051_active");
		struct pinctrl_state *turnoff_reset=pinctrl_lookup_state(phandle,"dm9051_sleep");
		struct pinctrl_state *turnon_power=pinctrl_lookup_state(phandle,"lan_active");
		struct pinctrl_state *turnon_reset_8305=pinctrl_lookup_state(phandle,"rtl8305_active");
		if(IS_ERR_OR_NULL(turnon_reset))
				printk("[ *dm9051 probe pinctrl_lookup_state reset failed\n");
			else
				printk("[ *dm9051 probe pinctrl_lookup_state reset sucess\n");
			
			if(IS_ERR_OR_NULL(turnon_power))
				printk("[ *dm9051 probe pinctrl_lookup_state power failed\n");
			else
				printk("[ *dm9051 probe pinctrl_lookup_state power sucess\n");
			pinctrl_select_state(phandle,turnon_power);
			//pinctrl_select_state(phandle,turnon_reset_8305);
			 //msleep(100);
			 // pinctrl_select_state(phandle,turnon_reset);
		 msleep(100);

	 // pinctrl_select_state(phandle,turnoff_reset);
		msleep(10);
	  
	//if (!dts_pin->gpio_rst)
			  //printk("[ *dm9051 READ-MTK dts WARN ] {%s} is found in the dts data-grouping-set\n", "gpio-rst-dm9051");
	//if (!dts_pin->gpio_rst_rtl)
			 //printk("[ *dm9051 READ-MTK dts WARN ] {%s} is found in the dts data-grouping-set00\n", "gpio-rst-rtl8305");
	//dts_pin->gpio_power = of_get_named_gpio(spi->dev.of_node, "en-vdd-lan ", 0);
		// if (!dts_pin->gpio_power)
			 //printk("[ *dm9051 READ-MTK dts WARN ] {%s} is not found in the dts data-grouping-set\n", "gpio-power");
//yangguangfu add
#endif
}

void SPI_SPI_Setup(struct board_info *db) //(struct spi_device *spi)
{    
#if DMA3_P1_MTKSETUP
		SPI_PARAM_Set(db);
#endif

#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
		/* While DTS modle, Define spi max speed in the DTS file */
#else
#if LNX_KERNEL_v58
		db->spidev->max_speed_hz= DRV_MAX_SPEED_HZ;
#else
		db->spidev->max_speed_hz= dm9051_spi_board_devs[0].max_speed_hz;	
#endif
#endif
#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
		/* While DTS modle, Define spi max speed in the DTS file */
#else
		db->spidev->mode = SPI_MODE_0;
		db->spidev->bits_per_word = 8;
		
		printk("%s Driver spi_setup()\n", CARDNAME_9051);
		if(spi_setup(db->spidev)){
			printk("[dm95_spi] spi_setup fail\n");
			return;
		}
#endif		
}

#if DEF_PRO
static int SubNetwork_SPI_Init(struct board_info *db, int enable)
{
	//mutex_lock(&db->addr_lock);
	if(enable){	
#if defined MTK_CONF_XY6762TEMP /* || defined QCOM_CONF_BOARD_YES */
		SPI_GPIO_SetupPwrOn(db);
#endif	
#if 0 //DMA3_P1_MTKSETUP
        SPI_GPIO_Set(1); //mt_dm9051_pinctrl_init(db->spidev); //or, SPI_GPIO_Set(1);
#endif
        SPI_SPI_Setup(db);
	}
	//mutex_unlock(&db->addr_lock);
	return 0;
}
#endif

//----------------------------------------------------------------------------------------

//[printnb_c]
#define printnb(format, args...)	printnb_process(format, ##args)

//printnb.s //[nb: new buffer]
struct {
  int enab;
  int n;
  char bff[100];
} nb;

/* message print [0: disable, 1: enable] */
void printnb_init(int enab)
{
  nb.enab = enab;
  nb.n = 0; //printnb's initialization-reset.
}

void printnb_process(const char *format, ...)
{
  struct va_format vaf;
  va_list args;
  if (!nb.enab)
    return;
  
  va_start(args, format);
  vaf.fmt= format;
  vaf.va= &args;
  nb.n += sprintf(&nb.bff[nb.n], "%pV", &vaf); 
  va_end(args);
  
  if (nb.bff[nb.n -1]=='\n') {
    printk(nb.bff);
    nb.n = 0;
  }
}
//printnb.e
#define DISP_PER_LINE		16
#define DISP_HALF_LINE	8 
#define DISP_PER_MLINE	32
#define DISP_HALF_MLINE	16
void printnb_packet(u8 *mdat, int n) //u8 * //char *
{
	int i;
	 for (i=0; i<n; i++)
	 {
		 if (i && (!(i%DISP_PER_MLINE)) ) printnb("\n");
		 else if (i && (!(i%DISP_HALF_MLINE)) ) printnb(" ");
		 printnb(" %02x", mdat[i]);
	 }
	 printnb("\n");
}

//----------------------------------------------------------------------------------------

void conf_spi_print(struct spi_device *spi)
{
	#if DM_DM_CONF_RARE_PROJECTS_DTS_USAGE
	int rc;
	struct device_node *nc;
	u32 value;
	nc = of_find_compatible_node(NULL, NULL, DM_DM_CONF_DTS_COMPATIBLE_USAGE);
	if (!nc) {
		printk("[ *dm9051 dts WARN ] %s has no spi-max-frequency[= ?]\n", nc->full_name);
		return;
	}
	
	//;if (nc) { //get DTS define as spi0 or spi1 ?
	//;}
	
	rc = of_property_read_u32(nc, "spi-max-frequency", &value); // read to DTS 
	if (rc) {
		printk("[ *dm9051 dts WARN ] No DTS compatible not find, so SPI speed[= ?]\n");
		return;
	}
	
	/* DTS, this speed setting is in the dts file, not in the driver ! */
	//if (!rc) {
	//}
	
	printk("[ *dm9051 dts: yes] %s spi-max-frequency[= %d]\n", nc->full_name, value);
	printk("[ *dm9051 dts: yes] spi bus_num[= %d]\n", spi->master->bus_num);
	printk("[ *dm9051 dts: yes] spi  spi_cs[= %d]\n", spi->chip_select);
	return;
	#else
#if !LNX_KERNEL_v58
	printk("[ *dm9051 dts: no] CONFIG SPI speed[= %d]\n", dm9051_spi_board_devs[0].max_speed_hz);
#endif
	printk("[ *dm9051 dts: no] spi bus_num[= %d]\n", DRV_SPI_BUS_NUMBER);
	printk("[ *dm9051 dts: no] spi  spi_cs[= %d]\n", DRV_SPI_CHIP_SELECT);
	#endif	
}

//----------------------------------------------------------------------------------------

//[board_info_c]
#if DEF_PRO | DEF_OPE
/*
 *  init
 */	
static void SCH_clear(board_info_t *db) //open
{
	//db->rx_count= //need in open only
    db->Enter_hash=
    db->sch_cause=
	db->nSCH_INIT=
	db->nSCH_LINK=
	db->nSCH_XMIT=
	db->nSCH_GoodRX = 
	db->nSCH_UniRX =
	db->nSCH_INT= db->nSCH_INT_B=
	db->nSCH_INT_Glue=
	db->nSCH_INT_NUm= db->nSCH_INT_NUm_A=
	db->nSCH_INFINI = 
	db->mac_process = 0;
	db->nSCH_INT_Num_Disp= 100;
}
#endif

#if DEF_PRO
void Operation_clear(board_info_t *db) // probe
{
	db->rwtrace1=
	db->RUNNING_rwregs[0]=
	db->RUNNING_rwregs[1]= 0;
	
	db->bC.OvrFlw_counter=
	db->bC.ERRO_counter=
	db->bC.RXBErr_counter=
	db->bC.LARGErr_counter=
	db->bC.StatErr_counter=
    db->bC.DO_FIFO_RST_counter= 
    db->rx_rst_quan= 
    db->rx_tot_quan= 0;
}
#endif

//[EXTRA]
/* ----- This is essential for working buffer ----- */
static int dm9051_dbg_alloc(struct board_info *db)
{
#ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
	db->blkin = kmalloc(DM_CONF_SPI_TEST_BLKLEN, GFP_ATOMIC);
        if (!db->blkin)
                return -ENOMEM;
#endif

#ifdef FREE_NO_DOUBLE_MEM_MAX
	db->prebuf = kmalloc((SCAN_LEN_HALF+1)*1, GFP_ATOMIC); 
#else
	db->prebuf = kmalloc((SCAN_LEN_HALF+1)*2, GFP_ATOMIC);   //or 'SCAN_LEN'
#endif

        if (!db->prebuf) { 
#ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
		kfree(db->blkin);
#endif		
                return -ENOMEM;
	}                             
	//[db->scanmem= 0;]
#ifdef FREE_NO_DOUBLE_MEM_MAX
	db->sbuf = db->prebuf;
#else
	db->sbuf = db->prebuf + (SCAN_LEN_HALF+1);
#endif
	return 0;
}
/* ----- This is essential for working buffer ----- */
static void  dm9051_dbg_free(struct board_info *db)
{
#ifdef DM_CONF_SPI_TEST_BLKIN_SPLIT
	kfree(db->blkin);
#endif
	kfree(db->prebuf);
}

#if DEF_PRO
#define DM9051_PHY		0x40	/* PHY address 0x01 */

//SPI:
// do before: mutex_lock(&db->addr_lock); | (spin_lock_irqsave(&db->statelock,flags);)
// do mid: spin_unlock_irqrestore(&db->statelock,flags);, spin_lock_irqsave(&db->statelock,flags);
// do after: (spin_unlock_irqrestore(&db->statelock,flags);) | mutex_unlock(&db->addr_lock);
static int dm9051_phy_read(struct net_device *dev, int phy_reg_unused, int reg)
{
	board_info_t *db = netdev_priv(dev);
	int ret;

	/* Fill the phyxcer register into REG_0C */
	iiow(db, DM9051_EPAR, DM9051_PHY | reg);
	iiow(db, DM9051_EPCR, EPCR_ERPRR | EPCR_EPOS);	/* Issue phyxcer read command */

	//dm9051_msleep(db, 1);		/* Wait read complete */
	//= 
	while ( ior(db, DM9051_EPCR) & EPCR_ERRE) ;

	iiow(db, DM9051_EPCR, 0x0);	/* Clear phyxcer read command */
	/* The read data keeps on REG_0D & REG_0E */
	ret = (ior(db, DM9051_EPDRH) << 8) | ior(db, DM9051_EPDRL);
	return ret;
}

static void dm9051_phy_write(struct net_device *dev,
		 int phyaddr_unused, int reg, int value)
{
	board_info_t *db = netdev_priv(dev);

	printk("iowPHY[%02d %04x]\n", reg, value);
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9051_EPAR, DM9051_PHY | reg);
	/* Fill the written data into REG_0D & REG_0E */
	iiow(db, DM9051_EPDRL, value);
	iiow(db, DM9051_EPDRH, value >> 8);
	iow(db, DM9051_EPCR, EPCR_EPOS | EPCR_ERPRW);	/* Issue phyxcer write command */

	//dm9051_msleep(db, 1);		/* Wait write complete */
	//= 
	while ( ior(db, DM9051_EPCR) & EPCR_ERRE) ;

	iow(db, DM9051_EPCR, 0x0);	/* Clear phyxcer write command */
}

static int dm9051_phy_read_lock(struct net_device *dev, int phy_reg_unused, int reg)
{
	int val;
	board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	val= dm9051_phy_read(dev, 0, reg);
	mutex_unlock(&db->addr_lock);
	return val;
}
static void dm9051_phy_write_lock(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
	board_info_t *db = netdev_priv(dev);
	mutex_lock(&db->addr_lock);
	dm9051_phy_write(dev, 0, reg, value);
	mutex_unlock(&db->addr_lock);
}
#endif

#if DEF_OPE
static void dm9051_init_dm9051(struct net_device *dev)
{
	board_info_t *db = netdev_priv(dev);
#if DEF_SPIRW	
	int	phy4;
	iiow(db, DM9051_GPCR, GPCR_GEP_CNTL);	/* Let GPIO0 output */
	
	/* dm9051_reset(db); */

/* DBG_20140407 */
  phy4= dm9051_phy_read(dev, 0, MII_ADVERTISE);	
  dm9051_phy_write(dev, 0, MII_ADVERTISE, phy4 | ADVERTISE_PAUSE_CAP);	/* dm95 flow-control RX! */	
  dm9051_phy_read(dev, 0, MII_ADVERTISE);

	/* Program operating register */
	iow(db, DM9051_TCR, 0);	        /* TX Polling clear */
	iiow(db, DM9051_BPTR, 0x3f);	/* Less 3Kb, 200us */
	iiow(db, DM9051_SMCR, 0);        /* Special Mode */
	/* clear TX status */
	iiow(db, DM9051_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);
	iow(db, DM9051_ISR, ISR_CLR_STATUS); /* Clear interrupt status */
#endif
	/* Init Driver variable */
	db->imr_all = IMR_PAR | IMR_PRM; /* "| IMR_PTM" */
#ifdef JABBER_PACKET_SUPPORT
	db->rcr_all= RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN | RCR_DIS_WATCHDOG_TIMER;
#else	
	db->rcr_all= RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN;
#endif

	/*
	 * (Set address filter table) 
	 * After.call.ndo_open
	 * "kernel_call.ndo_set_multicast_list.later".
	*/
    //(1)
    #if DM_CONF_APPSRC
    dm9051_fifo_reset(1, NULL, db); // 'NULL' for reset FIFO, and no increase the RST counter
    #endif
    int_reg_stop(db); //iiow(db, DM9051_IMR, IMR_PAR); //= int_reg_stop()
}
#endif

#if DEF_OPE
static void dm9051_open_code(struct net_device *dev, board_info_t *db) // v.s. dm9051_probe_code()
{
	//[(db->chip_code_state==CCS_NUL)].OK.JJ
	
    /* Note: Reg 1F is not set by reset */
#if DEF_SPIRW
    iow(db, DM9051_GPR, 0);	/* REG_1F bit0 activate phyxcer */
#endif
    mdelay(1); /* delay needs by DM9051 */ 
	
    /* Initialize DM9051 board */
    dm9051_reset(db);
	dm9051_init_dm9051(dev);
}
#endif

#if DEF_STO
static void dm9051_stop_code(struct net_device *dev, board_info_t *db) // v.s. dm9051_probe_code()
{
#if DEF_SPIRW	
	mutex_lock(&db->addr_lock);
	dm9051_phy_write(dev, 0, MII_BMCR, BMCR_RESET);	/* PHY RESET */
	iow(db, DM9051_GPR, 0x01);	/* Power-Down PHY */
	//int._reg_stop(db); //iow(db, DM9051_IMR, IMR_PAR);	/* Disable all interrupt */
	iow(db, DM9051_RCR, RCR_RX_DISABLE);	/* Disable RX */
	mutex_unlock(&db->addr_lock);
#endif
}
#endif

//[spi_user_c]
#if DEF_OPE
void read_intcr_print(board_info_t *db)
{
	unsigned  rdv = 0;
	unsigned char *int_pol;
#if DEF_SPIRW
	rdv= iior(db, DM9051_INTCR);
#endif
	int_pol= "------- active high -------";
	if (rdv&0x01)
	  int_pol= "------- active low -------";
	printk("ior[REG39H][%02x] (b0: %d)(%s)\n", rdv, rdv&0x01, int_pol);
}
#endif

/* ops */
/**
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device.
 */
static int
dm9051_open(struct net_device *dev)
{
//	printk("[dm951_open].maincode.s ------- 02.s -------\n");
#if DEF_OPE
	do {
	board_info_t *db = netdev_priv(dev);
	SCH_clear(db);

#if 0		
#ifdef DM_CONF_POLLALL_INTFLAG	
/* (Must be after dm9051.open.code()) */
    int_begin(db->spidev, dev); 
/* (Splite this func to = 'int_get_attribute' + 'int_get_begin') */
#endif
#endif
#if 1
#ifdef DM_CONF_POLLALL_INTFLAG	
	int_get_attribute(db->spidev, dev);
#endif
#endif

	mutex_lock(&db->addr_lock); //Note: must 
	
	printk("dm9.[dm9051_open_c].s\n");
	
	if (db->chip_code_state==CCS_NUL)
		dm9051_open_code(dev, db);
		
	read_intcr_print(db);
	printk("dm9.[dm9051_open_c].e\n");
	
#if 1
#ifdef DM_CONF_POLLALL_INTFLAG	
/* (Must be after dm9051.open.code()) or later and before int_en() */
    int_get_begin(db->spidev, dev); // (disable.irq)_insided
#endif
#endif

#if DM_CONF_APPSRC	
	netif_carrier_off(dev); //new_add: (We add in begin for 'dm_schedule._phy' or 'dm_sched_start_rx' to detect and change to linkon.)
#endif    
  #if DM_CONF_APPSRC & DM9051_CONF_TX
    //[Init.] [Re-init compare to ptobe.] //db->tx_eq= 0; //db->tx_err= 0;
	skb_queue_head_init(&db->txq); 
	netif_start_queue(dev);
  #endif	
  
	mutex_unlock(&db->addr_lock);
	
  #if DM_CONF_APPSRC & DM9051_CONF_TX
    opening_wake_queue1(dev);
  #endif	
  
//#ifdef DM_CONF_POLLALL_INTFLAG	
//	printk("[dm951_open].maincode.m ------- 02.e.INTmode -------\n\n");
//#else
//	printk("[dm951_open].maincode.m ------- 02.e.POLLmode -------\n\n");
//#endif
  
#ifdef DM_CONF_POLLALL_INTFLAG	
	printk("[dm951_open].INT_EN.s -------\n");
#else
	printk("[dm951_open].POLL.s -------\n");
#endif	
	  
#if DEF_SPIRW
	printk("[dm951_open].[before-int_reg_start (IMR %02x ) statis nSCH_INT= %d] -------\n\n", /*ior*/ iior(db, DM9051_IMR), db->nSCH_INT);
#endif

#ifdef DM_CONF_POLLALL_INTFLAG	
	int_en(dev);
	int_reg_start(db, "[dm951_INT_open]"); //peek(imr_all)
#else
	int_reg_start(db, "[dm951_poll_open]"); //pol_reg_start(db);
#endif

#if DEF_SPIRW
	iiow(db, DM9051_RCR, db->rcr_all);
#endif
  #if defined DM_CONF_PHYPOLL & DM_CONF_APPSRC
	dm_schedule_phy(db); //.........dfbtyjuyukytru8k8.....
  #endif
#if DM_CONF_APPSRC
	dm_sched_start_rx(db); //Finally, start the delay work, to be the last calling, for you can not read/wrie dm9051 register since poling schedule work has began! 
  #endif
  
#ifdef DM_CONF_POLLALL_INTFLAG	
	printk("[dm951_open].INT_EN.e -------\n");
#else
	printk("[dm951_open].POLL.e -------\n");
#endif	

//#ifdef DM_CONF_POLLALL_INTFLAG	
//	printk("[dm951_open].maincode.e ------- 02.e.INTmode -------\n\n");
//#else
//	printk("[dm951_open].maincode.e ------- 02.e.POLLmode -------\n\n");
//#endif
	} while (0);
#endif
	return 0;
}

/**
 * dm951_net_stop - close network device
 * @dev: The device being closed.
 *
 * Called to close down a network device which has been active. Cancell any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state whilst it is not being used.
 */
static int dm9051_stop(struct net_device *dev)
{
	printk("[dm951_if_stop].s ------- 02.e -------\n");
#if DEF_STO
	do {
	board_info_t *db = netdev_priv(dev);
#ifdef DM_CONF_POLLALL_INTFLAG	
	int_dis(dev);
	 mutex_lock(&db->addr_lock);
	int_reg_stop(db);
	 mutex_unlock(&db->addr_lock);
    int_end(db->spidev, db);
#endif

	/* "kernel_call.ndo_set_multicast_list.first". */
	/* Then.call.ndo_stop                          */
	db->driver_state= DS_IDLE;
	db->chip_code_state= CCS_NUL;
	
#if DM_CONF_APPSRC	
	sched_delay_work_cancel(db);
	toend_stop_queue1(dev, NUM_QUEUE_TAIL);
#endif
	//JJ-Count-on
	netif_carrier_off(dev);
	
	/* dm9051_shutdown(dev) */
	dm9051_stop_code(dev, db);
	} while (0);
#endif
	return 0;
}

/* ops */
#if 1
//#if DEF_PRO
static const struct net_device_ops dm9051_netdev_ops = {
	.ndo_open		= 	dm9051_open,
	.ndo_stop		= 	dm9051_stop,
	.ndo_start_xmit		= 	DM9051_START_XMIT, //(dm9051_start_xmit)
//>.ndo_tx_timeout		= 	dm9000_timeout,
	.ndo_set_rx_mode 	= 	dm9051_set_multicast_list_schedule,
//..ndo_do_ioctl		= 	dm9051_ioctl,
//..ndo_set_features		= 	dm9000_set_features,
#if !LNX_KERNEL_v58
	.ndo_change_mtu		= 	eth_change_mtu,
#endif
	.ndo_validate_addr	= 	eth_validate_addr,
	.ndo_set_mac_address	= 	dm9051_set_mac_address, //eth_mac_addr,	
	#ifdef CONFIG_NET_POLL_CONTROLLER
	//.ndo_poll_controller= ...
	#endif
};
//#endif
#endif

//[../new_load/driver.c]
 	
/*
 * Search DM9051 board, allocate space and register it
 */
//static int
//dm9051_probe_db(struct spi_device *spi)
//{
//	board_info_t *db = dev_get_drvdata(&spi->dev); //struct board_info *db;//db= netdev_priv(ndev);
//	db->spidev = spi; //[This is for using 'spi' by 'db' pointer]	
//	return 0;
//}

static int
dm9051_probe_ndev(struct spi_device *spi)
{
	//struct net_device *ndev = spi->dev;
	board_info_t *db = dev_get_drvdata(&spi->dev);
	struct net_device *ndev = db->ndev;
	ndev->if_port = IF_PORT_100BASET;
#if 1
	ndev->netdev_ops	= &dm9051_netdev_ops;
#endif
#if 1
      ndev->ethtool_ops = &dm9051_ethtool_ops;
#endif
	return 0;
}

//[debug_as/eth_1.c]
// eth.c
	
void disp_mtu(struct net_device *dev)
{
	printk("dm951: mtu %d\n", dev->mtu);
}

void conf_mii(struct net_device *dev, struct board_info *db)
{	
	db->mii.dev = dev;
	db->mii.phy_id_mask  = 1;   //db->mii.phy_id_mask  = 0x1f;
	db->mii.reg_num_mask = 0xf; //db->mii.reg_num_mask = 0x1f;
	db->mii.phy_id		= 1;
#if DEF_SPIRW
	db->mii.mdio_read    = dm9051_phy_read_lock;
	db->mii.mdio_write   = dm9051_phy_write_lock;
#endif
}

void control_objects_init(board_info_t *db)
{                     
	struct net_device *ndev = db->ndev;                  
#ifdef MORE_DM9051_MUTEX
	mutex_init(&db->spi_lock);
#endif
	mutex_init(&db->addr_lock);
	spin_lock_init(&db->statelock_tx1_rx1); // used in 'dm9051' 'start' 'xmit'
#if DM_CONF_APPSRC		
	define_delay_work(db);
#endif	
#if DM_CONF_APPSRC	
	toend_stop_queue1(ndev, NUM_QUEUE_TAIL); //ending_stop_queue1(ndev);	
#endif	
	skb_queue_head_init(&db->txq); //[Init.]
}


#if DEF_PRO
unsigned dm9051_chipid(board_info_t * db)
{
	unsigned  chipid = 0;
#if DEF_SPIRW
	chipid= ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL= %02x]\n", DM9051_PIDL, chipid);
	chipid |= (unsigned)ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
	if (chipid == (DM9051_ID>>16))
		return chipid;
	
	chipid= ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL] = %02x\n", DM9051_PIDL, chipid);
	chipid |= (unsigned)ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
	if (chipid == (DM9051_ID>>16))
		return chipid;
	
	chipid= ior(db, DM9051_PIDL); //printk("ior %02x [DM9051_PIDL= %02x]\n", DM9051_PIDL, chipid);
	chipid |= (unsigned)ior(db, DM9051_PIDH) << 8; //printk("+ ior %02x [DM9051_PIDH <<8] = %04x\n", DM9051_PIDH, chipid);
	if (chipid == (DM9051_ID>>16))
		return chipid;
#endif
	return chipid;
}
#endif

/*
 * Search DM9051 board, allocate space and register it
 */
#if 0
/*int dm9051_set_mac(board_info_t *db, struct net_device *ndev)
{
#if DEF_SPIRW
	int i;
	for (i = 0; i < 6; i++)
      ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
#endif
	printk("dm951: at MAC: %pM (%s)\n", ndev->dev_addr, "DBG-0");
		   
	if (!is_valid_ether_addr(ndev->dev_addr)) {
#if DEF_SPIRW
		iow(db, DM9051_PAR+0, 0x00);
		iow(db, DM9051_PAR+1, 0x60);
		iow(db, DM9051_PAR+2, 0x6e);
		iow(db, DM9051_PAR+3, 0x90);
		iow(db, DM9051_PAR+4, 0x51);
		iow(db, DM9051_PAR+5, 0xee);
		for (i = 0; i < 6; i++)
		  ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
#endif
		printk("dm951: at MAC: %pM (%s)\n", ndev->dev_addr, "DBG-0.1");
		return 0; //[mac_src= "fixed2chip.0";] //["Free-Style";]
	}
	return 1;
}*/
#endif

static int
dm9051_probe(struct spi_device *spi)
{
	const unsigned char *mac_src;
	
#if DEF_PRO
	do {
	struct board_info *db;
	struct net_device *ndev;
	unsigned  chipid;
#if DEF_SPIRW
	int i;
#endif	
	int ret = 0;
   
	printnb_init(1); // 1 for print-log, 0 for no print-log 
	ndev = alloc_etherdev(sizeof(struct board_info));
	if (!ndev) {
		dev_err(&spi->dev, "failed to alloc ethernet device\n");
		return -ENOMEM;
	}
	
	ndev->mtu = 1500; // My-eth-conf
	/* setup board info structure */
	db = netdev_priv(ndev);
	db->ndev = ndev;
	db->spidev = spi;
	
	#if 1
	SET_NETDEV_DEV(ndev, &spi->dev);
	/*
	 * No need: db->dev = &pdev->dev;            
	 * May need: dev_set_drvdata(&spi->dev, db); 
	 */
	dev_set_drvdata(&spi->dev, db);
	#endif
	
	dm9051_probe_ndev(spi); /* (ndev) */

#if DEF_SPIRW
	Custom_Board_Init(spi);
	printk("SubNetwork_SPI_Init\n");
	SubNetwork_SPI_Init(db, 1); //contain with spi->bits_per_word = 8;
	if (dm9051_dbg_alloc(db)) {
		ret = -ENOMEM;
		goto err_first_prepare;
	}
	if (dm9051_spirw_begin(db)) {
		ret = -ENOMEM;
		goto err_prepare;
	}
	dm9051_spimsg_init(db);
#endif	
	
	control_objects_init(db);
#if 0	

#ifdef MORE_DM9051_MUTEX
	mutex_init(&db->spi_lock);
#endif
	mutex_init(&db->addr_lock);
	spin_lock_init(&db->statelock_tx1_rx1); // used in 'dm9051' 'start' 'xmit'
#if DM_CONF_APPSRC		
	define_delay_work(db);
#endif	
	
#if DM_CONF_APPSRC	
	toend_stop_queue1(ndev, NUM_QUEUE_TAIL); //ending_stop_queue1(ndev);	
#endif	
	skb_queue_head_init(&db->txq); //[Init.]
	
#endif	
	
	/* initialise pre-made spi transfer messages */
#if DEF_SPICORE_IMPL0	
	spi_message_init(&db->spi_msg1); ..............dnknbgr.................
	spi_message_add_tail(&db->spi_xfer1, &db->spi_msg1);
#endif
	
	/* setup mii state */
	disp_mtu(ndev);
	conf_mii(ndev, db);          
	
//]	#if 1
//]	SET_NETDEV_DEV(ndev, &spi->dev);
	/*
	 * No need: db->dev = &pdev->dev;            
	 * May need: dev_set_drvdata(&spi->dev, db); 
	 */
//]	dev_set_drvdata(&spi->dev, db);
//]	#endif
//]	dm9051_probe_ndev(spi); /* (ndev) */

	dm9051_reset(db);
	conf_spi_print(spi);
	chipid= dm9051_chipid(db); 
	if (chipid!=(DM9051_ID>>16)) {
		printk("Read [DM9051_PID] = %04x\n", chipid);
		printk("Read [DM9051_PID] error!\n");
		goto err_id;
	}

	//MAC
	mac_src= "eeprom2chip.0"; //"Free-Style";
	
if (1) {	
	int i;
	for (i=0; i<6; i++){
	printk("\n");
	dm9051_reload_eeprom(db);
	dm9051_show_eeprom_mac(db);
	}
}
	
#if DEF_SPIRW
	for (i = 0; i < 6; i++)
      ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
#endif
	printk("dm951: at MAC: %pM (%s)\n", ndev->dev_addr, "DBG-0");
		   
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		mac_src= "fixed2chip.0"; //"Free-Style";	
#if DEF_SPIRW
		iow(db, DM9051_PAR+0, 0x00);
		iow(db, DM9051_PAR+1, 0x60);
		iow(db, DM9051_PAR+2, 0x6e);
		iow(db, DM9051_PAR+3, 0x90);
		iow(db, DM9051_PAR+4, 0x51);
		iow(db, DM9051_PAR+5, 0xee);
		for (i = 0; i < 6; i++)
		  ndev->dev_addr[i]= ior(db, DM9051_PAR+i);
#endif
		printk("dm951: at MAC: %pM (%s)\n", ndev->dev_addr, "DBG-0.1");
	}
	
//]	ndev->if_port = IF_PORT_100BASET;
//]	ndev->netdev_ops	= &dm9051_netdev_ops;
//]#if DMA3_P3_KT
//]	SET_ETHTOOL_OPS(ndev, &dm9051_ethtool_ops); /*3p*/  
//]#else
//]	ndev->ethtool_ops = &dm9051_ethtool_ops;
//]#endif

 
	printk("dm951: mtu %d\n", ndev->mtu);
	printk("[dm9051_eth(n)-reg-netdev].s --------- 01.m1 ---------\n");
	ret = register_netdev(ndev);
    if (ret) {
	dev_err(&spi->dev, "failed to register network device\n");
	printk("[  dm9051  ] dm9051_probe {failed to register network device}\n");
	goto err_netdev;
    }
	printk("[dm9051_(%s)-reg-netdev].s --------- 01.m2 ---------\n", ndev->name);
	printk("dm951 %s: mtu %d\n", ndev->name, ndev->mtu);
    
	//printk("dm951: at MAC: %pM (%s)\n", ndev->dev_addr, "DBG-1");
		   
    db->driver_state= DS_NUL;
    db->chip_code_state= CCS_NUL;
	printk("dm951 %s: at MAC: %pM, summary (%s)\n", //isNO_IRQ %d 
		   ndev->name, ndev->dev_addr, mac_src); //ndev->irq,
    printk("dm951 %s: bus_num %d, spi_cs %d\n",  //"(%s)", DRV_VERSION
           ndev->name, spi->master->bus_num, 
    	   spi->chip_select); 
	printk("[dm95_spi] spi_setup db->spidev->bits_per_word= %d\n",db->spidev->bits_per_word);
	printk("[dm95_spi] spi_setup db->spidev->mode= %d\n",db->spidev->mode);
	printk("[dm95_spi] spi_setup db->spidev->db->spidev->max_speed_hz= %d\n",db->spidev->max_speed_hz);
	Operation_clear(db); //[In probe, this should be essential.]
	SCH_clear(db); //[In probe, this should be not essential.]
	return 0;
		   
err_netdev:
err_id:
	dm9051_spirw_end(db);
#if DEF_SPIRW
err_prepare:
	dm9051_dbg_free(db);
err_first_prepare:
#endif
	free_netdev(ndev);
	return ret;
	} while (0);
#endif
	return 0;
}

//--------------------------------------------------------------------------------------

static int
dm9051_drv_remove(struct spi_device *spi)  // vs. dm9051_probe
{
    //printk("[dm951_u-probe].s ------- 01.s -------\n");
#if DEF_REM    
	do {
    board_info_t *db = dev_get_drvdata(&spi->dev);
    
    dm9051_spirw_end(db);
    //kfree(db->spi_sypc_buf);
    //devm_kfree(&spi->dev, db->spi_sypc_buf);
    dm9051_dbg_free(db);
    
  //int._end(db->spidev, db);
	unregister_netdev(db->ndev);
	free_netdev(db->ndev);
	} while(0);
#endif
	return 0;
}   
