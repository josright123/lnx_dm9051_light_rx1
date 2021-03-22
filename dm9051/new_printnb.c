
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

void printnb_packeth(u8 *mdat, int n) //u8 * //char *
{
	int i;
	 for (i=0; i<n; i++)
	 {
	    if (i && (!(i%DISP_PER_MLINE)) ) printnb("\n");
	    else 
	    if (i && (!(i%DISP_HALF_MLINE)) ) printnb(" ");
	    if (i < 8)
	      printnb(" %02x", mdat[i]);
	    else
	      printnb(" ..");
	 }
	 printnb("\n");
}

void printnb_packetdot(u8 *mdat, int n) //u8 * //char *
{
	int i;
	 for (i=0; i<n; i++)
	 {
	    if (i && (!(i%DISP_PER_MLINE)) ) printnb("\n");
	    else 
	    if (i && (!(i%DISP_HALF_MLINE)) ) printnb(" ");
	    printnb(" ..");
	 }
	 printnb("\n");
}

void printnb_packetend(u8 *mdat, int n) 
{
	int i;
	if (n <= 16)
	{
	for (i=0; i<n; i++)
	  printnb(" %02x", mdat[i]);
	printnb("\n");
	return;
	}
	
	//
	//printnb_packetdot(mdat, 16);
	for (i=0; i<8; i++)
	  printnb(" ..");
	mdat+=8;
	n -= 8;
	for (i=0; i<8; i++)
	  printnb(" %02x", mdat[i]);
	mdat+=8;
	n -= 8;
	//=
	//for (i=0; i<16; i++)
	//  printnb(" ..");
	//mdat+=16;
	//n -= 16;
	  
	printnb(" ");
	for (i=0; i<n; i++)
	  printnb(" %02x", mdat[i]);
	printnb("\n");
}

//----------------------------------------------------------------------------------------

void printnb_rx_fifo(u8 *hdrdat, int hdrlen, u8 *mdat, int len) 
{
  int pp;
    printnb_packet(hdrdat, hdrlen);
    if (len < 96)
      printnb_packet(mdat, len);
    else {
      pp = 0;
      printnb_packet(mdat, 64);
      pp += 64;
      printnb_packeth(mdat+pp, 32);
      pp += 32;
      
      if ((len-pp) <= 32)
	printnb_packetend(mdat+pp, len-pp);
      else {
	printnb_packetdot(mdat+pp, 32);
	pp += 32;
	if ((len-pp) <= 32)
	  printnb_packetend(mdat+pp, len-pp);
	else {
	  pp = (len/32)*32; //(len >> 5) << 5;
	  printnb_packetend(mdat+pp, len-pp);
	}
      }
    }
}
