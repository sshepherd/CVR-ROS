#include <errno.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <netdb.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/stat.h>
#ifdef __QNX__
  #include <hw/pci.h>
  #include <hw/inout.h>
  #include <sys/neutrino.h>
  #include <sys/mman.h>
#endif
#include "registers.h"

#define SWITCHES 0
#define ATTEN    1
#define READ     0
#define WRITE    1
#define ON       1
#define OFF      0

#define NEW_PMAT 1 
#define write_to_matrix 0
#define read_lookup_table 1 
#define write_lookup_table 1
#define CARDS 20 
#define PHASECODES 8192 
#define BEAMCODES 8192 
#define MAX_FREQS 2000
#define MAX_ANGLES 200

int sock=-1;
int verbose=2;
char summaryfile_prefix[20]="beamcodes";
char beamfile_prefix[20]="beamcodes_cal";
char file_ext[20]=".dat";
char filename[120];
char dir[80];
FILE *beamcodefile=NULL;
FILE *beamtablefile=NULL;
FILE *summaryfile=NULL;
char radar_name[80];

struct timeval t0,t1,t2,t3;
unsigned long elapsed;

/*-SET WRITE ENABLE BIT-------------------------------------------------------*/
int set_WE(int base,int onoff,int radar){
        int temp;
        int portA,portB,portC;
        switch(radar) {
          case 1:
            portC=PC_GRP_0;
            portB=PB_GRP_0;
            portA=PA_GRP_0;
            break;
          case 2:
            portC=PC_GRP_2;
            portB=PB_GRP_2;
            portA=PA_GRP_2;
            break;
        }
#ifdef __QNX__
        if(onoff==OFF){
                temp=in8(base+portC);
                out8(base+portC,temp & 0xfe);
        }
        if(onoff==ON){
                temp=in8(base+portC);
                out8(base+portC,temp | 0x01);
        }
#endif
}
/*-SET READ/WRITE BIT-------------------------------------------------------*/
int set_RW(int base,int rw,int radar){
        int temp;
        int portA,portB,portC;
        switch(radar) {
          case 1:
            portC=PC_GRP_0;
            portB=PB_GRP_0;
            portA=PA_GRP_0;
            break;
          case 2:
            portC=PC_GRP_2;
            portB=PB_GRP_2;
            portA=PA_GRP_2;
            break;
        }
#ifdef __QNX__
        if(rw==READ){
                temp=in8(base+portC);
                out8(base+portC,temp & 0xbf);
        }
        if(rw==WRITE){
                temp=in8(base+portC);
                out8(base+portC,temp | 0x40);
        }
#endif
}
/*-SET SWITCHED/ATTEN BIT-------------------------------------------------------*/
int set_SA(int base,int sa,int radar){
        int temp;
        int portA,portB,portC;
        switch(radar) {
          case 1:
            portC=PC_GRP_0;
            portB=PB_GRP_0;
            portA=PA_GRP_0;
            break;
          case 2:
            portC=PC_GRP_2;
            portB=PB_GRP_2;
            portA=PA_GRP_2;
            break;
        }
#ifdef __QNX__
        if(sa==SWITCHES){
                temp=in8(base+portC);
                out8(base+portC,temp & 0x7f);
        }
        if(sa==ATTEN){
                temp=in8(base+portC);
                out8(base+portC,temp | 0x80);
        }
#endif
}


/*-REVERSE_BITS-------------------------------------------------------*/
int reverse_bits(int data){
	
	int temp=0;
	
	temp=temp + ((data & 1)  << 12);
	temp=temp + ((data & 2)  << 10);
	temp=temp + ((data & 4)  << 8);
	temp=temp + ((data & 8)  << 6);
	temp=temp + ((data & 16)  << 4);
	temp=temp + ((data & 32)  << 2);
	temp=temp + ((data & 64)  << 0);
	temp=temp + ((data & 128)  >> 2);
	temp=temp + ((data & 256)  >> 4);
	temp=temp + ((data & 512)  >> 6);
	temp=temp + ((data & 1024)  >> 8);
	temp=temp + ((data & 2048)  >> 10);
	temp=temp + ((data & 4096)  >> 12);

	return temp;
}
/*-GET_DEALY---------------------------------------------------------*/
float get_delay(int code){
	
	int	i;
	float	delay;
	float	delaylist[]={0.25, 0.45, 0.8, 1.5, 2.75, 5, 8, 15, 25, 45, 80, 140, 250};

	delay=0;
	for(i=0;i<13;i++){
		delay+=( (code & (int)pow(2,i)) >> i)*delaylist[i];
	}	
	return delay;
}
/*-BEAM_CODE---------------------------------------------------------*/
int beam_code(unsigned int base, int code,int radar){
	/* the beam code is 13 bits, pAD0 thru pAD12.  This code
	   uses bits 0-7 of CH0, PortA, and bits 0-4 of CH0, PortB
	   to output the beam code. Note: The beam code is an address
	   of the EEPROMs in the phasing cards.  This code is broadcast
	   to ALL phasing cards.  If you are witing the EEPROM, then this
	   be the beam code you are writing
	*/
	

	int temp;
        int portA,portB,portC;
        switch(radar) {
          case 1:
            portC=PC_GRP_0;
            portB=PB_GRP_0;
            portA=PA_GRP_0;
            break;
          case 2:
            portC=PC_GRP_2;
            portB=PB_GRP_2;
            portA=PA_GRP_2;
            break;
        }
#ifdef __QNX__
    // bit reverse the code
	code=reverse_bits(code);

    // check if beam code is reasonable
	if ( (code>8192) | (code<0) ){
		fprintf(stderr,"INVALID BEAM CODE - must be between 0 and 8192\n");
		return -1;
	}  
    // set CH0, Port A to lowest 8 bits of beam code and output on PortA
	temp=code & 0xff;
	out8(base+portA,temp);
    // set CH0, Port B to upper 5 bits of beam code and output on PortB
	temp=code & 0x1f00;
	temp=temp >> 8;
	out8(base+portB,temp);

    // verify that proper beam code was sent out
	temp=in8(base+portB);
	temp=(temp & 0x1f) << 8;
	temp=temp+in8(base+portA);
	if (temp==code) return 0;
	else{
		fprintf(stderr,"BEAM CODE OUTPUT ERROR - requested code not sent\n");
		return -1;
	}
#else
  return 0; 
#endif
}
/*-SELECT_CARD------------------------------------------------------*/
int select_card(unsigned int base, int address,int radar){

	/* This code selects a card to address.  This can be used for
	   writing data to the EEPROM, or to verify the output of the
	   EEPROM. There are 20 cards in the phasing matrix, addresses
	   0-19.  A card is addressed when this address corresponds to
	   the switches on the phasing card.  Card address 31 is reserved for
	   programming purposes.
	*/
        int portA,portB,portC;
        switch(radar) {
          case 1:
            portC=PC_GRP_0;
            portB=PB_GRP_0;
            portA=PA_GRP_0;
            break;
          case 2:
            portC=PC_GRP_2;
            portB=PB_GRP_2;
            portA=PA_GRP_2;
            break;
        }
#ifdef __QNX__	
	int temp;
	struct 	timespec nsleep;
	nsleep.tv_sec=0;
	nsleep.tv_nsec=5000;


    // check if card address is reasonable
	if ( (address>31) | (address<0) ){
		fprintf(stderr,"INVALID CARD ADDRESS - must be between 0 and 32\n");
		return -1;
	}
    // shift address left 1 bit (write enable is the lowest bit)
	address=address << 1;
    // mask out bits not used for addressing the cards
	address=address & 0x3e;
    // check for other bits in CH0, PortC that may be on
	temp=in8(base+portC);
	temp=temp & 0xc1;
    // add other bit of PortC to the address bits
	address=address+temp;
    // output the address and original other bits to PortC
	out8(base+portC,address);
	nanosleep(&nsleep,NULL);
    // verify the output
	temp=in8(base+portC);
	if (temp==address) return 0;
	else{
		fprintf(stderr,"CARD SELECT OUTPUT ERROR - requested code not sent\n");
		fprintf(stderr," code=%d\n", temp);
		return -1;
	}
#else
  return 0;
#endif	
}

int write_attenuators(unsigned int base, int card, int code, int data,int radar){

        int temp;
        struct  timespec nsleep;
        nsleep.tv_sec=0;
        nsleep.tv_nsec=5000;
        int portA,portB,portC,cntrl1;
        switch(radar) {
          case 1:
            portC=PC_GRP_1;
            portB=PB_GRP_1;
            portA=PA_GRP_1;
            cntrl1=CNTRL_GRP_1;
            break;
          case 2:
            portC=PC_GRP_3;
            portB=PB_GRP_3;
            portA=PA_GRP_3;
            cntrl1=CNTRL_GRP_3;
            break;
        }
    // check that the data to write is valid
        if ( (data>63) | (data<0) ){
                fprintf(stderr,"INVALID DATA TO WRITE - must be between 0 and 63\n");
                return -1;
        }
#ifdef __QNX__
        data=data ^ 0x3f;
    // select card to write
        temp=select_card(base,card,radar);
    // choose the beam code to write (output appropriate EEPROM address
        temp=beam_code(base,code,radar);
        set_SA(base,ATTEN,radar);
    // enable writing
        set_RW(base,WRITE,radar);
    // set CH1, PortA and Port B to output for writing
        out8(base+cntrl1,0x81);
    // bit reverse the data
        data=reverse_bits(data);
    // apply the data to be written to PortA and PortB on CH1
    // set CH1, Port A to lowest 8 bits of data and output on PortA
        temp=data & 0xff;
        out8(base+portA,temp);
    // set CH0, Port B to upper 5 bits of data and output on PortB
        temp=data & 0x1f00;
        temp=(temp >> 8);
        out8(base+portB,temp);
    // toggle write enable bit
        set_WE(base,ON,radar);
        set_WE(base,OFF,radar);
    // reset CH1, PortA and PortB to inputs
        out8(base+cntrl1,0x93);
    // disable writing
        set_RW(base,READ,radar);
        delay(3);
    // verify written data
    // read PortA and PortB to see if EEPROM output is same as progammed
        temp=in8(base+portB);
        temp=temp & 0x1f;
        temp=temp << 8;
        temp=temp + in8(base+portA);
        temp=temp & 0x1f80;
        if (temp == data){
                //printf("    Code read after writing is %d\n", reverse_bits(temp));
         //       printf("       - DATA WAS WRITTEN: data: %x = readback: %x\n", reverse_bits(data), reverse_bits(temp));
                return 0;
        }
        else {
                printf(" ERROR - ATTEN DATA NOT WRITTEN: data: %x != readback: %x :: Code: %d Card: %d\n", reverse_bits(data), reverse_bits(temp),code,card);
                return -1;
        }
#endif
}
/*-WRITE_CODE--------------------------------------------------------*/
int write_data_new(unsigned int base, int card, int code, int data,int radar){

        int temp;
        struct  timespec nsleep;
        nsleep.tv_sec=0;
        nsleep.tv_nsec=5000;
        int portA,portB,portC,cntrl1;
        switch(radar) {
          case 1:
            portC=PC_GRP_1;
            portB=PB_GRP_1;
            portA=PA_GRP_1;
            cntrl1=CNTRL_GRP_1;
            break;
          case 2:
            portC=PC_GRP_3;
            portB=PB_GRP_3;
            portA=PA_GRP_3;
            cntrl1=CNTRL_GRP_3;
            break;
        }

    // check that the data to write is valid
        if ( (data>8192) | (data<0) ){
                fprintf(stderr,"INVALID DATA TO WRITE - must be between 0 and 8192\n");
                return -1;
        }
#ifdef __QNX__
        data=data ^ 0x1fff;
    // select card to write
        temp=select_card(base,card,radar);
    // choose the beam code to write (output appropriate EEPROM address
        temp=beam_code(base,code,radar);
        set_SA(base,SWITCHES,radar);
    // enable writing
        set_RW(base,WRITE,radar);
    // set CH1, PortA and Port B to output for writing
        out8(base+cntrl1,0x81);
    // bit reverse the data
        data=reverse_bits(data);
    // apply the data to be written to PortA and PortB on CH1
    // set CH1, Port A to lowest 8 bits of data and output on PortA
        temp=data & 0xff;
        out8(base+portA,temp);
    // set CH0, Port B to upper 5 bits of data and output on PortB
        temp=data & 0x1f00;
        temp=(temp >> 8);
        out8(base+portB,temp);
    // toggle write enable bit
        set_WE(base,ON,radar);
        set_WE(base,OFF,radar);
    // reset CH1, PortA and PortB to inputs
        out8(base+cntrl1,0x93);
    // disable writing
        set_RW(base,READ,radar);
        delay(3);
    // verify written data
    // read PortA and PortB to see if EEPROM output is same as progammed
        temp=in8(base+portB);
        temp=temp & 0x1f;
        temp=temp << 8;
        temp=temp + in8(base+portA);
        temp=temp & 0x1fff;
        if (temp == data){
                //printf("    Code read after writing is %d\n", reverse_bits(temp));
                return 0;
        }
        else {
                printf(" ERROR - New Card DATA NOT WRITTEN: data: %x != readback: %x :: Code: %d Card: %d\n", reverse_bits(data), reverse_bits(temp),code,card);
                return -1;
        }
#endif
}

/*-WRITE_CODE--------------------------------------------------------*/
int write_data_old(unsigned int base, int card, int code, int data,int radar){

        int temp;
        struct  timespec nsleep;
        nsleep.tv_sec=0;
        nsleep.tv_nsec=5000;
        int portA0,portB0,portC0,cntrl0;
        int portA1,portB1,portC1,cntrl1;
        switch(radar) {
          case 1:
            portC0=PC_GRP_0;
            portC1=PC_GRP_1;
            portB0=PB_GRP_0;
            portB1=PB_GRP_1;
            portA0=PA_GRP_0;
            portA1=PA_GRP_1;
            cntrl0=CNTRL_GRP_0;
            cntrl1=CNTRL_GRP_1;
            break;
          case 2:
            portC0=PC_GRP_2;
            portC1=PC_GRP_3;
            portB0=PB_GRP_2;
            portB1=PB_GRP_3;
            portA0=PA_GRP_2;
            portA1=PA_GRP_3;
            cntrl0=CNTRL_GRP_2;
            cntrl1=CNTRL_GRP_3;
            break;
        }
    // check that the data to write is valid
        if ( (data>8192) | (data<0) ){
                fprintf(stderr,"INVALID DATA TO WRITE - must be between 0 and 8192\n");
                return -1;
        }
#ifdef __QNX__
    // select card 31 so that no real card is selected
        temp=select_card(base,31,radar);
    // choose the beam code to write (output appropriate EEPROM address
        temp=beam_code(base,code,radar);
    // enable writing (turn on WRITE_ENABLE);
        temp=in8(base+portC0);
        temp=temp | 0x01;
        out8(base+portC0,temp);
    // set CH1, PortA and Port B to output for writing
        out8(base+cntrl1,0x81);
    // bit reverse the data
        data=reverse_bits(data);
    // apply the data to be written to PortA and PortB on CH1
    // set CH1, Port A to lowest 8 bits of data and output on PortA
        temp=data & 0xff;
        out8(base+portA1,temp);
    // set CH0, Port B to upper 5 bits of data and output on PortB
        temp=data & 0x1f00;
        temp=(temp >> 8);
        out8(base+portB1,temp);
    // select card to write
        temp=select_card(base,card,radar);
    // wait 20 ms to write
        //delay(20);
        //delay(1);
        //nanosleep(&nsleep,NULL);
    // select card 31 so that no real card is selected
        temp=select_card(base,31,radar);
        //delay(5);
        delay(1);
        //nanosleep(&nsleep,NULL);
    // reset CH1, PortA and PortB to inputs
        out8(base+cntrl1,0x93);
    // disable writing (turn off WRITE_ENABLE);
        temp=in8(base+portC0);
        temp=temp & 0xfe;
        out8(base+portC0,temp);
        //delay(5);
        delay(1);
        //nanosleep(&nsleep,NULL);
    // verify written data
    // select card to read
        temp=select_card(base,card,radar);
        //delay(5);
        delay(1);
        //nanosleep(&nsleep,NULL);
    // read PortA and PortB to see if EEPROM output is same as progammed
        temp=in8(base+portB1);
        temp=temp & 0x1f;
        temp=temp << 8;
        temp=temp + in8(base+portA1);
        if (temp == data){
                //printf("    Code read after writing is %d\n", reverse_bits(temp));
                temp=select_card(base,31,radar);
                return 0;
        }
        else {
                printf(" ERROR - Old Card DATA NOT WRITTEN: data: %x != readback: %x :: Code: %d Card: %d\n", reverse_bits(data), reverse_bits(temp),code,card);
                temp=select_card(base,31,radar);
                return -1;
        }
#endif
}

/*-READ_DATA---------------------------------------------------------*/
int read_data(unsigned int base,int radar){
#ifdef __QNX__	
	int temp;
        int portA1,portB1,portC1;
        switch(radar) {
          case 1:
            portC1=PC_GRP_1;
            portB1=PB_GRP_1;
            portA1=PA_GRP_1;
            break;
          case 2:
            portC1=PC_GRP_3;
            portB1=PB_GRP_3;
            portA1=PA_GRP_3;
            break;
        }
    // read PortA and PortB to see if EEPROM output is same as progammed
	//delay(1);
	temp=in8(base+portB1);
	temp=temp & 0x1f;
	temp=temp << 8;
	temp=temp + in8(base+portA1);

    // bit reverse data
	temp=reverse_bits(temp);

	return temp;
#else
  return 0;
#endif	
}


int main(int argc, char **argv)
{
  double *pwr_mag[MAX_FREQS];
  double freqs[MAX_FREQS];
  double angles[MAX_ANGLES],std_angles[MAX_ANGLES];
  int lowest_pwr_mag_index[3]={-1,-1,-1}; // freq,card,phasecode
  int *best_phasecode[MAX_FREQS], *best_std_phasecode[MAX_FREQS];
  int *best_attencode[MAX_FREQS], *best_std_attencode[MAX_FREQS];
  int *final_beamcodes[CARDS], *final_attencodes[CARDS];
  double *final_angles[CARDS];
  double *final_freqs[CARDS];
  int a=0,i=0,c=0,b=0,rval=0,count=0; 
  int num_freqs,num_angles,num_std_angles,num_beamcodes,num_cards;
  int std_angle_index_offset=0,angle_index_offset=192;
  double df=0.0,angle,freq;
  int requested_phasecode=0, requested_attencode=0,beamcode=0;
  int radar,loop=0,read=0;
  unsigned int portA0,portB0,portC0,cntrl0 ;
  unsigned int portA1,portB1,portC1,cntrl1 ;
	int		 temp, pci_handle, j,  IRQ  ;
	unsigned char	 *BASE0, *BASE1;
	unsigned int	 mmap_io_ptr,IOBASE, CLOCK_RES;
	float		 time;
#ifdef __QNX__
	struct		 _clockperiod new, old;
	struct		 timespec start_p, stop_p, start, stop, nsleep;
#endif
    if(argc <2 ) {
      fprintf(stderr,"%s: invoke with radar number (1 or 2)\n",argv[0]);
      fprintf(stderr,"%s:   optional second argument: loop over cards (1)\n",argv[0]);
      exit(0);
    }
    radar=atoi(argv[1]);
    printf("Radar: %d\n",radar);
    if (argc==3) loop=atoi(argv[2]);
    printf("Loop: %d\n",loop);
    switch(radar) {
      case 1:
            portC0=PC_GRP_0;
            portC1=PC_GRP_1;
            portB0=PB_GRP_0;
            portB1=PB_GRP_1;
            portA0=PA_GRP_0;
            portA1=PA_GRP_1;
            cntrl0=CNTRL_GRP_0;
            cntrl1=CNTRL_GRP_1;
            sprintf(dir,"/superdarn/calibrations/kansas_tx_east/");
        break;
      case 2:
            portC0=PC_GRP_2;
            portC1=PC_GRP_3;
            portB0=PB_GRP_2;
            portB1=PB_GRP_3;
            portA0=PA_GRP_2;
            portA1=PA_GRP_3;
            cntrl0=CNTRL_GRP_2;
            cntrl1=CNTRL_GRP_3;
            sprintf(dir,"/superdarn/calibrations/kansas_tx_west/");
        break;
      default:
        fprintf(stderr,"Invalid radar number %d",radar);
        exit(-1);
    } 
#ifdef __QNX__
    /* SET THE SYSTEM CLOCK RESOLUTION AND GET THE START TIME OF THIS PROCESS */
	new.nsec=10000;
	new.fract=0;
	temp=ClockPeriod(CLOCK_REALTIME,&new,0,0);
	if(temp==-1){
		perror("Unable to change system clock resolution");
	}
	temp=clock_gettime(CLOCK_REALTIME, &start_p);
	if(temp==-1){
		perror("Unable to read sytem time");
	}
	temp=ClockPeriod(CLOCK_REALTIME,0,&old,0);
	CLOCK_RES=old.nsec;
	printf("CLOCK_RES: %d\n", CLOCK_RES);
    /* OPEN THE PLX9656 AND GET LOCAL BASE ADDRESSES */
	fprintf(stderr,"PLX9052 CONFIGURATION ********************\n");
	clock_gettime(CLOCK_REALTIME, &start);
	temp=_open_PLX9052(&pci_handle, &mmap_io_ptr, &IRQ, 1);
	IOBASE=mmap_io_ptr;
	if(temp==-1){
		fprintf(stderr, "	PLX9052 configuration failed");
	}
	else{
		fprintf(stderr, "	PLX9052 configuration successful!\n");
	}
	printf("IOBASE=%x\n",IOBASE);
    /* INITIALIZE THE CARD FOR PROPER IO */
	// GROUP 0 - PortA=output, PortB=output, PortClo=output, PortChi=output
	out8(IOBASE+cntrl0,0x80);
	// GROUP 1 - PortAinput, PortB=input, PortClo=input, PortChi=output
	out8(IOBASE+cntrl1,0x93);
	out8(IOBASE+portA0,0x00);
	out8(IOBASE+portB0,0x00);
	out8(IOBASE+portC0,0x00);
	out8(IOBASE+portA1,0x00);
	out8(IOBASE+portB1,0x00);
	temp=in8(IOBASE+portC1);
	temp=temp & 0x0f;
	printf("input on group 1, port c is %x\n", temp);
#endif

  for (c=0;c<CARDS;c++) {
    final_beamcodes[c]=calloc(BEAMCODES,sizeof(int));
    final_attencodes[c]=calloc(BEAMCODES,sizeof(int));
    final_angles[c]=calloc(BEAMCODES,sizeof(double));
    final_freqs[c]=calloc(BEAMCODES,sizeof(double));
    for (b=0;b<BEAMCODES;b++) {
      final_beamcodes[c][b]=-1;
      final_attencodes[c][b]=-1;
      final_angles[c][b]=-1;
      final_freqs[c][b]=-1;
    }
  }
  printf("\n\nEnter Radar Name: ");
  fflush(stdin);
  scanf("%s", &radar_name);
  fflush(stdout);
  fflush(stdin);

  if(read_lookup_table) {
      sprintf(filename,"%s/beamcode_lookup_table_%s.dat",dir,radar_name,c);
      beamtablefile=fopen(filename,"r+");
      printf("%p %s\n",beamtablefile,filename);
      if(beamtablefile!=NULL) {
        printf("Reading from saved beamcode lookup table\n"); 
        fread(&num_freqs,sizeof(int),1,beamtablefile);
        fread(&num_std_angles,sizeof(int),1,beamtablefile);
        fread(&num_angles,sizeof(int),1,beamtablefile);
        fread(&num_beamcodes,sizeof(int),1,beamtablefile);
        fread(&num_cards,sizeof(int),1,beamtablefile);
        fread(&std_angle_index_offset,sizeof(int),1,beamtablefile);
        fread(&angle_index_offset,sizeof(int),1,beamtablefile);
        fread(freqs,sizeof(double),num_freqs,beamtablefile);
        fread(std_angles,sizeof(double),num_std_angles,beamtablefile);
        fread(angles,sizeof(double),num_angles,beamtablefile);

        printf("Counting saved codes in lookup table\n");
        for (c=0;c<CARDS;c++) { 
          count=0;
          rval=fread(final_beamcodes[c],sizeof(int),num_beamcodes,beamtablefile);
          rval=fread(final_attencodes[c],sizeof(int),num_beamcodes,beamtablefile);
          for (b=0;b<BEAMCODES;b++) {
           if (final_beamcodes[c][b] >= 0 ) {
              count++; 
            }
          }  
          printf("Lookup Card: %d Count: %d\n",c,count);
        }
        fclose(beamtablefile);
        beamtablefile=NULL;
        read=1;
      } else {
        fprintf(stderr,"Error writing beam lookup table file\n");
        read=0;
      }
  }

  if(loop==0) {
    c=-1; 
    printf("\n\nEnter Phasing Card Number: ");
    fflush(stdin);
    fflush(stdout);
    scanf("%d", &c);
    printf("Radar: <%s>  Card: %d\n",radar_name,c);
    fflush(stdout);
  } else {
    c=0;
    printf("Radar: <%s>  All Cards\n",radar_name);
  }
  while((c<CARDS) && (c >=0)) {
      sprintf(filename,"%s%s_%s_%d%s",dir,beamfile_prefix,radar_name,c,file_ext);
      printf("File: %s\n",filename);
      beamcodefile=fopen(filename,"r");
      if(beamcodefile!=NULL) {
        fread(&temp,sizeof(int),1,beamcodefile);
        if(temp!=num_freqs && read ) fprintf(stderr,"num_freq mismatch! %d %d\n",temp,num_freqs); 
        num_freqs=temp;
        printf("Num freqs: %d\n",num_freqs);
        fread(freqs,sizeof(double),num_freqs,beamcodefile);
        df=(double)freqs[1]-(double)freqs[0];

        printf("freq 0: %g df: %lf\n",freqs[0],df);
        printf("freq max: %g df: %lf\n",freqs[num_freqs-1],df);
        fread(&temp,sizeof(int),1,beamcodefile);
        if(temp!=num_std_angles && read) fprintf(stderr,"num_std_angles mismatch! %d %d\n",temp,num_std_angles); 
        num_std_angles=temp;
        printf("Num standard angles: %d\n",num_std_angles);
        fread(std_angles,sizeof(double),num_std_angles,beamcodefile);
        printf("\n");
        for(i=0;i<num_freqs;i++) {
          best_std_phasecode[i]=calloc(num_std_angles,sizeof(int));
          rval=fread(best_std_phasecode[i],sizeof(int),num_std_angles,beamcodefile);
          best_std_attencode[i]=calloc(num_std_angles,sizeof(int));
          rval=fread(best_std_attencode[i],sizeof(int),num_std_angles,beamcodefile);
          if( rval!=num_std_angles) fprintf(stderr,"Error reading angles for freq: %d :: %d %d\n",i,rval,num_std_angles);
        }
        printf("  Freq  ::  Bmnum  ::  Ang   ::  Phasecode :: Attencode\n");
        for(a=0;a<num_std_angles;a++) printf("%8.0lf :: %8d :: %6.1lf :: %8d :: %8d\n",freqs[0],a,std_angles[a],best_std_phasecode[0][a],best_std_attencode[0][a]);
 
        fread(&temp,sizeof(int),1,beamcodefile);
        if(temp!=num_angles && read) fprintf(stderr,"num_angles mismatch! %d %d\n",temp,num_angles); 
        num_angles=temp;
        printf("Num angles: %d\nAngles: ",num_angles);
        fread(angles,sizeof(double),num_angles,beamcodefile);
        for(a=0;a<num_angles;a++) printf("%4.1lf ",angles[a]);
        printf("\n");
        for(i=0;i<num_freqs;i++) {
          best_phasecode[i]=calloc(num_angles,sizeof(int));
          rval=fread(best_phasecode[i],sizeof(int),num_angles,beamcodefile);
          best_attencode[i]=calloc(num_angles,sizeof(int));
          rval=fread(best_attencode[i],sizeof(int),num_angles,beamcodefile);

          if( rval!=num_angles) fprintf(stderr,"Error reading angles for freq: %d :: %d %d\n",i,rval,num_angles);
        }

        fclose(beamcodefile); //beamcode file
        read=1;
        for (b=0;b<BEAMCODES;b++) {
          i=(b)/100;
          if (b<angle_index_offset) {
            i=b/num_std_angles*(int)(1E6/df);
            a=b%num_std_angles;
            requested_phasecode=best_std_phasecode[i][a];
            requested_attencode=best_std_attencode[i][a];
            angle=std_angles[a];
            freq=freqs[i];
//            printf("calculate: %d %d %lf %lf\n",b,i,freq,freqs[i]);
          } else {
            i=(b-angle_index_offset)/num_angles*2E5/df;
            a=(b-angle_index_offset) % num_angles ;
            if (i < num_freqs) {
              requested_phasecode=best_phasecode[i][a];
              requested_attencode=best_attencode[i][a];
              angle=angles[a];
              freq=freqs[i];
            } else {
              requested_phasecode=-1;
              angle=-1E10;
              freq=-1E10;
            }
          } 
          final_beamcodes[c][b]=requested_phasecode;
          final_attencodes[c][b]=requested_attencode;
          final_freqs[c][b]=freq;

          final_angles[c][b]=angle;
        }
      } else {
        printf("Bad beamcode file\n");
      }
      if(loop) c++;
      else c=-1;
  } // end of card loop

  for (c=0;c<CARDS;c++) {
      for (b=0;b<BEAMCODES;b++) {
         if (final_beamcodes[c][b] >= 0 ) {
            if (write_to_matrix) {
              if(NEW_PMAT) {
                temp=write_data_new(IOBASE,c,b,final_beamcodes[c][b],radar);
                temp=write_attenuators(IOBASE,c,b,final_attencodes[c][b],radar); //JDS need to set attenuators here
              } else {
                temp=write_data_old(IOBASE,c,beamcode,b,radar);
              }
            }
         }
      } //end beamcode loop 
  } // end card loop
//JDS write to lookup table
    if(write_lookup_table) {
      printf("Writing Beam lookup table\n");
      sprintf(filename,"%s/beamcode_lookup_table_%s.dat",dir,radar_name,c);
      beamtablefile=fopen(filename,"w+");
      printf("%p %s\n",beamtablefile,filename);
      if(beamtablefile!=NULL) {
        temp=num_freqs;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=num_std_angles;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=num_angles;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=BEAMCODES;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=CARDS;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=std_angle_index_offset;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        temp=angle_index_offset;
        fwrite(&temp,sizeof(int),1,beamtablefile);
        fwrite(freqs,sizeof(double),num_freqs,beamtablefile);
        fwrite(std_angles,sizeof(double),num_std_angles,beamtablefile);
        fwrite(angles,sizeof(double),num_angles,beamtablefile);
        for (c=0;c<CARDS;c++) {
          rval=fwrite(final_beamcodes[c],sizeof(int),BEAMCODES,beamtablefile);
          rval=fwrite(final_attencodes[c],sizeof(int),BEAMCODES,beamtablefile);
        }
        fclose(beamtablefile);
      } else {
        fprintf(stderr,"Error writing beam lookup table file\n");
      }
   }
}

