/*
 * File:        ADC > FFT > TFT
 * Author:      Bruce Land
 * 
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_3.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
#include <math.h>
////////////////////////////////////
// DDS sine table


/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// === the fixed point macros ========================================
typedef signed short fix14 ;
#define RECORD_TIME 5

int refresh =1;
#define multfix14(a,b) ((fix14)((((long)(a))*((long)(b)))>>14)) //multiply two fixed 2.14
#define float2fix14(a) ((fix14)((a)*16384.0)) // 2^14
#define fix2float14(a) ((float)(a)/16384.0)
#define absfix14(a) abs(a) 
// === input arrays ==================================================
#define nSamp 256
#define sample_rate 16000
#define RECORD_COUNT (sample_rate*RECORD_TIME/nSamp)
#define nPixels 340
fix14 v_in[nSamp],v_in_temp[nSamp] ;
#define Filter_bank_size 22
// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_fft ;
static short x=30, y=0, ypow, ypow_1[Filter_bank_size], color;
      static short color_index, display_phase ;
// system 1 second interval tick
int sys_time_seconds ;
//int mel[22]={19,24,29,35,41,47,54,62,70,79,89,99,110,122,135,148,163,179,196,215,235,256};
int freq[22]={300,376,458,547,642,745,856,975,1103,1241,1389,1549,1721,1906,2105,2320,2551,2800,3067,3355,3666,4000};
int mel[22]={4,6,7,8,10,11,13,15,17,19,22,24,27,30,33,37,40,44,49,53,58,64};

int m=0;
int bandData[20];
int mode=0,prev_mode=0;
int color_map[32]={0x07F7,
0x07F5,
0x07F2,
0x07F0,
0x07ED,
0x07EA,
0x07E8,
0x07E5,
0x07E2,
0x07E0,
0x17E0,
0x2FE0,
0x47E0,
0x57E0,
0x6FE0,
0x87E0,
0x97E0,
0xAFE0,
0xBFE0,
0xD7E0,
0xEFE0,
0xFFE0,
0xFF40,
0xFEA0,
0xFDE0,
0xFD40,
0xFCA0,
0xFC00,
0xFB40,
0xFAA0,
0xFA00,
0xF940};

int record =0, play=0;
int frame_count=0;
void printval( char* print_buffer){
 
    int iNumSendChars = 0;
    while (print_buffer[iNumSendChars] != '\0'){
        while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, print_buffer[iNumSendChars]);
        iNumSendChars++;
    }
    
    iNumSendChars=0;
    while(!UARTTransmitterIsReady(UART1));
        UARTSendDataByte(UART1, '\n');
        

}
// === SPI setup ========================================================
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this RAM
//volatile int spiClkDiv = 4 ;

// === RAM commands ======================================================
// from 23LC1024 datasheet
// http://ww1.microchip.com/downloads/en/DeviceDoc/20005142C.pdf
// Default setup includes data streaming from current address
// This is used in the array read/write functions
#define RAM_WRITE_CMD (0x2000000) // top 8 bits -- 24 bits for address
#define RAM_READ_CMD  (0x3000000) // top 8 bits -- 24 bits for address

// command format consists of a read or write command ORed with a 24-bit
// address, even though the actual address is never more than 17 bits
// general procedure will be:
// Drop chip-select line
// Do a 32-bit SPI transfer with command ORed with address
// switch to 8-bit SPI mode
// Send one or more data bytes
// Raise chip-select line
//
// At 20 MHz SPI bus rate:
// Command ORed with address, plus mode change, takes 2.2 microsec
// Each byte takes 0.75 microseconds 

/* ====== MCP4822 control word =========================================
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 ? Don?t Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. ?
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
VOUT pin is connected to 500 k???typical)?
bit 11-0 D11:D0: DAC Input Data bits. Bit x is ignored.
*/
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
char buffer[60];
char print_buffer[30];
void printLine(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 10 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(1);
    tft_writeString(print_buffer);
}

void printLine2(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 20 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 16, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(2);
    tft_writeString(print_buffer);
}





// === spi bit widths ====================================================
// hit the SPI control register directly
inline void Mode16_2(void){  // configure SPI1 for 16-bit mode
    SPI2CONSET = 0x400;
    SPI2CONCLR = 0x800;
}
// ========
inline void Mode8_2(void){  // configure SPI1 for 8-bit mode
    SPI2CONCLR = 0x400;
    SPI2CONCLR = 0x800;
}
// ========
inline void Mode32_2(void){  // configure SPI1 for 8-bit mode
    SPI2CONCLR = 0x400;
    SPI2CONSET = 0x800;
}
// === DAC byte write =====================================================
// address between 0 and 2^17-1
// data bytes
void dac_write_byte(int data){
    int junk;
    // Channel config ORed with data
    Mode16_2();
    mPORTBClearBits(BIT_1);
    // write to spi2 and convert 8 bits to 12 bits
    WriteSPI2(DAC_config_chan_A | (data));
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    junk = ReadSPI2(); // must always read, even if nothing useful is returned
    // set 8-bit transfer for each byte
    mPORTBSetBits(BIT_1);
    return ;
}

// === RAM byte write =====================================================
// address between 0 and 2^17-1
// data bytes
void ram_write_byte(int addr, char data){
    int junk;
    // set 32-bit transfer for read/write command ORed with
    // actual address
    Mode32_2();
    mPORTAClearBits(BIT_4);
    WriteSPI2(RAM_WRITE_CMD | addr); // addr not greater than 17 bits
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    junk = ReadSPI2(); // must always read, even if nothing useful is returned
    // set 8-bit transfer for each byte
    Mode8_2();
    WriteSPI2(data); // data write
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    junk = ReadSPI2();
    mPORTASetBits(BIT_4);
    return ;
}

// === RAM array write =====================================================
// address, pointer to an array containing the data, number of BYTES to store
void ram_write_byte_array(int addr, char* data, int count){
    int junk, i;
    Mode32_2();
    mPORTAClearBits(BIT_4);
    WriteSPI2(RAM_WRITE_CMD | addr); // addr not greater than 17 bits
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    junk = ReadSPI2();
    Mode8_2();
    for(i=0; i<count; i++){
        WriteSPI2(data[i]); // data write
        while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
       
        junk = ReadSPI2();
    }
    mPORTASetBits(BIT_4);
    return ;
}

// === RAM byte read ======================================================
int ram_read_byte(int addr){
    int junk, data;
    Mode32_2();
    mPORTAClearBits(BIT_4);
    WriteSPI2(RAM_READ_CMD | addr); // addr not greater than 17 bits
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    junk = ReadSPI2();
    Mode8_2();
    WriteSPI2(junk); // force the read
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    data = ReadSPI2();
    mPORTASetBits(BIT_4);
    return data;
}

// === RAM array read ======================================================
// address, pointer to an array receiving the data, number of BYTES to read
int ram_read_byte_array(int addr, char* data, int count){
    int junk, i;
    Mode32_2();
    mPORTAClearBits(BIT_4);
    WriteSPI2(RAM_READ_CMD | addr); // addr not greater than 17 bits
    while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
    junk = ReadSPI2();
    Mode8_2();
    for(i=0; i<count; i++){
        WriteSPI2(junk); // force the read
        while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
        data[i] = ReadSPI2();
    }
    mPORTASetBits(BIT_4);
    return ;
}




// === print line for TFT ============================================
// print a line on the TFT
// string buffer

//=== FFT ==============================================================
// FFT
#define N_WAVE          256    /* size of FFT 512 */
#define LOG2_N_WAVE     8    /* log2(N_WAVE) 0 */
#define begin {
#define end }

fix14 Sinewave[N_WAVE]; // a table of sines for the FFT
fix14 window[N_WAVE]; // a table of window values for the FFT
fix14 fr[N_WAVE], fi[N_WAVE];
int pixels[nPixels] ;

void FFTfix(fix14 fr[], fix14 fi[], int m)
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m
// This routine does foward transform only
begin
    int mr,nn,i,j,L,k,istep, n;
    int qr,qi,tr,ti,wr,wi;

    mr = 0;
    n = 1<<m;   //number of points
    nn = n - 1;

    /* decimation in time - re-order data */
    for(m=1; m<=nn; ++m)
    begin
        L = n;
        do L >>= 1; while(mr+L > nn);
        mr = (mr & (L-1)) + L;
        if(mr <= m) continue;
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        //ti = fi[m];   //for real inputs, don't need this
        //fi[m] = fi[mr];
        //fi[mr] = ti;
    end

    L = 1;
    k = LOG2_N_WAVE-1;
    while(L < n)
    begin
        istep = L << 1;
        for(m=0; m<L; ++m)
        begin
            j = m << k;
            wr =  Sinewave[j+N_WAVE/4];
            wi = -Sinewave[j];
            //wr >>= 1; do need if scale table
            //wi >>= 1;

            for(i=m; i<n; i+=istep)
            begin
                j = i + L;
                tr = multfix14(wr,fr[j]) - multfix14(wi,fi[j]);
                ti = multfix14(wr,fi[j]) + multfix14(wi,fr[j]);
                qr = fr[i] >> 1;
                qi = fi[i] >> 1;
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            end
        end
        --k;
        L = istep;
    end
end

// === FFT Thread =============================================
    
// DMA channel busy flag
#define CHN_BUSY 0x80
#define log_min 0x10   
static PT_THREAD (protothread_fft(struct pt *pt))
{
    PT_BEGIN(pt);
    static int sample_number ;
    static fix14 zero_point_4 = float2fix14(0.4) ;
    // approx log calc ;
    static int sx, y, ly, temp ;
    display_phase = 0;
    while(1) {
        // yield time 1 second
       // PT_YIELD_TIME_msec(30);
        
        if(!play && !record){
        
        // enable ADC DMA channel and get
        // 512 samples from ADC
        // yield until DMA done: while((DCH0CON & Chn_busy) ){};
        PT_WAIT_WHILE(pt, DCH0CON & CHN_BUSY);
        memcpy(v_in,v_in_temp,nSamp*2);
        DmaChnEnable(0);

        // profile fft time
      //   sprintf(buffer, "FFT time %dms", (ReadTimer4()*64)/40000);
     //   printLine(1, buffer, ILI9340_WHITE, ILI9340_BLACK);    
        WriteTimer4(0);
        
        // compute and display fft
        // load input array
        for (sample_number=0; sample_number<nSamp; sample_number++){
            // window the input and perhaps scale it
            fr[sample_number] = multfix14(v_in[sample_number], window[sample_number]); 
            fi[sample_number] = 0 ;
        }
        
        // do FFT
        FFTfix(fr, fi, LOG2_N_WAVE);
   /*     // get magnitude and log
        // The magnitude of the FFT is approximated as: 
        //   |amplitude|=max(|Re|,|Im|)+0.4*min(|Re|,|Im|). 
        // This approximation is accurate within about 4% rms.
        // https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
      for (sample_number = 0; sample_number < nPixels; sample_number++) {  
            // get the approx magnitude
            fr[sample_number] = abs(fr[sample_number]); //>>9
            fi[sample_number] = abs(fi[sample_number]);
            // reuse fr to hold magnitude
            fr[sample_number] = max(fr[sample_number], fi[sample_number]) + 
                    multfix14(min(fr[sample_number], fi[sample_number]), zero_point_4); 
            
            // reuse fr to hold log magnitude
            // shifting finds most significant bit
            // then make approxlog  = ly + (fr-y)./(y) + 0.043;
            // BUT for an 8-bit approx (4 bit ly and 4-bit fraction)
            // ly 1<=ly<=14
            // omit the 0.043 because it is too small for 4-bit fraction
            
            sx = fr[sample_number];
            y=1; ly=0;
            while(sx>1) {
               y=y*2; ly=ly+1; sx=sx>>1;
            }
            // shift ly into upper 4-bits as integer part of log
            // take bits below y and shift into lower 4-bits
            fr[sample_number] = ((ly)<<4) + ((fr[sample_number]-y)>>(ly-4) ) ;
            // bound the noise at low amp
            if(fr[sample_number]<log_min) fr[sample_number] = log_min;
        }
        
        // timer 4 set up with prescalar=8, 
        // hence mult reading by 8 to get machine cycles
    //    sprintf(buffer, "FFT cycles %d", (ReadTimer4())*8);
    //    printLine2(11, buffer, ILI9340_WHITE, ILI9340_BLACK);
        
        // Display on TFT
        // erase, then draw
     /*   for (sample_number = 0; sample_number < nPixels; sample_number++) {
            tft_drawPixel(sample_number, pixels[sample_number], ILI9340_BLACK);
            pixels[sample_number] = -fr[sample_number] + 200 ;
            tft_drawPixel(sample_number, pixels[sample_number], ILI9340_WHITE);
            // reuse fr to hold magnitude 
        }    
      */
        long place, root;
       
        for (sample_number = 0; sample_number < nSamp; sample_number++) {
            fr[sample_number] = (fr[sample_number] * fr[sample_number] + 
                  fi[sample_number] * fi[sample_number]);
                   
            // Now we find the square root of realNumbers[k] using a very
            // fast (but compiler dependent) integer approximation:
            // (adapted from: http://www.codecodex.com/wiki/Calculate_an_integer_square_root)
            place = 0x40000000;
			root = 0;
			
			if ( fr[sample_number] >= 0) // Ensure we don't have a negative number
			{
				while (place >  fr[sample_number]) place = place >> 2; 
				
				while (place)  
				{  
					if ( fr[sample_number] >= root + place)  
					{  
						 fr[sample_number] -= root + place;  
						root += place * 2;  
					}  
					root = root >> 1;  
					place = place >> 2;  
				}
			}
			 fr[sample_number] = root;
            
            
        //tft_drawFastVLine(sample_number, 0,  220, ILI9340_BLACK);
	    //tft_drawFastVLine(sample_number, 0,  fr[sample_number], ILI9340_RED);
        }
        prev_mode=mode;
        mode=mPORTBReadBits(BIT_3);
        if(mode!=prev_mode || refresh){
              tft_fillScreen(ILI9340_BLACK);
            refresh=0;
                int k=0;

                if(mode){
                    int m=0;
                    for(m=0;m<32;m++)
                    tft_fillRoundRect(305,225-(7*m), 15, 7, 1, color_map[m]);// x,y,w,h,radius,color
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setTextSize(1);
                    for(k=0;k<22;k++){
                        tft_setCursor(0, 230-(10*k));
                        sprintf(buffer, "%d", freq[k]);
                        tft_writeString(buffer);
                    }
                        
                        
                }
                else{
                    tft_setTextColor(ILI9340_WHITE); 
                    tft_setTextSize(1);
                    tft_setRotation(0); 
                    for(k=0;k<22;k++){
                        tft_setCursor(0, 15*k);
                        sprintf(buffer, "%d", freq[k]);
                        tft_writeString(buffer);
            
                }
        tft_setRotation(1); 
                }
        
        }
        if(mode){
 tft_fillRoundRect(x-1, 0, 3, 10, 1, ILI9340_BLACK);// x,y,w,h,radius,color
 tft_fillRoundRect(x, 0, 3, 10, 1, ILI9340_RED);// x,y,w,h,radius,color
  tft_fillRoundRect(x+1, 20, 3, 240, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        }
      char str[128];
int index = 0;  
 int startFreqIdx, centerFreqIdx, stopFreqIdx, magnitudeScale;   
for(m= 1; m <= 20; m++) {
    startFreqIdx = mel[m-1];
     centerFreqIdx = mel[m];
     stopFreqIdx   = mel[m+1];
     int prevbandData =bandData[m];
     int temp1=0,temp2=0;
bandData[m]=0;
    for( sample_number = startFreqIdx; sample_number < centerFreqIdx; sample_number++){
       // magnitudeScale = centerFreqIdx-startFreqIdx;
        temp1 += fr[sample_number]*(sample_number-startFreqIdx);//magnitudeScale;
    }
    for( sample_number = centerFreqIdx; sample_number <= stopFreqIdx; sample_number++) {
      //  magnitudeScale = centerFreqIdx-stopFreqIdx;
       temp2 += fr[sample_number]*(sample_number-stopFreqIdx);///magnitudeScale;
    }
bandData[m]=(temp1/(centerFreqIdx-startFreqIdx))+(temp2/(centerFreqIdx-stopFreqIdx));
/*

    */
 index += sprintf(&str[index], "%d, ", bandData[m]);
if(mode){
color_index=bandData[m]>>2;
        
/*if(color_index< 10)
    color=color_index;
else if(color_index< 20)
    color=(color_index<<6);
else
    color=(color_index<<11);
            // */
    
    if(color_index<32)
        color =color_map[color_index];
    else
    color =0xF800;
           // tft_fillRoundRect(x,230-(10*m), 1, 10, 1, color);// x,y,w,h,radius,color
             tft_drawFastVLine(x    , 230-(10*m)  ,10, color); // Left
}
   
        
      
        else{
        
     /*   if(prevbandData>bandData[m])
    tft_fillRect(m*16,bandData[m], 12,prevbandData-bandData[m],ILI9340_BLACK);
else if(prevbandData<bandData[m])
    tft_fillRect(m*16,prevbandData, 12, bandData[m]-prevbandData,ILI9340_RED);
       */ 
        
            tft_fillRect((m-1)*16+2,0, 12,205,ILI9340_BLACK);
      
        tft_fillRect((m-1)*16+2,205-bandData[m], 12,bandData[m],ILI9340_RED);
       
        }

  
}
      
if(mode) {  
    x++;
    if (x>300) x=30 ;
}
 printval(str);
    }
        
    else if(record){

    mPORTBSetBits(BIT_8);
    tft_fillScreen(ILI9340_BLACK);
    tft_setTextColor(ILI9340_WHITE); 
    tft_setTextSize(3);
    tft_setCursor(70, 100);
    sprintf(buffer, "RECORDING");
    tft_writeString(buffer);
    tft_fillRect(100,140, 100,10,ILI9340_WHITE);
	for(frame_count=0;frame_count<RECORD_COUNT;frame_count++){
    tft_fillRect(100+(frame_count*100/RECORD_COUNT),140, 1,10,ILI9340_RED);

    DmaChnEnable(0);
        // yield until DMA done: while((DCH0CON & Chn_busy) ){};
    PT_WAIT_WHILE(pt, DCH0CON & CHN_BUSY);
    memcpy(v_in,v_in_temp,nSamp*2);
    ram_write_byte_array(frame_count*nSamp*2,(char *)v_in,nSamp*2);    
    //ram_read_byte_array(frame_count*512*2,(char *) sin_table2,512*2);
 /*int i;
        for(i=0;i<nSamp;i++)
 {sprintf(buffer, "%x\t\t%x", sin_table[i],sin_table2[i]);
        printLine(i+1, buffer, ILI9340_WHITE, ILI9340_BLACK);
    }
   
  */
    }
    mPORTBClearBits(BIT_8); 
refresh=1;   
    record=0;    
    }
        
    else if(play){
        DmaChnDisable(0);
        mPORTBSetBits(BIT_9);
        DmaChnOpen(1, 0, DMA_OPEN_DEFAULT);
        DmaChnSetTxfer(1, (char*)v_in, &SPI2BUF, nSamp*2, 2,2); //512 16-bit integers
	    DmaChnSetEventControl(1, DMA_EV_START_IRQ(_TIMER_3_IRQ));
        tft_fillScreen(ILI9340_BLACK);
    tft_setTextColor(ILI9340_WHITE); 
    tft_setTextSize(3);
    tft_setCursor(85, 100);
    sprintf(buffer, "PLAYING");
    tft_writeString(buffer);
      
    tft_fillRect(100,140, 100,10,ILI9340_WHITE);
	for(frame_count=0;frame_count<RECORD_COUNT;frame_count++){    
    tft_fillRect(100+(frame_count*100/RECORD_COUNT),140, 1,10,ILI9340_GREEN);
        ram_read_byte_array(frame_count*nSamp*2,(char *) v_in,nSamp*2);
      
        int i;
        for(i=0;i<nSamp;i++)
           // sprintf(buffer, "%x", v_in[i]);
        v_in[i]=DAC_config_chan_A|(v_in[i]<<2);
        SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL, 2);
          PPSOutput(4, RPB10, SS2);
	
        DmaChnSetEvEnableFlags(1, DMA_EV_BLOCK_DONE);
         DmaChnEnable(1);      
        while(!(DmaChnGetEvFlags(1)&DMA_EV_BLOCK_DONE));            
        // delay_ms(1000/16);
        DmaChnClrEvFlags(1, DMA_EV_BLOCK_DONE);
        SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE8 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);

    }
  mPORTBClearBits(BIT_9);     
        play=0;
        refresh=1;
        DmaChnDisable(1);
        DmaChnEnable(0);
    }    
        
         // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // FFT thread


void __ISR(_EXTERNAL_4_VECTOR, ipl5) INT4Interrupt() {
    
   mINT4ClearIntFlag();  
    
    record=1;
    
    
}


void __ISR(_EXTERNAL_1_VECTOR, ipl4) INT1Interrupt() {
    
   mINT1ClearIntFlag();  
    
    play=1;
    
    
}


// === Main  ======================================================

void main(void) {
    //SYSTEMConfigPerformance(PBCLK);

    ANSELA = 0;
    ANSELB = 0;

    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    // the ADC ///////////////////////////////////////
    
    // timer 3 setup for adc trigger  ==============================================
    // Set up timer3 on,  no interrupts, internal clock, prescalar 1, compare-value
    // This timer generates the time base for each ADC sample. 
    // works at ??? Hz
  
    // 40 MHz PB clock rate
    #define timer_match 40000000/sample_rate
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, timer_match); 


    //=== DMA Channel 0 transfer ADC data to array v_in ================================
    // Open DMA Chan1 for later use sending video to TV
    #define DMAchan0 0
	DmaChnOpen(DMAchan0, 0, DMA_OPEN_DEFAULT);
    DmaChnSetTxfer(DMAchan0, (void*)&ADC1BUF0, (void*)v_in_temp, 2, nSamp*2, 2); //512 16-bit integers
    DmaChnSetEventControl(DMAchan0, DMA_EV_START_IRQ(28)); // 28 is ADC done
    // ==============================================================================
    
    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_TMR | ADC_AUTO_SAMPLING_ON //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    //
    // Define setup parameters for OpenADC10
    // for a 40 MHz PB clock rate
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy should work at 40 MHz.
    // ADC_SAMPLE_TIME_6 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_6 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_5| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN11 and  as analog inputs
    #define PARAM4	ENABLE_AN11_ANA // pin 24

    // define setup parameters for OpenADC10
    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

    // use ground as neg ref for A | use AN11 for input A     
    // configure to sample AN11 
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11); // configure to sample AN4 
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC
    ///////////////////////////////////////////////////////
   // OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, sys_clock/256000);
  //  OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,sys_clock/512000 ,sys_clock/256000 );
  //  PPSOutput(	1	,	RPA0	,	OC1	    );// 
    // timer 4 setup for profiling code ===========================================
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, compare-value
    // This timer generates the time base for each horizontal video line
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_64, 0xffff);
        
    // === init FFT data =====================================
    // one cycle sine table
    //  required for FFT
    int ii;
    for (ii = 0; ii < N_WAVE; ii++) {
        Sinewave[ii] = float2fix14(sin(6.283 * ((float) ii) / N_WAVE)*0.5);
        window[ii] = float2fix14(1.0 * (1.0 - cos(6.283 * ((float) ii) / (N_WAVE - 1))));
        //window[ii] = float2fix(1.0) ;
    }
    // ========================================================
    
    // init the threads
    PT_INIT(&pt_fft);

    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    delay_ms(1000);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
for(m= 1; m <= 20; m++) {
    bandData[m]=0;
}
    
  
    
     // === set up SPI =======================
  // SCK2 is pin 26 
  // SDO2 (MOSI) is in PPS output group 2, could be connected to RPB5 which is pin 14
  PPSOutput(2, RPB5, SDO2);
  // SDI2 (MISO) is PPS output group 3, could be connected to RPA2 which is pin 9
  PPSInput(3,SDI2,RPA2);

  // control CS for RAM (bit 0) and for DAC (bit 1)
  mPORTASetPinsDigitalOut(BIT_4);

  //and set  both bits to turn off both enables
  mPORTASetBits(BIT_4);
  // divide Fpb by 2, configure the I/O ports. Not using SS in this example
  // 8 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripheral, you will need to match these
  SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE8 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
 
  mPORTBSetPinsDigitalOut(BIT_10);

  //and set  both bits to turn off both enables
mPORTBSetBits(BIT_10);  
Mode16_2();
mPORTAClearBits(BIT_4);
WriteSPI2(0x0140); // 
while (SPI2STATbits.SPIBUSY); // wait for it to end of transaction
ReadSPI2();
mPORTASetBits(BIT_4);

mPORTBSetPinsDigitalOut(BIT_8 | BIT_9);//RECORD LED PLAY LED
mPORTBClearBits(BIT_8 | BIT_9);
mPORTBSetPinsDigitalIn(BIT_3);  //MODE SELECT
PPSInput(1, INT4, RPB7);//RECORD
ConfigINT4(EXT_INT_PRI_5 | FALLING_EDGE_INT | EXT_INT_ENABLE);
INTClearFlag(INT_INT4);
    
PPSInput(4, INT1, RPA3);//PLAY
ConfigINT1(EXT_INT_PRI_4 | FALLING_EDGE_INT | EXT_INT_ENABLE);
INTClearFlag(INT_INT1);  


  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART1, pb_clock, 19200);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX)); 
   PPSOutput(	1	,	RPA0	,	U1TX	);
      //int i;
      DmaChnEnable(0);
     //RPB10 CHIP SELECT FOR DAC
//RPA4  CHIP SELECT FOR RAM   
 // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_fft(&pt_fft));
    }
} // main

// === end  ====================================================== 