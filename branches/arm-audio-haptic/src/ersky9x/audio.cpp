/*
 * Authors (alphabetical order)
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny <shadow@privy.de>
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * open9x is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "open9x.h"

//

void start_sound( void ) ;
void buzzer_on( void ) ;
void buzzer_off( void ) ;
void buzzer_sound( uint8_t time ) ;
void start_timer1( void ) ;
void init_dac( void ) ;
extern "C" void DAC_IRQHandler( void ) ;
void disp_mem( register uint32_t address ) ;
void end_sound( void ) ;
void tone_start( register uint32_t time ) ;
void tone_stop( void ) ;
void init_twi( void ) ;
void set_volume( register uint8_t volume ) ;
extern "C" void TWI0_IRQHandler (void) ;
void audioDefevent( uint8_t e ) ;

extern uint32_t Master_frequency ; // TODO in a .h?
volatile uint8_t Buzzer_count ;

struct t_sound_globals
{
	uint32_t Next_freq ;
	volatile uint32_t Sound_time ;
	uint32_t Frequency ;
	volatile uint32_t Tone_timer ;		// Modified in interrupt routine
	volatile uint8_t Tone_ms_timer ;
	uint32_t Frequency_increment ;
	uint32_t Next_frequency_increment ;
} Sound_g ;


// Must NOT be in flash, PDC needs a RAM source.
uint16_t Sine_values[] =
{
2048,2173,2298,2422,2545,2666,2784,2899,3011,3119,
3223,3322,3417,3505,3589,3666,3736,3800,3857,3907,
3950,3985,4012,4032,4044,4048,4044,4032,4012,3985,
3950,3907,3857,3800,3736,3666,3589,3505,3417,3322,
3223,3119,3011,2899,2784,2666,2545,2422,2298,2173,
2048,1922,1797,1673,1550,1429,1311,1196,1084, 976,
 872, 773, 678, 590, 506, 429, 359, 295, 238, 188,
 145, 110,  83,  63,  51,  48,  51,  63,  83, 110,
 145, 188, 238, 295, 359, 429, 506, 590, 678, 773,
 872, 976,1084,1196,1311,1429,1550,1673,1797,1922
} ;

// Must NOT be in flash, PDC needs a RAM source.
// We'll use these for higher frequencies
//uint16_t Sine_values64[] =
//{
//2048,2244,2438,2628,2813,2990,3159,3316,
//3462,3594,3710,3811,3895,3961,4009,4038,
//4048,4038,4009,3961,3895,3811,3710,3594,
//3462,3316,3159,2990,2813,2628,2438,2244,
//2048,1851,1657,1467,1282,1105, 936, 779,
// 633, 501, 385, 284, 200, 134,  86,  57,
//  48,  57,  86, 134, 200, 284, 385, 501,
// 633, 779, 936,1105,1282,1467,1657,1851
//} ;

const uint16_t PianoTones[] =
{
  28,   29,   31,   33,   35,   37,   39,   41,   44,   46,
  49,   52,   55,   58,   62,   65,   69,   73,   78,   82,
  87,   92,   98,  104,  110,  117,  123,  131,  139,  147,
 156,  165,  175,  185,  196,  208,  220,  233,  247,  262, // d#, E, F, f#, G, g#, A, a#, B, C(middle)
 277,  294,  311,  330,  349,  370,  392,  415,  440,  466, // c#, D, d#, E, F, f#, G, g#, A, a#
 494,  523,  554,  587,  622,  659,  698,  740,  784,  831, // B, C, c#, D, d#, E, F, f#, G, g#
 880,  932,  988, 1047, 1109, 1175, 1245, 1319, 1397, 1480,
1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637,
2794, 2960, 3136, 3322, 3520 ,3729, 3951, 4186
} ;


// Sound routines

void start_sound()
{
	register Pio *pioptr ;
	
	start_timer1() ;
	init_dac() ;
	init_twi() ;

	pioptr = PIOA ;
#ifdef REVB
	pioptr->PIO_CODR = 0x02000000L ;	// Set bit A25 OFF
	pioptr->PIO_PER = 0x02000000L ;		// Enable bit A25 (Stock buzzer)
	pioptr->PIO_OER = 0x02000000L ;		// Set bit A25 as output
#else
	pioptr->PIO_CODR = 0x00010000L ;	// Set bit A16 OFF
	pioptr->PIO_PER = 0x00010000L ;		// Enable bit A16 (Stock buzzer)
	pioptr->PIO_OER = 0x00010000L ;		// Set bit A16 as output
#endif
}

#ifdef REVB
void buzzer_on()
{
	PIOA->PIO_SODR = 0x02000000L ;	// Set bit A25 ON
}

void buzzer_off()
{
	PIOA->PIO_CODR = 0x02000000L ;	// Set bit A25 ON
}
#else
void buzzer_on()
{
	PIOA->PIO_SODR = 0x00010000L ;	// Set bit A16 ON
}

void buzzer_off()
{
	PIOA->PIO_CODR = 0x00010000L ;	// Set bit A16 ON
}
#endif

void buzzer_sound( uint8_t time )
{
	buzzer_on() ;
	Buzzer_count = time ;
}


void set_frequency( uint32_t frequency )
{
  register Tc *ptc ;
	register uint32_t timer ;

	timer = Master_frequency / (800 * frequency) ;		// MCK/8 and 100 000 Hz
	if ( timer > 65535 )
	{
		timer = 65535 ;		
	}
	if ( timer < 2 )
	{
		timer = 2 ;		
	}
	ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_CHANNEL[1].TC_CCR = TC_CCR0_CLKDIS ;		// Stop clock
	ptc->TC_CHANNEL[1].TC_RC = timer ;			// 100 000 Hz
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
}


// Start TIMER1 at 100000Hz, used for DACC trigger
void start_timer1()
{
  register Tc *ptc ;
	register uint32_t timer ;

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x01000000L ;		// Enable peripheral clock to TC1
  
	timer = Master_frequency / 800000 ;		// MCK/8 and 100 000 Hz
	ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[1].TC_RC = timer ;			// 100 000 Hz
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x0009C001 ;	// 0000 0000 0000 1001 1100 0000 0000 0001
																						// MCK/8, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	Sound_g.Frequency = 1000 ;
}



// Configure DAC1 (or DAC0 for REVB)
// Not sure why PB14 has not be allocated to the DAC, although it is an EXTRA function
// So maybe it is automatically done
void init_dac()
{
	register Dacc *dacptr ;

  PMC->PMC_PCER0 |= 0x40000000L ;		// Enable peripheral clock to DAC
	dacptr = DACC ;
#ifdef REVB
	dacptr->DACC_MR = 0x0B000215L ;			// 0000 1011 0000 0001 0000 0010 0001 0101
#else
	dacptr->DACC_MR = 0x0B010215L ;			// 0000 1011 0000 0001 0000 0010 0001 0101
#endif
#ifdef REVB
	dacptr->DACC_CHER	= 1 ;							// Enable channel 0
#else
	dacptr->DACC_CHER	= 2 ;							// Enable channel 1
#endif
	dacptr->DACC_CDR = 2048 ;						// Half amplitude
// Data for PDC must NOT be in flash, PDC needs a RAM source.
#ifndef SIMU
	dacptr->DACC_TPR = (uint32_t) Sine_values ;
	dacptr->DACC_TNPR = (uint32_t) Sine_values ;
#endif
	dacptr->DACC_TCR = 50 ;		// words, 100 16 bit values
	dacptr->DACC_TNCR = 50 ;	// words, 100 16 bit values
	dacptr->DACC_PTCR = DACC_PTCR_TXTEN ;
	NVIC_EnableIRQ(DACC_IRQn) ;
}

extern "C" void DAC_IRQHandler()
{
// Data for PDC must NOT be in flash, PDC needs a RAM source.
#ifndef SIMU
	DACC->DACC_TNPR = (uint32_t) Sine_values ;
#endif
	DACC->DACC_TNCR = 50 ;	// words, 100 16 bit values
	if ( Sound_g.Tone_timer )
	{
		if ( --Sound_g.Tone_timer == 0 )
		{
			DACC->DACC_IDR = DACC_IDR_ENDTX ;
		}
	}
//	if ( Tone_ms_timer == 0 )
//	{
//		Tone_ms_timer = -1 ;
//		DACC->DACC_IDR = DACC_IDR_ENDTX ;
//	}
}

void end_sound()
{
	DACC->DACC_IDR = DACC_IDR_ENDTX ;
	NVIC_DisableIRQ(DACC_IRQn) ;
	TWI0->TWI_IDR = TWI_IDR_TXCOMP ;
	NVIC_DisableIRQ(TWI0_IRQn) ;
	PMC->PMC_PCER0 &= ~0x00080000L ;		// Disable peripheral clock to TWI0
  PMC->PMC_PCER0 &= ~0x40000000L ;		// Disable peripheral clock to DAC
}

// Called every 5mS from interrupt routine
void sound_5ms()
{
	if ( Sound_g.Tone_ms_timer > 0 )
	{
		Sound_g.Tone_ms_timer -= 1 ;
	}
		
	if ( Sound_g.Tone_ms_timer == 0 )
	{
		if ( Sound_g.Sound_time )
		{
			Sound_g.Tone_ms_timer = ( Sound_g.Sound_time + 4 ) / 5 ;
			if ( Sound_g.Next_freq )		// 0 => silence for time
			{
				Sound_g.Frequency = Sound_g.Next_freq ;
				Sound_g.Frequency_increment = Sound_g.Next_frequency_increment ;
				set_frequency( Sound_g.Frequency ) ;
				tone_start( 0 ) ;
			}
			else
			{
				DACC->DACC_IDR = DACC_IDR_ENDTX ;		// Silence
			}
			Sound_g.Sound_time = 0 ;
		}
		else
		{
			DACC->DACC_IDR = DACC_IDR_ENDTX ;	// Disable interrupt
			Sound_g.Tone_timer = 0 ;	
		}
	}
	else if ( ( Sound_g.Tone_ms_timer & 1 ) == 0 )		// Every 10 mS
	{
		if ( Sound_g.Frequency )
		{
			if ( Sound_g.Frequency_increment )
			{
				Sound_g.Frequency += Sound_g.Frequency_increment ;
				set_frequency( Sound_g.Frequency ) ;
			}
		}
	}
}

// frequency in Hz, time in mS
void playTone( uint32_t frequency, uint32_t time )
{
	Sound_g.Next_frequency_increment = 0 ;
	Sound_g.Next_freq = frequency ;
	Sound_g.Sound_time = time ;
//	set_frequency( frequency ) ;
//	Tone_ms_timer = ( time + 4 ) / 5 ;
//	tone_start( 0 ) ;
}

uint32_t queueTone( uint32_t frequency, uint32_t time, uint32_t frequency_increment )
{
	if ( Sound_g.Sound_time == 0 )
	{
		Sound_g.Next_freq = frequency ;
		Sound_g.Next_frequency_increment = frequency_increment ;
		Sound_g.Sound_time = time ;
		return 1 ;
	}
	return 0 ;	
}

// Time is in milliseconds
void tone_start( register uint32_t time )
{
  PMC->PMC_PCER0 |= 0x40000000L ;		// Enable peripheral clock to DAC
	Sound_g.Tone_timer = Sound_g.Frequency * time / 1000 ;
	DACC->DACC_IER = DACC_IER_ENDTX ;
}

void tone_stop()
{
	DACC->DACC_IDR = DACC_IDR_ENDTX ;	// Disable interrupt
	Sound_g.Tone_timer = 0 ;	
}


// Set up for volume control (TWI0)
// Need PA3 and PA4 set to peripheral A
void init_twi()
{
	register Pio *pioptr ;
	register uint32_t timing ;
  
	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
	/* Configure PIO */
	pioptr = PIOA ;
  pioptr->PIO_ABCDSR[0] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_ABCDSR[1] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_PDR = 0x00000018 ;					// Assign to peripheral
	
	timing = Master_frequency * 5 / 1000000 ;		// 5uS high and low
	timing += 15 - 4 ;
	timing /= 16 ;
	timing |= timing << 8 ;

	TWI0->TWI_CWGR = 0x00040000 | timing ;			// TWI clock set
	TWI0->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS ;		// Master mode enable
	TWI0->TWI_MMR = 0x002F0000 ;		// Device 5E (>>1) and master is writing
	NVIC_EnableIRQ(TWI0_IRQn) ;
	set_volume( 2 ) ;
}

static int16_t Volume_required ;
static const uint8_t Volume_scale[NUM_VOL_LEVELS] = 
{
	 0,  2,  4,   6,   8,  10,  13,  17,  22,  27,  33,  40,
	64, 82, 96, 105, 112, 117, 120, 122, 124, 125, 126, 127 	
} ;

void set_volume( register uint8_t volume )
{
//	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
	if ( volume >= NUM_VOL_LEVELS )
	{
		volume = NUM_VOL_LEVELS - 1 ;		
	}
	volume = Volume_scale[volume] ;

	__disable_irq() ;
	if ( TWI0->TWI_IMR & TWI_IMR_TXCOMP )
	{
		Volume_required = volume ;
	}
	else
	{
		TWI0->TWI_THR = volume ;		// Send data
		TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
		TWI0->TWI_IER = TWI_IER_TXCOMP ;

	}
	__enable_irq() ;
}

extern "C" void TWI0_IRQHandler()
{
	if ( Volume_required >= 0 )
	{
		TWI0->TWI_THR = Volume_required ;		// Send data
		Volume_required = -1 ;
		TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
	}
	else
	{
		TWI0->TWI_IDR = TWI_IDR_TXCOMP ;
	}
}

// --


audioQueue::audioQueue()
{
  toneTimeLeft = 0;
  tonePause = 0;

  t_queueRidx = 0;
  t_queueWidx = 0;
}


#define QUEUE_TONE(tf,ttl,tfi) queueTone(tf * 61 / 2, ttl * 10, tfi * 61 / 2)


// heartbeat is responsibile for issueing the audio tones and general square waves
// it is essentially the life of the class.
// it is called every 10ms
void audioQueue::heartbeat()
{
#if defined(SIMU)
  return;
#endif

  if (toneTimeLeft == 0) {
    if (tonePause == 0) {
      if (t_queueRidx != t_queueWidx) {
        toneFreq = queueToneFreq[t_queueRidx];
        toneTimeLeft = queueToneLength[t_queueRidx];
        toneFreqIncr = queueToneFreqIncr[t_queueRidx];
        tonePause = queueTonePause[t_queueRidx];
        if (!queueToneRepeat[t_queueRidx]--) {
          t_queueRidx = (t_queueRidx + 1) % AUDIO_QUEUE_LENGTH;
        }
      }
    }
  }

  if ((toneFreq > 0) & (toneTimeLeft > 0)) {
    QUEUE_TONE(toneFreq, toneTimeLeft, toneFreqIncr);
  }
  else if (((tone2Freq > 0) & (tone2TimeLeft > 0)) & (tonePause == 0)) {
    //second flow tone here, priority on 1st, pause of 1st not allow to start second
    QUEUE_TONE(tone2Freq, tone2TimeLeft, 0);
  }

  if (toneTimeLeft > 0) {
    toneTimeLeft--; //time gets counted down
    toneFreq += toneFreqIncr;
  }
  else {
    if (tonePause > 0) {
      // SPEAKER_OFF;
      tonePause--; //time gets counted down
    }
  }

  if (tone2TimeLeft > 0) {
    tone2TimeLeft--; //time gets counted down
  }
  else {
    if (toneTimeLeft == 0) {
      //SPEAKER_OFF;
    }
    if (tone2Pause > 0) {
      tone2Pause--; //time gets counted down
    }
  }
}

inline uint8_t audioQueue::getToneLength(uint8_t tLen)
{
  uint8_t result = tLen; // default
  if (g_eeGeneral.beeperLength < 0) {
    result /= (1-g_eeGeneral.beeperLength);
  }
  if (g_eeGeneral.beeperLength > 0) {
    result *= (1+g_eeGeneral.beeperLength);
  }
  return result;
}

void audioQueue::play(uint8_t tFreq, uint8_t tLen, uint8_t tPause,
    uint8_t tFlags, int8_t tFreqIncr)
{
  if (tFlags & PLAY_SOUND_VARIO) {
    tone2Freq = tFreq;
    tone2TimeLeft = tLen;
    tone2Pause = tPause;
  }
  else {
    tFreq += g_eeGeneral.speakerPitch + BEEP_OFFSET; // add pitch compensator
    tLen = getToneLength(tLen);
    if (tFlags & PLAY_NOW || (!busy() && empty())) {
      toneFreq = tFreq;
      toneTimeLeft = tLen;
      tonePause = tPause;
      toneFreqIncr = tFreqIncr;
      t_queueWidx = t_queueRidx;
    }
    else {
      tFlags++;
    }

    tFlags &= 0x0f;
    if (tFlags) {
      uint8_t next_queueWidx = (t_queueWidx + 1) % AUDIO_QUEUE_LENGTH;
      if (next_queueWidx != t_queueRidx) {
        queueToneFreq[t_queueWidx] = tFreq;
        queueToneLength[t_queueWidx] = tLen;
        queueTonePause[t_queueWidx] = tPause;
        queueToneRepeat[t_queueWidx] = tFlags - 1;
        queueToneFreqIncr[t_queueWidx] = tFreqIncr;
        t_queueWidx = next_queueWidx;
      }
    }
  }
}

void audioQueue::event(uint8_t e, uint8_t f)
{
  if (g_eeGeneral.flashBeep && (e <= AU_ERROR || e >= AU_WARNING1))
    g_LightOffCounter = FLASH_DURATION;

  if (g_eeGeneral.beeperMode>0 || (g_eeGeneral.beeperMode==0 && e>=AU_WARNING1) || (g_eeGeneral.beeperMode>=-1 && e<=AU_ERROR)) {
    if (e < AU_FRSKY_FIRST || empty()) {
      switch (e) {
        // inactivity timer alert
        case AU_INACTIVITY:
          play(70, 10, 2, 2|PLAY_NOW);
          break;
        // low battery in tx
        case AU_TX_BATTERY_LOW:
          if (empty()) {
            play(60, 20, 3, 2, 1);
            play(80, 20, 3, 2, -1);
          }
          break;
        // error
        case AU_ERROR:
          play(BEEP_DEFAULT_FREQ, 40, 1, PLAY_NOW);
          break;
        // keypad up (seems to be used when going left/right through system menu options. 0-100 scales etc)
        case AU_KEYPAD_UP:
          play(BEEP_KEY_UP_FREQ, 10, 1, PLAY_NOW);
          break;
        // keypad down (seems to be used when going left/right through system menu options. 0-100 scales etc)
        case AU_KEYPAD_DOWN:
          play(BEEP_KEY_DOWN_FREQ, 10, 1, PLAY_NOW);
          break;
        // menu display (also used by a few generic beeps)
        case AU_MENUS:
          play(BEEP_DEFAULT_FREQ, 10, 2, PLAY_NOW);
          break;
        // trim move
        case AU_TRIM_MOVE:
          play(f, 6, 1, PLAY_NOW);
          break;
        // trim center
        case AU_TRIM_MIDDLE:
          play(BEEP_DEFAULT_FREQ, 10, 2, PLAY_NOW);
          break;
        // warning one
        case AU_WARNING1:
          play(BEEP_DEFAULT_FREQ, 10, 1, PLAY_NOW);
          break;
        // warning two
        case AU_WARNING2:
          play(BEEP_DEFAULT_FREQ, 20, 1, PLAY_NOW);
          break;
        // warning three
        case AU_WARNING3:
          play(BEEP_DEFAULT_FREQ, 30, 1, PLAY_NOW);
          break;
        // startup tune
        case AU_TADA:
          play(50, 10, 5);
          play(90, 10, 5);
          play(110, 5, 4, 2);
          break;
        // pot/stick center
        case AU_POT_STICK_MIDDLE:
          play(BEEP_DEFAULT_FREQ + 50, 10, 1, PLAY_NOW);
          break;
        // mix warning 1
        case AU_MIX_WARNING_1:
          play(BEEP_DEFAULT_FREQ + 50, 6, 0, PLAY_NOW);
          break;
        // mix warning 2
        case AU_MIX_WARNING_2:
          play(BEEP_DEFAULT_FREQ + 52, 6, 0, PLAY_NOW);
          break;
        // mix warning 3
        case AU_MIX_WARNING_3:
          play(BEEP_DEFAULT_FREQ + 54, 6, 0, PLAY_NOW);
          break;
        // time 30 seconds left
        case AU_TIMER_30:
          play(BEEP_DEFAULT_FREQ + 50, 15, 3, 2|PLAY_NOW);
          break;
        // time 20 seconds left
        case AU_TIMER_20:
          play(BEEP_DEFAULT_FREQ + 50, 15, 3, 1|PLAY_NOW);
          break;
        // time 10 seconds left
        case AU_TIMER_10:
          play(BEEP_DEFAULT_FREQ + 50, 15, 3, PLAY_NOW);
          break;
        // time <3 seconds left
        case AU_TIMER_LT3:
          play(BEEP_DEFAULT_FREQ + 50, 15, 3, PLAY_NOW);
          break;
        case AU_FRSKY_WARN1:
          play(BEEP_DEFAULT_FREQ+20,15,5,2);
          break;
        case AU_FRSKY_WARN2:
          play(BEEP_DEFAULT_FREQ+30,15,5,2);
          break;
        case AU_FRSKY_CHEEP:
          play(BEEP_DEFAULT_FREQ+30,10,2,2,2);
          break;
        case AU_FRSKY_RING:
          play(BEEP_DEFAULT_FREQ+25,5,2,10);
          play(BEEP_DEFAULT_FREQ+25,5,10,1);
          play(BEEP_DEFAULT_FREQ+25,5,2,10);
          break;
        case AU_FRSKY_SCIFI:
          play(80,10,3,2,-1);
          play(60,10,3,2,1);
          play(70,10,1,0);
          break;
        case AU_FRSKY_ROBOT:
          play(70,5,1,1);
          play(50,15,2,1);
          play(80,15,2,1);
          break;
        case AU_FRSKY_CHIRP:
          play(BEEP_DEFAULT_FREQ+40,5,1,2);
          play(BEEP_DEFAULT_FREQ+54,5,1,3);
          break;
        case AU_FRSKY_TADA:
          play(50,5,5);
          play(90,5,5);
          play(110,3,4,2);
          break;
        case AU_FRSKY_CRICKET:
          play(80,5,10,3);
          play(80,5,20,1);
          play(80,5,10,3);
          break;
        case AU_FRSKY_SIREN:
          play(10,20,5,2,1);
          break;
        case AU_FRSKY_ALARMC:
          play(50,4,10,2);
          play(70,8,20,1);
          play(50,8,10,2);
          play(70,4,20,1);
          break;
        case AU_FRSKY_RATATA:
          play(BEEP_DEFAULT_FREQ+50,5,10,10);
          break;
        case AU_FRSKY_TICK:
          play(BEEP_DEFAULT_FREQ+50,5,50,2);
          break;
        default:
          break;
      }
    }
  }
}

void audioDefevent(uint8_t e)
{
#ifdef HAPTIC
  haptic.event(e); //do this before audio to help sync timings
#endif	
  audio.event(e, BEEP_DEFAULT_FREQ);
}
