/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
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

#ifndef audio_h
#define audio_h

#include "FatFs/ff.h"

#define AUDIO_FILENAME_MAXLEN (40)
#define AUDIO_QUEUE_LENGTH    (20)
#define BEEP_DEFAULT_FREQ     (140)
#define BEEP_OFFSET           (10)
#define BEEP_KEY_UP_FREQ      (BEEP_DEFAULT_FREQ+10)
#define BEEP_KEY_DOWN_FREQ    (BEEP_DEFAULT_FREQ-10)

extern uint16_t Sine_values[];

class ToneFragment {
  public:
    uint16_t freq;
    uint8_t duration;
    uint8_t pause;
    uint8_t repeat;
    int8_t  freqIncr;
    uint8_t id;
};

class AudioFragment : public ToneFragment {
  public:
    char file[AUDIO_FILENAME_MAXLEN+1];
};

extern "C" void DAC_IRQHandler();

#define AUDIO_NOT_STARTED  0
#define AUDIO_SLEEPING     1
#define AUDIO_RESUMING     2
#define AUDIO_PLAYING_TONE 3
#define AUDIO_PLAYING_WAV  4

extern bool playingBackground;

class AudioContext {
  public:
    AudioFragment fragment;
#if defined(SDCARD)
    FIL wavFile;
#endif
    uint8_t pcmCodec;
    uint32_t pcmFreq;
    uint32_t wavSize;
};

class AudioQueue {

  friend void audioTask(void* pdata);
  friend void DAC_IRQHandler();

  public:

    AudioQueue();

    void start();

    void play(uint16_t tFreq, uint8_t tLen, uint8_t tPause, uint8_t tFlags=0, int8_t tFreqIncr=0);

    void playFile(const char *filename, uint8_t flags=0, uint8_t id=0);

    void stopPlay(uint8_t id);

    void pause(uint8_t tLen);

    void stopSD();

    bool isPlaying(uint8_t id);

    bool started()
    {
      return (state != AUDIO_NOT_STARTED);
    }

    bool busy()
    {
#ifdef SIMU
      return false;
#else
      return (state != AUDIO_SLEEPING);
#endif
    }

    bool empty()
    {
      return ridx == widx;
    }

    void reset();

  protected:

    void wakeup();

    void sdWakeup(AudioContext & context);

    volatile uint8_t state;
    uint8_t ridx;
    uint8_t widx;
    int8_t prioIdx;

    AudioFragment fragments[AUDIO_QUEUE_LENGTH];

    AudioContext currentContext;
    AudioContext backgroundContext; // for background music / vario
};

extern AudioQueue audioQueue;

void codecsInit();
void audioEvent(uint8_t e, uint8_t f=BEEP_DEFAULT_FREQ);
void audioStart();

#if defined(AUDIO) && defined(BUZZER)
  #define AUDIO_BUZZER(a, b)  do { a; b; } while(0)
#elif defined(AUDIO)
  #define AUDIO_BUZZER(a, b)  a
#else
  #define AUDIO_BUZZER(a, b)  b
#endif

#if defined(VOICE)
  #define AUDIO_ERROR_MESSAGE(e) audioEvent(e)
  #define AUDIO_TIMER_MINUTE(t)  playDuration(t, 0)
#else
  #define AUDIO_ERROR_MESSAGE(e) audioEvent(AU_ERROR)
  #define AUDIO_TIMER_MINUTE(t)  audioDefevent(AU_WARNING1)
#endif

#define AUDIO_TADA()             audioEvent(AU_TADA)
#define AUDIO_KEYPAD_UP()        AUDIO_BUZZER(audioEvent(AU_KEYPAD_UP), beep(0))
#define AUDIO_KEYPAD_DOWN()      AUDIO_BUZZER(audioEvent(AU_KEYPAD_DOWN), beep(0))
#define AUDIO_MENUS()            AUDIO_BUZZER(audioEvent(AU_MENUS), beep(0))
#define AUDIO_WARNING1()         AUDIO_BUZZER(audioEvent(AU_WARNING1), beep(3))
#define AUDIO_WARNING2()         AUDIO_BUZZER(audioEvent(AU_WARNING2), beep(2))
#define AUDIO_TX_BATTERY_LOW()   AUDIO_BUZZER(audioEvent(AU_TX_BATTERY_LOW), beep(4))
#if defined(PCBSKY9X)
  #define AUDIO_TX_MAH_HIGH()    audioEvent(AU_TX_MAH_HIGH)
  #define AUDIO_TX_TEMP_HIGH()   audioEvent(AU_TX_TEMP_HIGH)
#endif
#define AUDIO_ERROR()            AUDIO_BUZZER(audioEvent(AU_ERROR), beep(4))
#define AUDIO_TIMER_30()         AUDIO_BUZZER(audioEvent(AU_TIMER_30), { beepAgain=2; beep(2); })
#define AUDIO_TIMER_20()         AUDIO_BUZZER(audioEvent(AU_TIMER_20), { beepAgain=1; beep(2); })
#define AUDIO_TIMER_LT10(x)      AUDIO_BUZZER(audioEvent(AU_TIMER_LT10), beep(2))
#define AUDIO_INACTIVITY()       AUDIO_BUZZER(audioEvent(AU_INACTIVITY), beep(3))
#define AUDIO_MIX_WARNING(x)     AUDIO_BUZZER(audioEvent(AU_MIX_WARNING_1+x-1), beep(1))
#define AUDIO_POT_STICK_MIDDLE() AUDIO_BUZZER(audioEvent(AU_POT_STICK_MIDDLE), beep(2))
#define AUDIO_VARIO_UP()         audioEvent(AU_KEYPAD_UP)
#define AUDIO_VARIO_DOWN()       audioEvent(AU_KEYPAD_DOWN)
#define AUDIO_TRIM_MIDDLE(f)     AUDIO_BUZZER(audioEvent(AU_TRIM_MIDDLE, f), beep(2))
#define AUDIO_TRIM_END(f)        AUDIO_BUZZER(audioEvent(AU_TRIM_END, f), beep(2))
#define AUDIO_TRIM(event, f)     AUDIO_BUZZER(audioEvent(AU_TRIM_MOVE, f), { if (!IS_KEY_FIRST(event)) warble = true; beep(1); })
#define AUDIO_PLAY(p)            audioEvent(p)
#define AUDIO_VARIO(f, t)        audioQueue.play(f, t, 0, PLAY_BACKGROUND)

#if defined(PCBTARANIS)
#define AUDIO_A1_ORANGE()        audioEvent(AU_A1_ORANGE)
#define AUDIO_A1_RED()           audioEvent(AU_A1_RED)
#define AUDIO_A2_ORANGE()        audioEvent(AU_A2_ORANGE)
#define AUDIO_A2_RED()           audioEvent(AU_A2_RED)
#define AUDIO_RSSI_ORANGE()      audioEvent(AU_RSSI_ORANGE)
#define AUDIO_RSSI_RED()         audioEvent(AU_RSSI_RED)
#define AUDIO_SWR_RED()          audioEvent(AU_SWR_RED)
#endif

#define AUDIO_HEARTBEAT()

#define SYSTEM_AUDIO_CATEGORY 0
#define MODEL_AUDIO_CATEGORY  1
#define PHASE_AUDIO_CATEGORY  2
#define MIXER_AUDIO_CATEGORY  3

#define AUDIO_EVENT_OFF       0
#define AUDIO_EVENT_ON        1
#define AUDIO_EVENT_BG        2

extern void pushPrompt(uint16_t prompt, uint8_t id=0);

#define I18N_PLAY_FUNCTION(lng, x, ...) void lng ## _ ## x(__VA_ARGS__, uint8_t id)
#define PLAY_FUNCTION(x, ...)    void x(__VA_ARGS__, uint8_t id)
#define PUSH_NUMBER_PROMPT(p)    pushPrompt((p), id)
#define PLAY_NUMBER(n, u, a)     playNumber((n), (u), (a), id)
#define PLAY_DURATION(d)         playDuration((d), id)
#define IS_PLAYING(id)           audioQueue.isPlaying((id))
#define PLAY_VALUE(v, id)        playValue((v), (id))
#define PLAY_FILE(f, flags, id)  audioQueue.playFile((f), (flags), (id))
#define STOP_PLAY(id)            audioQueue.stopPlay((id))
#define AUDIO_RESET()            audioQueue.reset()

#if defined(SDCARD)
  #define PLAY_PHASE_OFF(phase) do { char filename[AUDIO_FILENAME_MAXLEN+1]; if (isAudioFileAvailable((PHASE_AUDIO_CATEGORY << 24) + (phase << 16) + AUDIO_EVENT_OFF, filename)) audioQueue.playFile(filename); } while (0)
  #define PLAY_PHASE_ON(phase)  do { char filename[AUDIO_FILENAME_MAXLEN+1]; if (isAudioFileAvailable((PHASE_AUDIO_CATEGORY << 24) + (phase << 16) + AUDIO_EVENT_ON, filename)) audioQueue.playFile(filename); } while (0)
#else
  #define PLAY_PHASE_OFF(phase)
  #define PLAY_PHASE_ON(phase)
#endif

extern void refreshSystemAudioFiles();
extern void refreshModelAudioFiles();
extern bool isAudioFileAvailable(uint32_t i, char * filename);

// TODO not public, everything could be inside a function in audio_arm.cpp
extern OS_TCID audioTimer;
extern OS_FlagID audioFlag;
extern OS_MutexID audioMutex;

#endif
