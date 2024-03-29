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

/*
 *
 * ============================================================
 * Templates file
 *
 * eccpm
 * crow
 * throttle cut
 * flaperon
 * elevon
 * v-tail
 * throttle hold
 * Aileron Differential
 * Spoilers
 * Snap Roll
 * ELE->Flap
 * Flap->ELE
 *
 */

#include "opentx.h"

MixData* setDest(uint8_t dch, uint8_t src, bool clear=false)
{
  uint8_t i = 0;
  MixData * mix;

  while (1) {
    mix = mixAddress(i);
    if (mix->srcRaw && mix->destCh <= dch) {
      if (clear && mix->destCh == dch)
        deleteExpoMix(0, i);
      else {
        if (++i==MAX_MIXERS) {
          // TODO should return null pointer but needs to be tested then
          mix = mixAddress(0);
          break;
        }
      }
    }
    else {
      break;
    }
  }

  memmove(mix+1, mix, (MAX_MIXERS-(i+1))*sizeof(MixData) );
  memclear(mix, sizeof(MixData));
  mix->destCh = dch;
  mix->srcRaw = src;
  mix->weight = 100;
  return mix;
}

void md_SetWeight(MixData* md, int8_t weight)
{
  u_int8int16_t tmp;
  tmp.word=weight;
  MD_UNION_TO_WEIGHT(tmp,md);
  // MD_SETWEIGHT(md,weight);  doesn't matter here in code cost compiler optimizes this anyway
}

void clearMixes()
{
  memset(g_model.mixData, 0, sizeof(g_model.mixData)); // clear all mixes
  eeDirty(EE_MODEL);
}

void clearCurves()
{
  memclear(g_model.curves, sizeof(g_model.curves) + sizeof(g_model.points)); // clear all curves
}

void setCurve(uint8_t c, const pm_int8_t ar[])
{
  int8_t * cv = curveAddress(c);
  for (uint8_t i=0; i<5; i++) {
    cv[i] = pgm_read_byte(&ar[i]);
  }
}

void setSwitch(uint8_t idx, uint8_t func, int8_t v1, int8_t v2)
{
  CustomSwData *cs = cswAddress(idx-1);
  cs->func = func;
  cs->v1   = v1;
  cs->v2   = v2;
}

const pm_int8_t heli_ar1[] PROGMEM = {-100, 20, 30, 70, 90};
const pm_int8_t heli_ar2[] PROGMEM = {80, 70, 60, 70, 100};
const pm_int8_t heli_ar3[] PROGMEM = {100, 90, 80, 90, 100};
const pm_int8_t heli_ar4[] PROGMEM = {-30, -15, 0, 50, 100};
const pm_int8_t heli_ar5[] PROGMEM = {-100, -50, 0, 50, 100};

void applyTemplate(uint8_t idx)
{
    MixData *md;

    //CC(STK)   -> vSTK
    //ICC(vSTK) -> STK
#define ICC(x) icc[(x)-1]
    uint8_t icc[4] = {0};
    for (uint8_t i=0; i<4; i++) //generate inverse array
      for(uint8_t j=0; j<4; j++) if(CC(i+1)==j+1) icc[j]=i;

    switch (idx) {
      case TMPL_CLEAR_MIXES:
      case TMPL_SIMPLE_4CH:
      case TMPL_HELI_SETUP:
        clearMixes();
        break;
    }

    switch (idx) {
      // Simple 4-Ch
      case TMPL_SIMPLE_4CH:
        setDest(ICC(STK_RUD), MIXSRC_Rud);
        setDest(ICC(STK_ELE), MIXSRC_Ele);
        setDest(ICC(STK_THR), MIXSRC_Thr);
        setDest(ICC(STK_AIL), MIXSRC_Ail);
        break;

      // Sticky-T-Cut
      case TMPL_STI_THR_CUT:
        md=setDest(ICC(STK_THR), MIXSRC_MAX); md_SetWeight(md, -100);  md->swtch=SWSRC_SWC;  md->mltpx=MLTPX_REP;
        md=setDest(13, MIXSRC_CH14); // md->weight= 100; done by setDest anyway
        md=setDest(13, MIXSRC_MAX); md_SetWeight(md, -100);  md->swtch=SWSRC_SWB;  md->mltpx=MLTPX_REP;
        md=setDest(13, MIXSRC_MAX); /* md->weight= 100;*/  md->swtch=SWSRC_THR;  md->mltpx=MLTPX_REP;
        setSwitch(11, CS_VNEG, STK_THR, -99);
        setSwitch(12, CS_VPOS, MIXSRC_CH14, 0);
        break;

      // V-Tail
      case TMPL_V_TAIL:
        setDest(ICC(STK_RUD), MIXSRC_Rud, true);
        md=setDest(ICC(STK_RUD), MIXSRC_Ele); md_SetWeight(md, -100);
        setDest(ICC(STK_ELE), MIXSRC_Rud, true);
        setDest(ICC(STK_ELE), MIXSRC_Ele);
        break;

      // Elevon\\Delta
      case TMPL_ELEVON_DELTA:
        setDest(ICC(STK_ELE), MIXSRC_Ele, true);
        setDest(ICC(STK_ELE), MIXSRC_Ail);
        setDest(ICC(STK_AIL), MIXSRC_Ele, true);
        md=setDest(ICC(STK_AIL), MIXSRC_Ail); md_SetWeight(md, -100);
        break;

      // eCCPM
      case TMPL_ECCPM:
        md=setDest(ICC(STK_ELE), MIXSRC_Ele, true); md->weight= 72;
        md=setDest(ICC(STK_ELE), MIXSRC_Thr);  md->weight= 55;
        md=setDest(ICC(STK_AIL), MIXSRC_Ele, true);  md_SetWeight(md, -36);
        md=setDest(ICC(STK_AIL), MIXSRC_Ail);  md->weight= 62;
        md=setDest(ICC(STK_AIL), MIXSRC_Thr);  md->weight= 55;
        md=setDest(5, MIXSRC_Ele, true);       md_SetWeight(md, -36);
        md=setDest(5, MIXSRC_Ail);             md_SetWeight(md, -62);
        md=setDest(5, MIXSRC_Thr);             md->weight= 55;
        break;

      // Heli Setup
      case TMPL_HELI_SETUP:
        clearCurves();

        //Set up Mixes
        // 3 cyclic channels
        md=setDest(0, MIXSRC_CYC1); // md->weight=100;
        md=setDest(1, MIXSRC_CYC2); // md->weight=100;
        md=setDest(2, MIXSRC_CYC3); // md->weight=100;

        // rudder
        md=setDest(3, MIXSRC_Rud); // md->weight=100;

        // throttle
        md=setDest(4, MIXSRC_Thr); md->swtch=SWSRC_ID0; md->curveMode=MODE_CURVE; md->curveParam=CV(1); md->carryTrim=TRIM_OFF;
        md=setDest(4, MIXSRC_Thr); md->swtch=SWSRC_ID1; md->curveMode=MODE_CURVE; md->curveParam=CV(2); md->carryTrim=TRIM_OFF;
        md=setDest(4, MIXSRC_Thr); md->swtch=SWSRC_ID2; md->curveMode=MODE_CURVE; md->curveParam=CV(3); md->carryTrim=TRIM_OFF;
        md=setDest(4, MIXSRC_MAX); md_SetWeight(md, -100); md->swtch=SWSRC_THR;  md->mltpx=MLTPX_REP;

        // gyro gain
        md=setDest(5, MIXSRC_MAX); md->weight= 30; md->swtch=-SWSRC_GEA;
        md=setDest(5, MIXSRC_MAX); md_SetWeight(md, -30); md->swtch= SWSRC_GEA;
   
        // collective
        md=setDest(10, MIXSRC_Thr); /*md->weight= 100;*/ md->swtch=SWSRC_ID0; md->curveMode=MODE_CURVE; md->curveParam=CV(4); md->carryTrim=TRIM_OFF;
        md=setDest(10, MIXSRC_Thr); /*md->weight= 100;*/ md->swtch=SWSRC_ID1; md->curveMode=MODE_CURVE; md->curveParam=CV(5); md->carryTrim=TRIM_OFF;
        md=setDest(10, MIXSRC_Thr); /*md->weight= 100;*/ md->swtch=SWSRC_ID2; md->curveMode=MODE_CURVE; md->curveParam=CV(6); md->carryTrim=TRIM_OFF;

        g_model.swashR.collectiveSource = MIXSRC_CH11;
        g_model.swashR.type = SWASH_TYPE_120;

        //Set up Curves
        setCurve(CURVE5(1), heli_ar1);
        setCurve(CURVE5(2), heli_ar2);
        setCurve(CURVE5(3), heli_ar3);
        setCurve(CURVE5(4), heli_ar4);
        setCurve(CURVE5(5), heli_ar5);
        setCurve(CURVE5(6), heli_ar5);
        break;

      // Servo Test
      case TMPL_SERVO_TEST:
        md=setDest(15, MIXSRC_SW1, true); md->weight=110; md->mltpx=MLTPX_ADD; md->delayUp = 6; md->delayDown = 6; md->speedUp = 8; md->speedDown = 8;
        setSwitch(1, CS_VNEG, MIXSRC_LAST_CH, 0);
        break;

    default:
        break;

    }

    eeDirty(EE_MODEL);
}
