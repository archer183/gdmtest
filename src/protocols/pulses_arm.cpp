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

#include "../opentx.h"

uint8_t s_pulses_paused = 0;
uint8_t s_current_protocol[NUM_MODULES] = { MODULES_INIT(255) };
uint32_t failsafeCounter[NUM_MODULES] = { MODULES_INIT(100) };

void setupPulses(unsigned int port)
{
  heartbeat |= (HEART_TIMER_PULSES << port);

#if defined(PCBTARANIS)
  if (port) port = 1;			// Ensure is 0 or 1 only
  uint8_t required_protocol;
  
  if (port == INTERNAL_MODULE) {
    required_protocol = g_model.moduleData[INTERNAL_MODULE].rfProtocol == RF_PROTO_OFF ? PROTO_NONE : PROTO_PXX;
  }
  else {
    switch (g_model.externalModule) {
      case MODULE_TYPE_PPM:
        required_protocol = PROTO_PPM;
        break;
      case MODULE_TYPE_XJT:
      case MODULE_TYPE_DJT:
      	required_protocol = PROTO_PXX;
        break;
      default:
        required_protocol = PROTO_NONE;
        break;
    }
  }
#else
  uint8_t required_protocol = g_model.protocol;
#endif

  if (s_pulses_paused)
    required_protocol = PROTO_NONE;

  if (s_current_protocol[port] != required_protocol) {

    switch (s_current_protocol[port]) { // stop existing protocol hardware
      case PROTO_PXX:
        disable_pxx(port);
        break;
#if defined(DSM2)
      case PROTO_DSM2_LP45:
      case PROTO_DSM2_DSM2:
      case PROTO_DSM2_DSMX:
        disable_dsm2(port);
        break;
#endif
      case PROTO_PPM:
        disable_ppm(port);
        break;
      default:
        disable_no_pulses(port);
        break;
    }

    s_current_protocol[port] = required_protocol;

    switch (required_protocol) { // Start new protocol hardware here
      case PROTO_PXX:
        init_pxx(port);
        break;
#if defined(DSM2)
      case PROTO_DSM2_LP45:
      case PROTO_DSM2_DSM2:
      case PROTO_DSM2_DSMX:
        init_dsm2(port);
        break;
#endif
      case PROTO_PPM:
        init_ppm(port);
        break;
      default:
        init_no_pulses(port);
        break;
    }
  }

  // Set up output data here
  switch (required_protocol) {
    case PROTO_PXX:
      setupPulsesPXX(port);
      break;
#if defined(DSM2)
    case PROTO_DSM2_LP45:
    case PROTO_DSM2_DSM2:
    case PROTO_DSM2_DSMX:
      setupPulsesDsm2(6);
      break;
#endif
    case PROTO_PPM:
      setupPulsesPPM(port);
      break ;
    default:
      break;
  }
}
