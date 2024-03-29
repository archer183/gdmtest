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

// NON ZERO TERMINATED STRINGS
#define LEN_OFFON        "\003"
#define TR_OFFON         "OFF""ON\0"

#define LEN_MMMINV       "\003"
#define TR_MMMINV        "---""INV"

#define LEN_NCHANNELS    "\004"
#define TR_NCHANNELS     "\0014CH\0016CH\0018CH10CH12CH14CH16CH"

#define LEN_VBEEPMODE    "\005"
#define TR_VBEEPMODE     "Aucun""Alarm""NoKey""Tout\0"

#define LEN_VBEEPLEN     "\005"
#define TR_VBEEPLEN      "0====""=0===""==0==""===0=""====0"

#define LEN_VRENAVIG     "\003"
#define TR_VRENAVIG      "NonREaREb"

#define LEN_VBLMODE      "\004"
#define TR_VBLMODE       "OFF\0""Btns""Stks""Tout""ON\0"

#define LEN_TRNMODE      "\003"
#define TR_TRNMODE       "OFF"" +="" :="

#define LEN_TRNCHN       "\003"
#define TR_TRNCHN        "CH1CH2CH3CH4"

#define LEN_DATETIME     "\005"
#define TR_DATETIME      "DATE:""HEURE"

#define LEN_VLCD         "\006"
#define TR_VLCD          "NormalOptrex"

#define LEN_COUNTRYCODES TR("\002", "\006")
#define TR_COUNTRYCODES  TR("US""JP""EU", "USA\0  ""Japon\0""Europe")

#define LEN_VTRIMINC     TR("\006","\013")
#define TR_VTRIMINC      TR("Expo\0 ""ExFin\0""Fin\0  ""Moyen\0""Gros\0 ","Exponentiel""Extra Fin\0 ""Fin\0       ""Moyen\0     ""Grossier\0  ")

#define LEN_RETA123      "\001"

#if defined(PCBGRUVIN9X)
  #if ROTARY_ENCODERS > 2
    #define TR_RETA123       "DPGA123abcd"
  #else
    #define TR_RETA123       "DPGA123ab"
  #endif
#elif defined(PCBTARANIS)
  #define TR_RETA123         "DPGA12LR"
#else  
  #define TR_RETA123         "DPGA123"
#endif

#define LEN_VPROTOS          "\006"

#if defined(PXX)
  #define TR_PXX         "PXX\0  "
#elif defined(DSM2) || defined(IRPROTOS)
  #define TR_PXX         "[PXX]\0"
#else
  #define TR_PXX
#endif

#if defined(DSM2)
  #define TR_DSM2        "LP45\0 ""DSM2\0 ""DSMX\0 "
#elif defined(IRPROTOS)
  #define TR_DSM2        "[LP45]""[DSM2]""[DSMX]"
#else
  #define TR_DSM2
#endif

#if defined(IRPROTOS)
  #define TR_IRPROTOS    "SILV  TRAC09PICZ  SWIFT\0"
#else
  #define TR_IRPROTOS
#endif

#if defined(CPUARM)
  #define TR_XPPM
#else
  #define TR_XPPM        "PPM16\0""PPMsim"
#endif

#define TR_VPROTOS       "PPM\0  " TR_XPPM TR_PXX TR_DSM2 TR_IRPROTOS

#define LEN_POSNEG       "\003"
#define TR_POSNEG        "POS""NEG"

#define LEN_VCURVEFUNC   "\003"
#define TR_VCURVEFUNC    "---""x>0""x<0""|x|""f>0""f<0""|f|"

#define LEN_VMLTPX       TR("\010","\013")
#define TR_VMLTPX        TR("Ajoute\0 ""Multipl.""Remplace","Additionner""Multiplier\0""Remplacer\0 ")

#define LEN_VMLTPX2      "\002"
#define TR_VMLTPX2       "+=""*="":="

#define LEN_VMIXTRIMS    "\003"
#define TR_VMIXTRIMS     "OFF""ON\0""Dir""Prf""Gaz""Ail"

#define LEN_VCSWFUNC     "\005"
#define TR_VCSWFUNC      "---\0 ""a{x\0 ""a>x\0 ""a<x\0 ""|a|>x""|a|<x""AND\0 ""OR\0  ""XOR\0 ""a=b\0 ""a>b\0 ""a<b\0 ""d}x\0 ""|d|}x""TIM\0"

#define LEN_VFSWFUNC     "\015"

#if defined(VARIO)
  #define TR_VVARIO         "Vario\0       "
#else
  #define TR_VVARIO         "[Vario]\0     "
#endif

#if defined(AUDIO)
  #define TR_SOUND         "Jouer son\0   "
#else
  #define TR_SOUND         "Bip\0         "
#endif

#if defined(PCBTARANIS)
  #define TR_HAPTIC
#elif defined(HAPTIC)
  #define TR_HAPTIC        "Vibreur\0     "
#else
  #define TR_HAPTIC        "[Vibreur]\0   "
#endif

#if defined(VOICE)
  #if defined(PCBSKY9X)
    #define TR_PLAY_TRACK    "Jouer\0       "
  #else
    #define TR_PLAY_TRACK    "Jouer fich\0  "
  #endif
  #define TR_PLAY_BOTH       "Jouer les 2\0 "
  #define TR_PLAY_VALUE      "Dire valeur\0 "
#else
  #define TR_PLAY_TRACK    "[Jouer fich.]"
  #define TR_PLAY_BOTH     "[Jouer les 2]"
  #define TR_PLAY_VALUE    "[Dire valeur]"
#endif

#define TR_CFN_VOLUME    "Volume\0      "
#define TR_CFN_BG_MUSIC  "Musique\0     ""Pause Musique"

#if defined(SDCARD)
  #define TR_SDCLOGS     "Logs SD\0     "
#else
  #define TR_SDCLOGS     "[Logs SD]\0   "
#endif

#ifdef GVARS
  #define TR_CFN_ADJUST_GVAR  "Ajuster\0     "
#else
  #define TR_CFN_ADJUST_GVAR
#endif

#ifdef DEBUG
  #define TR_CFN_TEST         "Test\0        "
#else
  #define TR_CFN_TEST
#endif

#if defined(CPUARM)
  #define TR_VFSWFUNC      "S\200cur.\0      ""Ecolage\0     ""Trim instant." TR_SOUND TR_HAPTIC "Remise \202 0\0  " TR_VVARIO TR_PLAY_TRACK TR_PLAY_VALUE TR_SDCLOGS TR_CFN_VOLUME "R\200tro\200cl.\0   " TR_CFN_BG_MUSIC TR_CFN_ADJUST_GVAR TR_CFN_TEST
#elif defined(PCBGRUVIN9X)
  #define TR_VFSWFUNC      "S\200cur.\0      ""Ecolage\0     ""Trim instant." TR_SOUND TR_HAPTIC "Remise \202 0\0  " TR_VVARIO TR_PLAY_TRACK TR_PLAY_BOTH TR_PLAY_VALUE TR_SDCLOGS "R\200tro\200cl.\0   " TR_CFN_ADJUST_GVAR TR_CFN_TEST
#else
  #define TR_VFSWFUNC      "S\200cur.\0      ""Ecolage\0     ""Trim instant." TR_SOUND TR_HAPTIC "Remise \202 0\0  " TR_VVARIO TR_PLAY_TRACK TR_PLAY_BOTH TR_PLAY_VALUE "R\200tro\200cl.\0   " TR_CFN_ADJUST_GVAR TR_CFN_TEST
#endif

#define LEN_VFSWRESET    "\006"

#if defined(FRSKY)
  #define TR_FSW_RESET_TELEM   "T\200l\200m."
#else
  #define TR_FSW_RESET_TELEM   
#endif

#if ROTARY_ENCODERS == 2
  #define TR_FSW_RESET_ROTENC  TR("ERa\0 ""ERb\0 ", "EncRot A\0""EncRot B\0")
#elif ROTARY_ENCODERS == 1
  #define TR_FSW_RESET_ROTENC  TR("Enc.r", "EncRot\0  ")
#else
  #define TR_FSW_RESET_ROTENC
#endif

#define TR_VFSWRESET      "Timer1""Timer2""Timers" TR_FSW_RESET_TELEM TR_FSW_RESET_ROTENC

#define LEN_FUNCSOUNDS   "\006"
#define TR_FUNCSOUNDS    "Beep1\0""Beep2\0""Beep3\0""Warn1\0""Warn2\0""Cheep\0""Ring\0 ""SciFi\0""Robot\0""Chirp\0""Tada\0 ""Crickt""Siren\0""AlmClk""Ratata""Tick\0 "

#define LEN_VTELEMCHNS   "\004"
#if defined(PCBTARANIS)
  #define TR_RSSI_0            "SWR\0"
  #define TR_RSSI_1            "RSSI"
#else
  #define TR_RSSI_0            "Tx\0 "
  #define TR_RSSI_1            "Rx\0 "
#endif
#define TR_VTELEMCHNS    "---\0""Batt""Chr1""Chr2" TR_RSSI_0 TR_RSSI_1 "A1\0 ""A2\0 ""Alt\0""Rpm\0""Carb""T1\0 ""T2\0 ""Vit\0""Dist""AltG""Elem""Velm""Vfas""Cour""Cnsm""Puis""AccX""AccY""AccZ""Cap\0""VitV""A1-\0""A2-\0""Alt-""Alt+""Rpm+""T1+\0""T2+\0""Vit+""Dst+""Cur+""Pwr+""Acc\0""Time"

#if defined(CPUARM)
  #define LEN_VUNITSSYSTEM     TR("\006", "\012")
  #define TR_VUNITSSYSTEM      TR("M\200tr.\0""Imp\200r.", "M\200triques\0""Imp\200riales")
  #define LEN_VTELEMUNIT_NORM  "\003"
  #define TR_VTELEMUNIT_NORM   "v\0 ""A\0 ""m/s""-\0 ""kmh""m\0 ""@\0 ""%\0 ""mA\0""mAh""W\0 "
  #define LEN_VTELEMUNIT_IMP   "\003"
  #define TR_VTELEMUNIT_IMP    "v\0 ""A\0 ""m/s""-\0 ""kts""ft\0""@\0 ""%\0 ""mA\0""mAh""W\0 "
#else
  #if defined(IMPERIAL_UNITS)
    #define LENGTH_UNIT        "ft\0"
    #define SPEED_UNIT         "kts"
  #else
    #define LENGTH_UNIT        "m\0 "
    #define SPEED_UNIT         "kmh"
  #endif
  #define LEN_VTELEMUNIT       "\003"
  #define TR_VTELEMUNIT        "v\0 ""A\0 ""m/s""-\0 " SPEED_UNIT LENGTH_UNIT "@\0 ""%\0 ""mA\0""mAh""W\0 "
#endif

#define STR_V            (STR_VTELEMUNIT+1)
#define STR_A            (STR_VTELEMUNIT+4)

#define LEN_VALARM       "\004"
#define TR_VALARM        "----""Jaun""Oran""Roug"

#define LEN_VALARMFN     "\001"
#define TR_VALARMFN      "<>"

#define LEN_VTELPROTO    "\007"
#define TR_VTELPROTO     "Aucun  ""Hub\0   ""WSHHigh""Halcyon"

#define LEN_VOLTSRC      "\003"
#define TR_VOLTSRC       "---""A1\0""A2\0""FAS""Cel"

#define LEN_VARIOSRC     "\005"
#define TR_VARIOSRC      "Alti\0""Alti+""Vario""A1\0  ""A2\0"

#define LEN_VSCREEN      TR("\004","\007")
#define TR_VSCREEN       TR("Val.""Bars","Valeurs""Barres\0")

#define LEN_GPSFORMAT    "\004"
#define TR_GPSFORMAT     "DMS\0NMEA"

#define LEN2_VTEMPLATES  15
#define LEN_VTEMPLATES   "\017"
#define TR_VTEMPLATES    "Suppr Mixages\0 ""4 Voies simple\0""Coupure Gaz\0   ""Empennage V\0   ""Elevon\\Delta\0  ""eCCPM\0         ""Conf. H\200lico\0  ""Test Servo\0    "

#define LEN_VSWASHTYPE   "\004"
#define TR_VSWASHTYPE    "--- ""120 ""120X""140 ""90\0"

#define LEN_VKEYS        "\005"
#define TR_VKEYS         TR(" Menu"" Exit""  Bas"" Haut""Droit""Gauch", " Menu"" Exit""Enter"" Page"" Plus""Minus")

#define LEN_VRENCODERS   "\003"
#define TR_VRENCODERS    "REa""REb"

#define LEN_VSWITCHES    "\003"
#define LEN_VSRCRAW      "\004"

#if defined(PCBTARANIS)
  #define TR_POTS_VSRCRAW      "S1\0 ""S2\0 ""LS\0 ""RS\0 "
  #define TR_SW_VSRCRAW        "SA\0 ""SB\0 ""SC\0 ""SD\0 ""SE\0 ""SF\0 ""SG\0 ""SH\0 "
#elif defined(EXTRA_3POS)
  #define TR_POTS_VSRCRAW      "P1\0 ""P2\0 "
  #define TR_SW_VSRCRAW        "3P1\0""3P2\0"
  #define TR_9X_3POS_SWITCHES  "ID0""ID1""ID2""ID3""ID4""ID5"
#else
  #define TR_POTS_VSRCRAW      "P1\0 ""P2\0 ""P3\0 "
  #define TR_SW_VSRCRAW        "3POS"
  #define TR_9X_3POS_SWITCHES  "ID0""ID1""ID2"
#endif

#if defined(CPUARM)
  #define TR_CUSTOMSW          "IP1""IP2""IP3""IP4""IP5""IP6""IP7""IP8""IP9""IPA""IPB""IPC""IPD""IPE""IPF""IPG""IPH""IPI""IPJ""IPK""IPL""IPM""IPN""IPO""IPP""IPQ""IPR""IPS""IPT""IPU""IPV""IPW"
#elif defined(PCBGRUVIN9X) || defined(CPUM2561) || defined(CPUM128)
  #define TR_CUSTOMSW          "IP1""IP2""IP3""IP4""IP5""IP6""IP7""IP8""IP9""IPA""IPB""IPC""IPD""IPE""IPF"
#else
  #define TR_CUSTOMSW          "IP1""IP2""IP3""IP4""IP5""IP6""IP7""IP8""IP9""IPA""IPB""IPC"
#endif

#if defined(PCBTARANIS)
  #define TR_VSWITCHES         "SA\300""SA-""SA\301""SB\300""SB-""SB\301""SC\300""SC-""SC\301""SD\300""SD-""SD\301""SE\300""SE-""SE\301""SF\300""SF\301""SG\300""SG-""SG\301""SH\300""SH\301" TR_CUSTOMSW " ON"
#else
  #define TR_VSWITCHES         TR_9X_3POS_SWITCHES "THR""RUD""ELE""AIL""GEA""TRN" TR_CUSTOMSW " ON"
#endif

#if defined(PCBSKY9X)
  #define TR_ROTARY_ENCODERS_VSRCRAW "REnc"
#elif defined(PCBGRUVIN9X) && ROTARY_ENCODERS > 2
  #define TR_ROTARY_ENCODERS_VSRCRAW "REa ""REb ""REc ""REd "
#elif defined(PCBGRUVIN9X) && ROTARY_ENCODERS <= 2
  #define TR_ROTARY_ENCODERS_VSRCRAW "REa ""REb "
#else
  #define TR_ROTARY_ENCODERS_VSRCRAW
#endif

#if defined(HELI)
#define TR_CYC_VSRCRAW "CYC1""CYC2""CYC3"
#else
#define TR_CYC_VSRCRAW "[C1]""[C2]""[C3]"
#endif

#define TR_VSRCRAW       "---\0""Dir\0""Prf\0""Gaz\0""Ail\0" TR_POTS_VSRCRAW TR_ROTARY_ENCODERS_VSRCRAW "MAX\0" TR_CYC_VSRCRAW "TrmD" "TrmP" "TrmG" "TrmA" TR_SW_VSRCRAW

#define LEN_VTMRMODES    "\003"
#define TR_VTMRMODES     "OFF""ABS""GZs""GZ%""GZt"

#define LEN_VTRAINERMODES      "\006"
#define TR_VTRAINERMODES       "Ma\203tre""El\201ve\0"

#define LEN_VFAILSAFE    "\011"
#define TR_VFAILSAFE     "Maintien\0""Pr\200d\200fini""Pas d'imp"

// ZERO TERMINATED STRINGS
#define INDENT                 "\001"
#define LEN_INDENT             1
#define INDENT_WIDTH           (FW/2)

#if defined(PCBTARANIS)
  #define TR_ENTER             "[ENTER]"
#else
  #define TR_ENTER             "[MENU]"
#endif

#define TR_POPUPS              TR_ENTER"\010[EXIT]"
#define OFS_EXIT               sizeof(TR_ENTER)

#define TR_MENUWHENDONE        CENTER"\006"TR_ENTER" QUAND PRET"
#define TR_FREE                "disp"
#define TR_DELETEMODEL         "SUPPRIMER MODELE"
#define TR_COPYINGMODEL        "Copie..."
#define TR_MOVINGMODEL         "D\200placement..."
#define TR_LOADINGMODEL        "Chargement..."
#define TR_NAME                "Nom"
#define TR_MODELNAME           TR("Nom mod\201le","Nom du mod\201le")
#define TR_PHASENAME           "Nom phase"
#define TR_MIXNAME             TR("Nom mix.","Nom du mixeur")
#define TR_EXPONAME            "Nom expo"
#define TR_BITMAP              "Image du mod\201le"
#define TR_TIMER               "Chrono "
#define TR_ELIMITS             TR("Limites \200t","Limites \200tendues")
#define TR_ETRIMS              TR("Trims \200t.","Trims \200tendus")
#define TR_TRIMINC             TR("Pas Trim","Pas des trims")
#define TR_TTRACE              "Source gaz"
#define TR_TTRIM               "Trim gaz"
#define TR_BEEPCTR             TR("Bips centr","Bips centrage")
#define TR_PROTO               TR(INDENT"Proto.",INDENT"Protocole")
#define TR_PPMFRAME            INDENT"Trame PPM"
#define TR_MS                  "ms"
#define TR_SWITCH              TR("Inter","Interrupteur")
#define TR_TRIMS               "Trims"
#define TR_FADEIN              "Fondu ON"
#define TR_FADEOUT             "Fondu OFF"
#define TR_DEFAULT             "(d\200faut)"
#define TR_CHECKTRIMS          "\006V\200rif\012Trims"
#define OFS_CHECKTRIMS         (9*FW)
#define TR_SWASHTYPE           TR("Type de Plat.","Type de plateau")
#define TR_COLLECTIVE          TR("Collectif","Voie du pas collectif")
#define TR_SWASHRING           TR("Limite Cycl.","Limite du cyclique")
#define TR_ELEDIRECTION        TR("Inv. longitud.","Inversion longitudinal")
#define TR_AILDIRECTION        TR("Inv. lat\200ral","Inversion lat\200ral")
#define TR_COLDIRECTION        TR("Inv. collectif","Inversion collectif")
#define TR_MODE                INDENT"Mode"
#define TR_NOFREEEXPO          "Max expos atteint!"
#define TR_NOFREEMIXER         "Max mixages atteint!"
#define TR_INSERTMIX           "INSERER MIXAGE"
#define TR_EDITMIX             "EDITER MIXAGE"
#define TR_SOURCE              INDENT"Source"
#define TR_WEIGHT              "Ratio"
#define TR_EXPO                TR("Expo","Exponentiel")
#define TR_SIDE                "Cot\200"
#define TR_DIFFERENTIAL        "Diff\200rentiel"
#define TR_OFFSET              INDENT"D\200calage"
#define TR_TRIM                "Trim"
#define TR_DREX                "DRex"
#define TR_CURVE               "Courbe"
#define TR_FLMODE              TR("Phase","Phases")
#define TR_MIXWARNING          "Alerte"
#define TR_OFF                 "OFF"
#define TR_MULTPX              "Op\200ration"
#define TR_DELAYDOWN           "Retard bas"
#define TR_DELAYUP             "Retard haut"
#define TR_SLOWDOWN            "Ralenti bas"
#define TR_SLOWUP              "Ralenti haut"
#define TR_MIXER               "MIXEUR"
#define TR_CV                  "CB"
#define TR_GV                  "VG"
#define TR_ACHANNEL            "A"
#define TR_RANGE               INDENT"Plage"
#define TR_BAR                 "Barre"
#define TR_ALARM               INDENT"Alarme"
#define TR_USRDATA             "Donn\200es"
#define TR_BLADES              INDENT"Pales"
#define TR_SCREEN              "Ecran "
#define TR_SOUND_LABEL         "Son"
#define TR_LENGTH              INDENT"Dur\200e"
#define TR_SPKRPITCH           INDENT"Tonalit\200"
#define TR_HAPTIC_LABEL        "Vibreur"
#define TR_HAPTICSTRENGTH      INDENT"Force"
#define TR_CONTRAST            "Contraste"
#define TR_ALARMS_LABEL        "Alarmes"
#define TR_BATTERY_RANGE       "Plage batterie"
#define TR_BATTERYWARNING      TR(INDENT"Batterie",INDENT"Batterie faible")
#define TR_INACTIVITYALARM     INDENT"Inactivit\200"
#define TR_MEMORYWARNING       INDENT"M\200moire faible"
#define TR_ALARMWARNING        TR(INDENT"Silence",INDENT"Sons d\200sactiv\200s")
#define TR_RENAVIG             "Navig EncRot"
#define TR_THROTTLEREVERSE     "Inversion gaz"
#define TR_MINUTEBEEP          TR(INDENT"Bip min.",INDENT"Annonces minutes")
#define TR_BEEPCOUNTDOWN       TR(INDENT"Bip fin",INDENT"Compte \202 rebours")
#define TR_PERSISTENT          TR(INDENT"Persist.",INDENT"Persistant")
#define TR_BACKLIGHT_LABEL     "R\200tro\200clairage"
#define TR_BLDELAY             INDENT"Dur\200e"
#define TR_BLONBRIGHTNESS      INDENT"Luminosit\200 ON"
#define TR_BLOFFBRIGHTNESS     INDENT"Luminosit\200 OFF"
#define TR_SPLASHSCREEN        "Logo d'accueil"
#define TR_THROTTLEWARNING     "Alerte gaz"
#define TR_SWITCHWARNING       TR("Alerte int","Alerte interrupt.")
#define TR_TIMEZONE            "Fuseau horaire"
#define TR_RXCHANNELORD        TR("Ordre voies RX","Ordre des voies pr\200f\200r\200")
#define TR_SLAVE               "El\201ve"
#define TR_MODESRC             "Mode\006% Source"
#define TR_MULTIPLIER          "Multiplieur"
#define TR_CAL                 "Cal"
#define TR_VTRIM               "Trim- +"
#define TR_BG                  "BG:"
#define TR_MENUTOSTART         CENTER"\006" TR_ENTER " POUR DEBUT"
#define TR_SETMIDPOINT         CENTER"\010REGLER NEUTRES"
#define TR_MOVESTICKSPOTS      CENTER"\004BOUGER STICKS/POTS"
#define TR_RXBATT              "Batt.RX"
#define TR_TXnRX               "Tx:\0Rx:"
#define OFS_RX                 4
#define TR_ACCEL               "Acc:"
#define TR_NODATA              "NO DATA"
#define TR_TM1TM2              "TM1\032TM2"
#define TR_THRTHP              "THR\032TH%"
#define TR_TOT                 "TOT"
#define TR_TMR1LATMAXUS        "Tmr1Lat max\006us"
#define STR_US (STR_TMR1LATMAXUS+12)
#define TR_TMR1LATMINUS        "Tmr1Lat min\006us"
#define TR_TMR1JITTERUS        "Tmr1 Jitter\006us"

#if defined(CPUARM)
  #define TR_TMIXMAXMS           "Tmix max\012ms"
#else
  #define TR_TMIXMAXMS           "Tmix max\014ms"
#endif

#define TR_T10MSUS             "T10ms\016us"
#define TR_FREESTACKMINB       "Free Stack\010b"
#define TR_MENUTORESET         CENTER TR_ENTER" pour reset"
#define TR_PPM                 "PPM"
#define TR_CH                  "CH"
#define TR_MODEL               "MODELE"
#define TR_FP                  "PV"
#define TR_MIX                 "MIX"
#define TR_EEPROMLOWMEM        "EEPROM pleine!"
#define TR_ALERT               "\014ALERTE"
#define TR_PRESSANYKEYTOSKIP   "Touche pour ignorer"
#define TR_THROTTLENOTIDLE     "Gaz pas \202 z\200ro"
#define TR_ALARMSDISABLED      "Alarmes D\200sactiv\200es"
#define TR_PRESSANYKEY         TR("Touche pour continuer","Touche pour continuer")
#define TR_BADEEPROMDATA       "EEPROM corrompue"
#define TR_EEPROMFORMATTING    "Formatage EEPROM"
#define TR_EEPROMOVERFLOW      "D\200passement EEPROM"
#define TR_MENURADIOSETUP      "CONFIG RADIO"
#define TR_MENUDATEANDTIME     "DATE ET HEURE"
#define TR_MENUTRAINER         "ECOLAGE"
#define TR_MENUVERSION         "VERSION"
#define TR_MENUDIAG            TR("INTERS","TEST INTERRUPTEURS")
#define TR_MENUANA             TR("ANAS","ENTREES ANALOGIQUES")
#define TR_MENUCALIBRATION     "CALIBRATION"
#define TR_TRIMS2OFFSETS       "\006Trims => Offsets"
#define TR_MENUMODELSEL        "MODELES"
#define TR_MENUSETUP           TR("CONF. MODELE","CONFIGURATION")
#define TR_MENUFLIGHTPHASE     "PHASE DE VOL"
#define TR_MENUFLIGHTPHASES    "PHASES DE VOL"
#define TR_MENUHELISETUP       TR("CONF.HELI","CONFIGURATION HELICO")

#if defined(PPM_CENTER_ADJUSTABLE) || defined(PPM_LIMITS_SYMETRICAL) // The right menu titles for the gurus ...
  #define TR_MENUDREXPO          "MANCHES"
  #define TR_MENULIMITS          "SORTIES"
#else
  #define TR_MENUDREXPO          "DR/EXPO"
  #define TR_MENULIMITS          "LIMITES"
#endif

#define TR_MENUCURVES          "COURBES"
#define TR_MENUCURVE           "COURBE"
#define TR_MENUCUSTOMSWITCH    "INTER PERS."
#define TR_MENUCUSTOMSWITCHES  TR("INTERS PERS.","INTERS PERSONNALISES")
#define TR_MENUCUSTOMFUNC      TR("FONCTIONS SPEC.","FONCTIONS SPECIALES")
#define TR_MENUTELEMETRY       "TELEMESURE"
#define TR_MENUTEMPLATES       "GABARITS"
#define TR_MENUSTAT            TR("STATS","STATISTIQUES")
#define TR_MENUDEBUG           "DEBUG"
#define TR_RXNUM               TR(INDENT"NumRx",INDENT"No. r\200cepteur")
#define TR_SYNCMENU            "Sync [MENU]"
#define TR_LIMIT               INDENT"Limite"
#define TR_MINRSSI             "RSSI Min."
#define TR_LATITUDE            "Latitude"
#define TR_LONGITUDE           "Longitude"
#define TR_GPSCOORD            TR("Coordonn\200es","Coordonn\200es GPS")
#define TR_VARIO               TR("Vario","Variom\201tre")
#define TR_SHUTDOWN            "ARRET EN COURS"
#define TR_BATT_CALIB          "Calib. Batterie"
#define TR_CURRENT_CALIB       "Calib. Courant"
#define TR_VOLTAGE             INDENT"Tension"
#define TR_CURRENT             INDENT"Courant"
#define TR_SELECT_MODEL        "S\200lect. Mod\201le"
#define TR_CREATE_MODEL        "Cr\200er Mod\201le"
#define TR_COPY_MODEL          "Copier Mod\201le"
#define TR_MOVE_MODEL          "D\200placer Mod\201le"
#define TR_BACKUP_MODEL        "Archiver Mod\201le"
#define TR_DELETE_MODEL        "Supprimer Mod\201le"
#define TR_RESTORE_MODEL       "Restaurer Mod\201le"
#define TR_SDCARD_ERROR        "Erreur carte SD"
#define TR_NO_SDCARD           "Pas de carte SD"
#define TR_INCOMPATIBLE        "Incompatible"
#define TR_WARNING             "ALERTE"
#define TR_EEPROMWARN          "EEPROM"
#define TR_THROTTLEWARN        "GAZ"
#define TR_ALARMSWARN          "SON"
#define TR_SWITCHWARN          "INTERS"
#define TR_INVERT_THR          "Inverser Gaz?"
#define TR_SPEAKER_VOLUME      INDENT"Volume"
#define TR_LCD                 "Afficheur"
#define TR_BRIGHTNESS          "Luminosit\200"
#define TR_CPU_TEMP            "Temp. CPU\016>"
#define TR_CPU_CURRENT         "Courant\022>"
#define TR_CPU_MAH             "Consomm."
#define TR_COPROC              "CoProc."
#define TR_COPROC_TEMP         "Temp. MB \016>"
#define TR_CAPAWARNING         INDENT "Capacit\200 Basse"
#define TR_TEMPWARNING         INDENT "Surchauffe"
#define TR_FUNC                TR("Fonc","Fonction")
#define TR_V1                  "V1"
#define TR_V2                  "V2"
#define TR_DURATION            "Dur\200e"
#define TR_DELAY               "D\200lai"
#define TR_SD_CARD             "Carte SD"
#define TR_SDHC_CARD           "Carte SD-HC"
#define TR_NO_SOUNDS_ON_SD     "Aucun son sur SD"
#define TR_NO_MODELS_ON_SD     "Aucun mod\201le SD"
#define TR_NO_BITMAPS_ON_SD    "Aucun Bitmap SD"
#define TR_PLAY_FILE           "Lire"
#define TR_DELETE_FILE         "Supprimer"
#define TR_COPY_FILE           "Copier"
#define TR_RENAME_FILE         "Renommer"
#define TR_REMOVED             " supprim\200"
#define TR_SD_INFO             "Information"
#define TR_SD_FORMAT           "Formater"
#define TR_NA                  "N/D"
#define TR_HARDWARE            "MATERIEL"
#define TR_FORMATTING          "Formatage..."
#define TR_TEMP_CALIB          "Calib. temp"
#define TR_TIME                "Heure"
#define TR_BAUDRATE            "Baudrate BT"
#define TR_SD_INFO_TITLE       "INFO SD"
#define TR_SD_TYPE             "Type:"
#define TR_SD_SPEED            "Vitesse:"
#define TR_SD_SECTORS          "Secteurs:"
#define TR_SD_SIZE             "Taille:"
#define TR_TYPE                "Type"
#define TR_GLOBAL_VARS         "Variables Globales"
#define TR_GLOBAL_VAR          "Variable globale"
#define TR_MENUGLOBALVARS      "VARIABLES GLOBALES"
#define TR_OWN                 "Pers"
#define TR_DATE                "Date"
#define TR_ROTARY_ENCODER      "Enc.Rot."
#define TR_CHANNELS_MONITOR    "MONITEUR CANAUX"
#define TR_INTERNALRF          "HF interne"
#define TR_EXTERNALRF          "HF externe"
#define TR_FAILSAFE            INDENT"Type failsafe"
#define TR_FAILSAFESET         "REGLAGES FAILSAFE"
#define TR_COUNTRYCODE         "Zone g\200o."
#define TR_VOICELANG           TR("Langue voix","Langue annonces vocales")
#define TR_UNITSSYSTEM         "Unit\200s"
#define TR_EDIT                "Editer"
#define TR_INSERT_BEFORE       "Ins\200rer avant"
#define TR_INSERT_AFTER        "Ins\200rer apr\201s"
#define TR_COPY                "Copier"
#define TR_MOVE                "D\200placer"
#define TR_DELETE              "Effacer"
#define TR_RESET_FLIGHT        "R\200initialiser vol"
#define TR_RESET_TIMER1        "R\200initialiser Timer1"
#define TR_RESET_TIMER2        "R\200initialiser Timer2"
#define TR_RESET_TELEMETRY     "R\200init. T\200l\200mesure"
#define TR_STATISTICS          "Statistiques"
#define TR_ABOUT_US            "A propos"
#define TR_AND_SWITCH          "ET suppl."
#define TR_CF                  "FS"
#define TR_SPEAKER             INDENT"Haut-p."
#define TR_BUZZER              INDENT"Bipeur"
#define TR_BYTES               "bytes"
#define TR_MODULE_BIND         "[Bind]"
#define TR_MODULE_RANGE        "[Port.]"
#define TR_RESET               "[RAZ]"
#define TR_SET                 "[D\200f.]"
#define TR_TRAINER             "Ecolage"
#define TR_ANTENNAPROBLEM      CENTER "Antenne radio d\200fectueuse!"
#define TR_MODELIDUSED         TR("ID d\200j\202 affect\200","No de mod�le d\200j\202 utilis\200")
#define TR_MODULE              INDENT "Type de module"
#define TR_CHANNELRANGE        INDENT "Plage de canaux"
#define TR_LOWALARM            INDENT "Alarme basse"
#define TR_CRITICALALARM       INDENT "Alarme critique"

// Taranis column headers
#define TR_PHASES_HEADERS      { " Nom ", " Inter ", " Trims ", " Fondu ON ", " Fondu OFF " }
#define TR_LIMITS_HEADERS      { " Nom ", " Subtrim ", " Min ", " Max ", " Direction ", " Neutre PPM ", " Mode subtrim " }
#define TR_CSW_HEADERS         { " Fonction ", " V1 ", " V2 ", " ET suppl. ", " Dur\200e ", " D\200lai " }

//Taranis About screen
#define TR_ABOUTUS             "A PROPOS"

#define TR_ABOUT_OPENTX_1      "OpenTX est libre, non-"
#define TR_ABOUT_OPENTX_2      "commercial et n'offre aucune"
#define TR_ABOUT_OPENTX_3      "garantie. Il a ete developpe"
#define TR_ABOUT_OPENTX_4      "gratuitement. Vos donations"
#define TR_ABOUT_OPENTX_5      "sont bienvenues!"

#define TR_ABOUT_BERTRAND_1    "Bertrand Songis"
#define TR_ABOUT_BERTRAND_2    "Auteur principal d'openTx"
#define TR_ABOUT_BERTRAND_3    "Codeveloppeur de Companion9x"

#define TR_ABOUT_MIKE_1        "Mike Blandford"
#define TR_ABOUT_MIKE_2        "Maitre du code et des"
#define TR_ABOUT_MIKE_3        "drivers"
#define TR_ABOUT_MIKE_4        ""
       
#define TR_ABOUT_ROMOLO_1      "Romolo Manfredini"
#define TR_ABOUT_ROMOLO_2      "Developpeur principal de"
#define TR_ABOUT_ROMOLO_3      "Companion9x"
      
#define TR_ABOUT_ANDRE_1       "Andr\200 Bernet"
#define TR_ABOUT_ANDRE_2       "Fonctionnalites, usabilite,"
#define TR_ABOUT_ANDRE_3       "debogage, documentation"

#define TR_ABOUT_ROB_1         "Rob Thomson"
#define TR_ABOUT_ROB_2         "Webmaster d'openrcforums"

#define TR_ABOUT_MARTIN_1      "Martin Hotar"
#define TR_ABOUT_MARTIN_2      "Design graphique"

#define TR_ABOUT_HARDWARE_1    "FrSky"
#define TR_ABOUT_HARDWARE_2    "Developpeur/fabricant"
#define TR_ABOUT_HARDWARE_3    "du materiel"

#define TR_ABOUT_PARENTS_1     "Projets parents"
#define TR_ABOUT_PARENTS_2     "ersky9x (Mike Blandford)"
#define TR_ABOUT_PARENTS_3     "ER9X (Erez Raviv)"
#define TR_ABOUT_PARENTS_4     "TH9X (Thomas Husterer)"

#define TR_CHR_SHORT  's'
#define TR_CHR_LONG   'l'
#define TR_CHR_TOGGLE 't'
#define TR_CHR_HOUR   'h'
