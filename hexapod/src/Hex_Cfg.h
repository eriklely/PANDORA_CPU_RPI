//=======================================================================================
// Hexapod-besturing via WiFi (UDP)
// Bestand: Hex_Cfg.h
// Huidige datum: 27 februari 2025
//=======================================================================================
 

#ifndef HEX_CFG_H
#define HEX_CFG_H

#include "ArduinoDefs.h"

// Om code te kunne testen
//#define TEST

// enable sound
#define OPT_LED

// Define the UDP port to match the C# controller
#define UDP_PORT 8888
#define SEND_UDP_PORT 8889
#define CONTROLLER_IP "10.10.10.4"

// Video feed (camera stream to controller)
#define VIDEO_STREAM_HOST    CONTROLLER_IP   // Destination IP for video UDP stream
#define VIDEO_STREAM_PORT   5000            // UDP port for video stream
#define VIDEO_STREAM_WIDTH  320             // Capture width (e.g. 320, 640, 1280)
#define VIDEO_STREAM_HEIGHT 240             // Capture height (e.g. 240, 480, 720)
#define VIDEO_STREAM_FPS    30              // Target frame rate (e.g. 15, 25, 30)

// Face detection (DNN): reduce false positives
#define FACE_CONFIDENCE_THRESHOLD  0.65f    // Lower = more detections (less strict). Range: 0.4–0.9. Was 0.75.
#define FACE_MIN_PIXELS            16       // Ignore detections smaller than this (width/height). Was 24.

// Camera-mode PI (face tracking): when face is briefly lost
#define FACE_NO_FACE_HOLD_FRAMES   24       // Frames to hold last position before full reset (~2 s at ~12 FPS)
#define FACE_NO_FACE_DECAY         0.97     // Decay per frame when no face (0.85 = fast, 0.97 = slow, 1.0 = hold)

// Face tracking smoothing
#define FACE_TRACKING_EMA_ALPHA    0.4      // Bounding box smoothing (lower = smoother gliding, higher = faster updates)

// Face tracking PI controller gains
#define FACE_KP_TURN               26.0     // Proportional gain for turning (Original: 26.0)
#define FACE_KI_TURN               17.0     // Integral gain for turning (Original: 17.0)
#define FACE_KP_TILT               10.0     // Proportional gain for tilting (Original: 10.0)
#define FACE_KI_TILT               8.0      // Integral gain for tilting (Original: 8.0)

// PRINTUDP: UDP echo to controller. For release builds on RPi, define HEXAPOD_DISABLE_UDP_ECHO
// in compiler flags (e.g. -DHEXAPOD_DISABLE_UDP_ECHO) to avoid vsnprintf/strstr/sendto in hot path.
#ifdef HEXAPOD_DISABLE_UDP_ECHO
#define PRINTUDP(...)  ((void)0)
#else
#define PRINTUDP sendUdpOnly
#endif

// Conditionele PRINT macro gebaseerd op USEPRINTF
#define PRINT(...) do { if (g_fPrintEnabled) { printf("\n"); printf(__VA_ARGS__); } } while (0)

// time-out voor geen commando's ontvangen en stoppen met bewegen
#define TIMEOUT_MS 2000

// meer boodschappen weergeven voor debug
// #define DEBUG
// #define DEBUG_SERVOS

// Servo offsets
#define cFemurHornOffset1 -100     // Offset voor femur servo
#define cTibiaHornOffset1 560      // Offset voor tibia servo

//Voor single leg body shift
#define BODY_SHIFT_X 20    // Base X shift magnitude for front/rear legs
#define BODY_SHIFT_Z 30    // Base Y shift magnitude for all legs
#define BODY_ROTATE_X 15   // Base X rotate magnitude for all legs
#define BODY_ROTATE_Z 10   // Base Z rotate magnitude for all legs

// Alle beweging maximaliseren door dynamic scaling
#define EXTENDED_MOVEMENT

//Lopen in cameramode toestaan
#define CAMERAWALKENABLE

// Standaard stahoogte in mm
#define StandingHeight 60  

//Compliance slope zetten
#define COMPLIANCE_SLOPE 0x40  // Hex waarde: kies uit 0x10, 0x20, 0x40, 0x80
//#define SHOW_COMPLIANCE_SLOPE

// routine voor AX speedcontrol gebruiken
#define USE_AX12_SPEED_CONTROL

//=======================================================================================
// Tabellen bouwen voor pootconfiguratie zoals I/O en MIN/MAX waarden voor eenvoudige toegang via een FOR-lus
// Constanten worden nog steeds als enkele waarden gedefinieerd in het cfg-bestand voor leesbaarheid/configuratiegemak
//
// Definieer welke servo's van de poten geïnverteerd moeten worden, afhankelijk van de robot
//=======================================================================================
#define cRRCoxaInv 1 
#define cRMCoxaInv 1 
#define cRFCoxaInv 1 
#define cLRCoxaInv 0 
#define cLMCoxaInv 0 
#define cLFCoxaInv 0 

#define cRRFemurInv 1 
#define cRMFemurInv 1 
#define cRFFemurInv 1 
#define cLRFemurInv 0 
#define cLMFemurInv 0 
#define cLFFemurInv 0 

#define cRRTibiaInv 0 
#define cRMTibiaInv 0 
#define cRFTibiaInv 0 
#define cLRTibiaInv 1 
#define cLMTibiaInv 1 
#define cLFTibiaInv 1

// Debug IO pinnen
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}

// Driver instellingen
#define USE_AX12_DRIVER          // Gebruik AX12 driver
#define OPT_BACKGROUND_PROCESS   // AX12 heeft achtergrondproces
#define TIMEOUT_SERVO  30000       // Timeout in milliseconden

// Voltage controle
#define cVoltagePin  7      // Analoge pin voor voltage meting
#define cTurnOffVol  990    // Uitschakelspanning (9.9V)
#define cTurnOnVol   1020   // Inschakelspanning (10.2V)

//=======================================================================================
// Gait snelheden
//=======================================================================================
#define DEFAULT_GAIT_SPEED 30     // Standaard gang snelheid
#define DEFAULT_SLOW_GAIT  80    // Langzame gang snelheid
#define SERVO_MOVE_TIME    80       // Het wacht interval dat de servo snelheid bepaald

//=======================================================================================
// Leg config
//=======================================================================================
#define CNT_LEGS 6                // Aantal poten (verondersteld hexapod)
//#define ADJUSTABLE_LEG_ANGLES     // Verstelbare poothoeken

//=======================================================================================
// Pin definities voor servo's
//=======================================================================================
#define cRRCoxaPin      7   // Links voor coxa
#define cRRFemurPin     8   // Links voor femur
#define cRRTibiaPin     9   // Links voor tibia

#define cRMCoxaPin      4   // Links midden coxa
#define cRMFemurPin     5   // Links midden femur
#define cRMTibiaPin     6   // Links midden tibia

#define cRFCoxaPin      1   // Links achter coxa
#define cRFFemurPin     2   // Links achter femur
#define cRFTibiaPin     3   // Links achter tibia

#define cLRCoxaPin      10  // Rechts voor coxa
#define cLRFemurPin     11  // Rechts voor femur
#define cLRTibiaPin     12  // Rechts voor tibia

#define cLMCoxaPin      13  // Rechts midden coxa
#define cLMFemurPin     14  // Rechts midden femur
#define cLMTibiaPin     15  // Rechts midden tibia

#define cLFCoxaPin      16   // Rechts achter coxa
#define cLFFemurPin     17   // Rechts achter femur
#define cLFTibiaPin     18   // Rechts achter tibia

#define cCMTiltPin      19   // Camera tilt
#define cCMPanPin       20   // Camera pan

//=======================================================================================
// [MIN/MAX HOEKEN]
//=======================================================================================
#define cRRCoxaMin1     -450    // Minimaal rechts achter coxa
#define cRRCoxaMax1     450     // Maximaal rechts achter coxa
#define cRRFemurMin1    -900    // Minimaal rechts achter femur
#define cRRFemurMax1    900     // Maximaal rechts achter femur
#define cRRTibiaMin1    -900   // Minimaal rechts achter tibia
#define cRRTibiaMax1    700     // Maximaal rechts achter tibia

#define cRMCoxaMin1     -450    // Minimaal rechts midden coxa
#define cRMCoxaMax1     450     // Maximaal rechts midden coxa
#define cRMFemurMin1    -900    // Minimaal rechts midden femur
#define cRMFemurMax1    900     // Maximaal rechts midden femur
#define cRMTibiaMin1    -900   // Minimaal rechts midden tibia
#define cRMTibiaMax1    700     // Maximaal rechts midden tibia

#define cRFCoxaMin1     -450    // Minimaal rechts voor coxa
#define cRFCoxaMax1     450     // Maximaal rechts voor coxa
#define cRFFemurMin1    -900    // Minimaal rechts voor femur
#define cRFFemurMax1    900     // Maximaal rechts voor femur
#define cRFTibiaMin1    -900   // Minimaal rechts voor tibia
#define cRFTibiaMax1    700     // Maximaal rechts voor tibia

#define cLRCoxaMin1     -450    // Minimaal links achter coxa
#define cLRCoxaMax1     450     // Maximaal links achter coxa
#define cLRFemurMin1    -900    // Minimaal links achter femur
#define cLRFemurMax1    900     // Maximaal links achter femur
#define cLRTibiaMin1    -900   // Minimaal links achter tibia
#define cLRTibiaMax1    700     // Maximaal links achter tibia

#define cLMCoxaMin1     -450    // Minimaal links midden coxa
#define cLMCoxaMax1     450     // Maximaal links midden coxa
#define cLMFemurMin1    -900    // Minimaal links midden femur
#define cLMFemurMax1    900     // Maximaal links midden femur
#define cLMTibiaMin1    -900   // Minimaal links midden tibia
#define cLMTibiaMax1    700     // Maximaal links midden tibia

#define cLFCoxaMin1     -450    // Minimaal links voor coxa
#define cLFCoxaMax1     450     // Maximaal links voor coxa
#define cLFFemurMin1    -900    // Minimaal links voor femur
#define cLFFemurMax1    900     // Maximaal links voor femur
#define cLFTibiaMin1    -900   // Minimaal links voor tibia
#define cLFTibiaMax1    700     // Maximaal links voor tibia

#define cCMTiltMin1     -550   // Minimaal camera tilt
#define cCMTiltMax1     350     // Maximaal camera tilt
#define cCMPanMin1      -300   // Minimaal camera pan
#define cCMPanMax1      300     // Maximaal camera pan
#define TURRET_TILT_SCALE  1.5  // Tilt servo angle scale factor
#define TURRET_PAN_SCALE   2.0  // Pan servo angle scale factor

//=======================================================================================
// [POOT AFMETINGEN] in mm
//=======================================================================================
#define cXXCoxaLength     52    // Coxa lengte
#define cXXFemurLength    66    // Femur lengte (gemeten waarde nodig)
#define cXXTibiaLength    138   // Tibia lengte (gemeten waarde nodig)

#define cRRCoxaLength     cXXCoxaLength    // Rechts achter
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength    // Rechts midden
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength    // Rechts voor
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength    // Links achter
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength    // Links midden
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength    // Links voor
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength

//=======================================================================================
// [LICHAAM AFMETINGEN]
//=======================================================================================
#define cRRCoxaAngle1   -450    // Standaard coxa hoek rechts achter
#define cRMCoxaAngle1    0      // Standaard coxa hoek rechts midden
#define cRFCoxaAngle1    450    // Standaard coxa hoek rechts voor
#define cLRCoxaAngle1    -450   // Standaard coxa hoek links achter
#define cLMCoxaAngle1    0      // Standaard coxa hoek links midden
#define cLFCoxaAngle1    450    // Standaard coxa hoek links voor

#define X_COXA      73   // Afstand tussen voor/achter poten /2
#define Y_COXA      73   // Afstand tussen voor/achter poten /2
#define M_COXA      117  // Afstand tussen midden poten /2

#define cRROffsetX      -73     // X offset rechts achter
#define cRROffsetZ      115     // Z offset rechts achter

#define cRMOffsetX      -117    // X offset rechts midden
#define cRMOffsetZ      0       // Z offset rechts midden

#define cRFOffsetX      -73     // X offset rechts voor
#define cRFOffsetZ      -115    // Z offset rechts voor

#define cLROffsetX      73      // X offset links achter
#define cLROffsetZ      115     // Z offset links achter

#define cLMOffsetX      117     // X offset links midden
#define cLMOffsetZ      0       // Z offset links midden

#define cLFOffsetX      73     // X offset links voor
#define cLFOffsetZ      -115     // Z offset links voor

//=======================================================================================
// [START POSITIES POTEN]
//=======================================================================================
#define cHexInitXZ      135    // Initiële XZ positie
#define CHexInitXZCos45 95     // Cosinus 45° berekening
#define CHexInitXZSin45 95     // Sinus 45° berekening
#define CHexInitY       55     // Initiële Y positie

#define CNT_HEX_INITS 2        // Aantal initiële posities
#define MAX_BODY_Y    175      // Maximale lichaamshoogte

#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 148};
const byte g_abHexMaxBodyY[] PROGMEM = {CHexInitY, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

#define cRRInitPosX     CHexInitXZCos45    // Startpositie rechts achter
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin45

#define cRMInitPosX     cHexInitXZ         // Startpositie rechts midden
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos45    // Startpositie rechts voor
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin45

#define cLRInitPosX     CHexInitXZCos45    // Startpositie links achter
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin45

#define cLMInitPosX     cHexInitXZ         // Startpositie links midden
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos45    // Startpositie links voor
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin45

#endif // HEX_CFG_H