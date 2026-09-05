//=======================================================================================
// Hexapod-besturing via WiFi (UDP)
// Bestand: Hexapod.h
// Huidige datum: 27 februari 2025
//=======================================================================================
 

#ifndef HEXAPOD_CORE_H  // Meer specifieke header guard naam
#define HEXAPOD_CORE_H
 
// Standaard includes
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <mutex>
#include <atomic>
#include "ArduinoDefs.h"

//=======================================================================================
// [CONSTANTEN]
//=======================================================================================
#define BUTTON_DOWN 0    // Knop ingedrukt
#define BUTTON_UP   1    // Knop losgelaten

#define c1DEC       10     // 1 decimaal
#define c2DEC       100    // 2 decimalen
#define c4DEC       10000  // 4 decimalen
#define c6DEC       1000000 // 6 decimalen


// Poten voor hexapod
#define cRR 0
#define cRM 1
#define cRF 2
#define cLR 3
#define cLM 4
#define cLF 5
#define CNT_LEGS 6

#define WTIMERTICSPERMSMUL      64    // Multiplier voor timer conversie
#define WTIMERTICSPERMSDIV      125   // Divider voor timer conversie
#define USEINT_TIMERAV                // Gebruik interne timer

extern const byte NUM_GAITS;          // Aantal gangen

extern int hexapod_rpicam_main(int argc, char** argv);

// Functie declaraties voor gang controle
extern void GaitSelect(void);
extern double SmoothControl(double CtrlMoveInp, double CtrlMoveOut, double CtrlDivider, double SmMrgn);

// Functie declaraties voor feedback led
void showFeedbackLED(int red1, int dur1);
void updateLEDState();

//=======================================================================================
// Globale variabelen
//=======================================================================================
extern boolean g_fDebugOutput;          // Debug output aan/uit
extern boolean g_fEnableServos;         // Servo verwerking aan/uit
extern boolean g_fPrintEnabled;  
extern boolean g_fDebugEnabled;
extern uint8_t g_iLegInitIndex; 

struct FaceData {
    int num_faces = 0;
    int centroid_x = 0;
    int centroid_y = 0;
};

extern FaceData latest_faces;   // declaration only
extern std::mutex face_mutex;   // declaration only
extern std::atomic<bool> g_fFaceDetectionEnabled;  // toggled by Recognition Mode button (bit 14)
extern void hexapod_rpicam_stop();  // signal camera thread to stop cleanly

// Functie declaraties
extern void MSound(byte cNotes, ...);
extern boolean CheckVoltage(void);
extern word GetLegsXZLength(void);
extern void AdjustLegPositions(word XZLength1);
extern void AdjustLegPositionsToBodyHeight();
extern void ResetLegInitAngles(void);
extern void RotateLegInitAngles(int iDeltaAngle);
extern void ResetHexapod();
extern void playFeedbackSound(int pigpio_handle, int freq1 = 2000, int dur1 = 50, int freq2 = 0, int dur2 = 0);
extern int pigpio_handle;

extern boolean g_fHeightTransition; // Declaration for height transition flag

extern unsigned long g_loop_time; 
extern word ServoMoveTime; //veranderd

extern double SmDiv;                       // Factor voor vloeiende controle
extern double SmMrgn;                      // Marge voor vloeiende controle

#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
#else
#define DoBackgroundProcess()   // Lege macro als achtergrondproces uit staat
#endif

// Platform specifieke definities
#ifdef __AVR__
#if not defined(UBRR1H)
#if cSSC_IN != 0
extern SoftwareSerial SSCSerial;
#endif
#endif
#endif

#if defined(__PIC32MX__)
#if defined F
#undef F
#endif
#define F(X) (X)
#endif

//=======================================================================================
// Input Controller Klasse
//=======================================================================================
class WifiInputController {
public:
    virtual ~WifiInputController() = default;
    virtual void Init(void);
    virtual void Cleanup(void);
    virtual void ControlInput(void);
    virtual void AllowControllerInterrupts(boolean fAllow);

private:
};

// Functie om input controller te registreren
extern void RegisterInputController(WifiInputController *pic);

// 3D Coördinaat structuur
typedef struct _Coord3D {
    long x = 0;
    long y = 0;
    long z = 0;
} COORD3D;

//=======================================================================================
// Gang Structuur
//=======================================================================================
typedef struct _HexapodGait {
    short NomGaitSpeed;     // Nominale snelheid van de gang
    byte StepsInGait;       // Aantal stappen in gang
    byte NrLiftedPos;       // Aantal posities dat een poot wordt opgetild
    byte FrontDownPos;      // Waar de poot neergezet wordt
    byte LiftDivFactor;     // Lift delingsfactor
    byte TLDivFactor;       // Aantal stappen dat een poot op de grond is
    byte HalfLiftHeight;    // Halve lifthoogte  
    byte GaitLegNr[CNT_LEGS]; // Initiële posities van poten
} HEXAPODGAIT;

#define GATENAME(name)

//=======================================================================================
// Controle Status Structuur
//=======================================================================================
typedef struct _InControlState {
    boolean fRobotOn;           // Robot aan/uit
    boolean fPrev_RobotOn;      // Vorige status
    boolean RobotUp;            // Robot staat
    boolean fPrev_RobotUp;      // Vorige status
    boolean fControllerInUse;   // Controller in gebruik  
    COORD3D BodyPos;           // Lichaam positie
    COORD3D BodyRotOffset;     // Lichaam rotatie offset
    COORD3D BodyRot1;          // Lichaam rotatie
    COORD3D Head;              // Hoofd positie
    byte GaitType;             // Gang type
    byte GaitStep;             // Huidige stap in gang
    HEXAPODGAIT gaitCur;       // Huidige gang definitie
    short LegLiftHeight;       // Poot lift hoogte
    COORD3D TravelLength;      // Stap lengte
    byte SelectedLeg;          // Geselecteerde poot
    COORD3D SLLeg;             // Enkele poot positie
    boolean fSLHold;           // Enkele poot modus
    boolean BalanceMode;       // Balans modus
    byte InputTimeDelay;       // Input vertraging
    word SpeedControl;         // Snelheidscontrole
    byte ForceGaitStepCnt;     // Forceer gang stap

#ifdef ADJUSTABLE_LEG_ANGLES
    short aCoxaInitAngle1[CNT_LEGS];  // Initiële hoeken
#endif
} INCONTROLSTATE;

//=======================================================================================
// Servo Driver Klasse
//=======================================================================================
class ServoDriver {
public:
    void Init(void);              // Initialisatie
    void Cleanup(void);          // Opruimen
    word GetBatteryVoltage(void); // Batterij spanning
    void BeginServoUpdate(void);  // Start servo update
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
    void OutputServoInfoForTurret(short sPanAngle1, short sTiltAngle1);                          
    boolean CommitServoDriver(word wMoveTime);
    void FreeServos(void);
    void IdleTime(void);

#ifdef OPT_BACKGROUND_PROCESS
    void BackgroundProcess(void);
#endif

    boolean _fServosActive;
};

//=======================================================================================
// Globale Objecten
//=======================================================================================
extern ServoDriver g_ServoDriver;         // Servo driver instantie
extern WifiInputController g_WifiInputController; // Input controller instantie
extern INCONTROLSTATE g_InControlState;   // Controle status

#endif