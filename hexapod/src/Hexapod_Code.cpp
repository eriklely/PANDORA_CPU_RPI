//=======================================================================================
// Hexapod-besturing via WiFi (UDP)
// Bestand: Hexapod_Code.cpp
// Huidige datum: 27 februari 2025
//=======================================================================================
 

#define DEFINE_HEX_GLOBALS // Defines global arrays g_abHexIntXZ and g_abHexMaxBodyY here (single definition)

#include "Hex_Cfg.h"
#include "Hexapod.h"
#include <signal.h>
#include <math.h>
#include <stdio_ext.h>
#include <unistd.h>
#include <thread>

FaceData latest_faces{};      // ONE AND ONLY definition of the shared variable
std::mutex face_mutex;        // ONE AND ONLY definition of the mutex
std::atomic<bool> g_fFaceDetectionEnabled{false};  // face detection off on start; toggled by Recognition Mode (button bit 14)

#define BalanceDivFactor CNT_LEGS  // Andere waarden dan 6 kunnen worden gebruikt, testen... LET OP!! Op eigen risico ;)

//======================================================================================
// [TABELLEN]
// ArcCosinus-tabel
// Tabel opgebouwd in 3 delen voor hogere nauwkeurigheid nabij cos = 1.
// Grootste fout is nabij cos = 1, max 3*0.012098rad = 0.521 graden.
// - Cos 0 tot 0.9: stappen van 0.0079 rad [1/127]
// - Cos 0.9 tot 0.99: stappen van 0.0008 rad [0.1/127]
// - Cos 0.99 tot 1: stappen van 0.0002 rad [0.01/64]
// Tabellen overlappen, totale grootte: 277 bytes
//=======================================================================================
static const byte GetACos[] = {
    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225,
    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193,
    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,
    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117,
    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70,
    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0
};

// Sinus-tabel 90 graden, precisie 0.5 graden [180 waarden]
static const word GetSin[] = {
    0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564,
    1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007,
    3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383,
    4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664,
    5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819,
    6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826,
    7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660,
    8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304,
    9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743,
    9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969,
    9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000
};

// Standaard balansvertraging definiëren
#ifndef BALANCE_DELAY
#define BALANCE_DELAY 50
#endif

// Camera streaming function (from rpicam_udp.cpp)
//int hexapod_rpicam_main(int argc, char** argv);

// Servo-hoorn offsets per poot
#ifdef cRRFemurHornOffset1
static const short cFemurHornOffset1[] = {
    cRRFemurHornOffset1, cRMFemurHornOffset1, cRFFemurHornOffset1, cLRFemurHornOffset1, cLMFemurHornOffset1, cLFFemurHornOffset1
};
#define CFEMURHORNOFFSET1(LEGI) ((short)(cFemurHornOffset1[LEGI]))
#else
#ifndef cFemurHornOffset1
#define cFemurHornOffset1 0
#endif
#define CFEMURHORNOFFSET1(LEGI) (cFemurHornOffset1)
#endif

#ifdef cRRTibiaHornOffset1
static const short cTibiaHornOffset1[] = {
    cRRTibiaHornOffset1, cRMTibiaHornOffset1, cRFTibiaHornOffset1, cLRTibiaHornOffset1, cLMTibiaHornOffset1, cLFTibiaHornOffset1
};
#define CTIBIAHORNOFFSET1(LEGI) ((short)(cTibiaHornOffset1[LEGI]))
#else
#ifndef cTibiaHornOffset1
#define cTibiaHornOffset1 0
#endif
#define CTIBIAHORNOFFSET1(LEGI) (cTibiaHornOffset1)
#endif

// Min/Max waarden voor servo's
const short cCoxaMin1[] = {
    cRRCoxaMin1, cRMCoxaMin1, cRFCoxaMin1, cLRCoxaMin1, cLMCoxaMin1, cLFCoxaMin1
};
const short cCoxaMax1[] = {
    cRRCoxaMax1, cRMCoxaMax1, cRFCoxaMax1, cLRCoxaMax1, cLMCoxaMax1, cLFCoxaMax1
};
const short cFemurMin1[] = {
    cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1
};
const short cFemurMax1[] = {
    cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1
};
const short cTibiaMin1[] = {
    cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1
};
const short cTibiaMax1[] = {
    cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1
};

// Servo-inversie definities
const bool cCoxaInv[] = {cRRCoxaInv, cRMCoxaInv, cRFCoxaInv, cLRCoxaInv, cLMCoxaInv, cLFCoxaInv};
bool cFemurInv[] = {cRRFemurInv, cRMFemurInv, cRFFemurInv, cLRFemurInv, cLMFemurInv, cLFFemurInv};
const bool cTibiaInv[] = {cRRTibiaInv, cRMTibiaInv, cRFTibiaInv, cLRTibiaInv, cLMTibiaInv, cLFTibiaInv};

// Pootlengtes
const byte cCoxaLength[] = {
    cRRCoxaLength, cRMCoxaLength, cRFCoxaLength, cLRCoxaLength, cLMCoxaLength, cLFCoxaLength
};
const byte cFemurLength[] = {
    cRRFemurLength, cRMFemurLength, cRFFemurLength, cLRFemurLength, cLMFemurLength, cLFFemurLength
};
const byte cTibiaLength[] = {
    cRRTibiaLength, cRMTibiaLength, cRFTibiaLength, cLRTibiaLength, cLMTibiaLength, cLFTibiaLength
};

// Lichaamsoffsets (afstand tussen midden van het lichaam en midden van de coxa)
const short cOffsetX[] = {
    cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX
};
const short cOffsetZ[] = {
    cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ
};

// Standaard poothoeken
const short cCoxaAngle1[] = {
    cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1
};

#ifdef cRRInitCoxaAngle1
const short cCoxaInitAngle1[] = {
    cRRInitCoxaAngle1, cRMInitCoxaAngle1, cRFInitCoxaAngle1, cLRInitCoxaAngle1, cLMInitCoxaAngle1, cLFInitCoxaAngle1
};
#endif

// Startposities voor de poten
const short cInitPosX[] = {
    cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX
};
const short cInitPosY[] = {
    cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY
};
const short cInitPosZ[] = {
    cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ
};

//================================================================================================
// Globale variabelen voor debug-informatie
boolean g_fDebugOutput;
boolean g_fDebugEnabled = false;
boolean g_fEnableServos = true;    
boolean g_fPrintEnabled = false;

//==================================================================
// [HOEKEN]
short CoxaAngle1[CNT_LEGS];   // Actuele hoek van de horizontale heup, decimalen = 1
short FemurAngle1[CNT_LEGS];  // Actuele hoek van de verticale heup, decimalen = 1
short TibiaAngle1[CNT_LEGS];  // Actuele hoek van de knie, decimalen = 1

//==================================================================
// [POSITIES ENKELE POOTBESTURING]
short LegPosX[CNT_LEGS];  // Actuele X-positie van de poot
short LegPosY[CNT_LEGS];  // Actuele Y-positie van de poot
short LegPosZ[CNT_LEGS];  // Actuele Z-positie van de poot

//==================================================================
// [VARIABLES]
byte Index;       // Universeel gebruikte index
byte LegIndex;    // Index voor pootnummer

// GetSinCos / ArcCos
short AngleDeg1;  // Invoerhoek in graden, decimalen = 1
short sin4;       // Uitvoer sinus van de hoek, decimalen = 4
short cos4;       // Uitvoer cosinus van de hoek, decimalen = 4
short AngleRad4;  // Uitvoerhoek in radialen, decimalen = 4

// GetAtan2
short AtanX;      // Invoer X
short AtanY;      // Invoer Y
short Atan4;      // ArcTan2-uitvoer
long XYhyp2;      // Uitvoer hypotenusa van X en Y

// Lichaam Inverse Kinematica
short PosX;       // Invoerpositie van de voeten X
short PosZ;       // Invoerpositie van de voeten Z
short PosY;       // Invoerpositie van de voeten Y
long BodyFKPosX;  // Uitvoerpositie X van voeten met rotatie
long BodyFKPosY;  // Uitvoerpositie Y van voeten met rotatie
long BodyFKPosZ;  // Uitvoerpositie Z van voeten met rotatie

// Poot Inverse Kinematica
long IKFeetPosX;      // Invoerpositie van de voeten X
long IKFeetPosY;      // Invoerpositie van de voeten Y
long IKFeetPosZ;      // Invoerpositie van de voeten Z
boolean IKSolution;       // Uitvoer: true als oplossing mogelijk is
boolean IKSolutionWarning;// Uitvoer: true als oplossing bijna mogelijk is
boolean IKSolutionError;  // Uitvoer: true als oplossing niet mogelijk is

//==================================================================
// [TIJD]
unsigned long lTimerStart;  // Starttijd van de berekeningscycli
unsigned long lTimerEnd;    // Eindtijd van de berekeningscycli
word ServoMoveTime;         // Tijd voor servo-updates
word PrevServoMoveTime;     // Vorige tijd voor servo-updates

//==================================================================
// [GLOBAAL]

// Globale invoerbesturingsstatus
INCONTROLSTATE g_InControlState;

// Globale servo-driver klasse
ServoDriver g_ServoDriver;

boolean g_fLowVoltageShutdown;  // True als de robot uitschakelt door lage spanning
word Voltage;

//==================================================================
// [BALANS]
long TotalTransX;
long TotalTransZ;
long TotalTransY;
long TotalYBal1;
long TotalXBal1;
long TotalZBal1;

// [Enkele pootbesturing]
byte PrevSelectedLeg;
boolean AllDown;

// [Gait - Status]
boolean TravelRequest;  // Tijdelijke controle of de gait in beweging is

long GaitPosX[CNT_LEGS];  // Array met relatieve X-positie voor de gait
long GaitPosY[CNT_LEGS];  // Array met relatieve Y-positie voor de gait
long GaitPosZ[CNT_LEGS];  // Array met relatieve Z-positie voor de gait
long GaitRotY[CNT_LEGS];  // Array met relatieve Y-rotatie voor de gait

// Loop-invariant gait division cache — set once per GaitSeq() call, read 6× in Gait()
static long s_tlx_ldf,  s_tlz_ldf,  s_tly_ldf;   // TravelLength / LiftDivFactor
static long s_tlx_tldf, s_tlz_tldf, s_tly_tldf;   // TravelLength / TLDivFactor
static long s_lift_mid;                             // -3 * LegLiftHeight / (3 + HalfLiftHeight)

boolean fWalking;         // True als de robot loopt
byte bExtraCycle;         // Extra cycli om "einde gait bug" te vermijden
#define cGPlimit 2        // Limiet voor GaitPos-testen

boolean g_fHeightTransition = false; // Flag to indicate up/down transition

double SmMrgn = 50.0; // startwaarde voor smoohtcontrol

//================================================================================================
// Standaard gaits definiëren
//================================================================================================
#ifndef DEFAULT_GAIT_SPEED
#define DEFAULT_GAIT_SPEED 90
#define DEFAULT_SLOW_GAIT 70
#endif

HEXAPODGAIT APG[] = { 
    {DEFAULT_SLOW_GAIT, 12, 3, 2, 2, 8, 3, {7, 11, 3, 1, 5, 9} GATENAME(s_szGN1)},  // Ripple 12
    {DEFAULT_SLOW_GAIT, 8, 3, 2, 2, 4, 3, {1, 5, 1, 5, 1, 5} GATENAME(s_szGN2)},   // Tripod 8 stappen
    {DEFAULT_GAIT_SPEED, 12, 3, 2, 2, 8, 3, {5, 10, 3, 11, 4, 9} GATENAME(s_szGN3)}, // Triple Tripod 12 stappen
    {DEFAULT_GAIT_SPEED, 16, 5, 3, 4, 10, 1, {6, 13, 4, 14, 5, 12} GATENAME(s_szGN4)}, // Triple Tripod 16 stappen
    {DEFAULT_SLOW_GAIT, 24, 3, 2, 2, 20, 3, {13, 17, 21, 1, 5, 9} GATENAME(s_szGN5)}, // Wave 24 stappen
    {DEFAULT_GAIT_SPEED, 6, 2, 1, 2, 4, 1, {1, 4, 1, 4, 1, 4} GATENAME(s_szGN6)}  // Tripod 6 stappen
};

#ifdef ADD_GAITS
const byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]) + sizeof(APG_EXTRA)/sizeof(APG_EXTRA[0]);
#else
const byte NUM_GAITS = sizeof(APG)/sizeof(APG[0]);
#endif

byte g_cNoServoChanged = 0;  // Teller voor commits zonder servo-beweging
#define CNT_NO_SERVO_CHANGE_IDLE 20

//================================================================================================
// Functieprototypes
//================================================================================================
extern void GaitSelect(void);
extern void WriteOutputs(void);
extern void SingleLegControl(void);
extern void GaitSeq(void);
extern void BalanceBody(void);
extern void CheckAngles(void);
extern void InitVoice(void);
extern void PrintSystemStuff(void);

extern void BalCalcOneLeg(long PosX, long PosZ, long PosY, byte BalLegNr);
extern void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg);
extern void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr);
extern void Gait(byte GaitCurrentLegNr);
extern void GetSinCos(short AngleDeg1);
extern short GetATan2(short AtanX, short AtanY);
extern unsigned long isqrt32(unsigned long n);

extern void StartUpdateServos(void);
extern boolean TerminalMonitor(void);

//==================================================================------
// Cleanup - Wordt aangeroepen door main bij ontvangst van signalen
//==================================================================------
void cleanup(void) {
    hexapod_rpicam_stop();  // signal camera thread before releasing servos
    if (g_InControlState.fRobotOn) {
        g_ServoDriver.FreeServos();
    }
    g_ServoDriver.Cleanup();
    g_WifiInputController.Cleanup();
    PRINT("Abort");
}

//==================================================================------
// SETUP: de hoofdfunctie voor Arduino setup
//==================================================================------
void setup() {
    g_fDebugOutput = false;
    PRINT("%d %zu %d\n", __flbf(stdout), __fbufsize(stdout), __flbf(stdin));
    g_ServoDriver.Init();
    delay(10);

    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
        LegPosX[LegIndex] = (short)(cInitPosX[LegIndex]);
        LegPosY[LegIndex] = (short)(cInitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)(cInitPosZ[LegIndex]);
    }

    ResetLegInitAngles();

    g_InControlState.SelectedLeg = 255;
    PrevSelectedLeg = 255;

    g_InControlState.BodyPos.x = 0;
    g_InControlState.BodyPos.y = 0;
    g_InControlState.BodyPos.z = 0;

    g_InControlState.BodyRot1.x = 0;
    g_InControlState.BodyRot1.y = 0;
    g_InControlState.BodyRot1.z = 0;
    g_InControlState.BodyRotOffset.x = 0;
    g_InControlState.BodyRotOffset.y = 0;
    g_InControlState.BodyRotOffset.z = 0;

    g_InControlState.GaitType = 0;
    g_InControlState.BalanceMode = 0;
    g_InControlState.LegLiftHeight = 50;
    g_InControlState.ForceGaitStepCnt = 0;
    g_InControlState.GaitStep = 1;
    GaitSelect();

    g_WifiInputController.Init();

    ServoMoveTime = SERVO_MOVE_TIME;
    g_InControlState.fRobotOn = 0;
    g_fLowVoltageShutdown = false;

    AllDown = true;

    extern std::thread g_camera_thread;  // defined in Hexapod_Main.cpp
    g_camera_thread = std::thread(hexapod_rpicam_main, 0, nullptr);
    // NOT detached — joined in main() after cleanup() + hexapod_rpicam_stop()
}

//================================================================================================
// Loop: de hoofdfunctie voor Arduino main loop
//================================================================================================
void loop(void) {
    unsigned long lTimeWaitEnd;  // Eindtijd voor wachtperiodes

    lTimerStart = millis();
    DoBackgroundProcess();

    updateLEDState();    

    CheckVoltage();
    if (!g_fLowVoltageShutdown) {
        g_WifiInputController.ControlInput();
    }

    if (g_InControlState.fRobotOn || g_InControlState.fPrev_RobotOn) {
        if (g_InControlState.fControllerInUse) {
            g_cNoServoChanged = 0;
        } else if (g_cNoServoChanged >= CNT_NO_SERVO_CHANGE_IDLE) {
            DoBackgroundProcess();
            delay(20);
            return;
        }

        SingleLegControl();
        DoBackgroundProcess();

        GaitSeq();
        DoBackgroundProcess();

        TotalTransX = 0;
        TotalTransZ = 0;
        TotalTransY = 0;
        TotalXBal1 = 0;
        TotalYBal1 = 0;
        TotalZBal1 = 0;

        if (g_InControlState.BalanceMode) {
#ifdef DEBUG
            if (g_fDebugOutput) {
                TravelRequest = (abs(g_InControlState.TravelLength.x) > 0) || 
                                (abs(g_InControlState.TravelLength.z) > 0) || 
                                (abs(g_InControlState.TravelLength.y) > 0) || 
                                (g_InControlState.ForceGaitStepCnt != 0) || fWalking;

                DBGSerial.print("T(");
                DBGSerial.print(fWalking, DEC);
                DBGSerial.print(" ");
                DBGSerial.print(g_InControlState.TravelLength.x, DEC);
                DBGSerial.print(",");
                DBGSerial.print(g_InControlState.TravelLength.y, DEC);
                DBGSerial.print(",");
                DBGSerial.print(g_InControlState.TravelLength.z, DEC);
                DBGSerial.print(")");
            }
#endif
            for (LegIndex = 0; LegIndex < (CNT_LEGS / 2); LegIndex++) {
                // OPT #5: DoBackgroundProcess removed from math loop — pure computation, no time gap
                BalCalcOneLeg(-LegPosX[LegIndex] + GaitPosX[LegIndex], 
                              LegPosZ[LegIndex] + GaitPosZ[LegIndex], 
                              (LegPosY[LegIndex] - (short)(cInitPosY[LegIndex])) + GaitPosY[LegIndex], 
                              LegIndex);
            }

            for (LegIndex = (CNT_LEGS / 2); LegIndex < CNT_LEGS; LegIndex++) {
                // OPT #5: DoBackgroundProcess removed from math loop
                BalCalcOneLeg(LegPosX[LegIndex] + GaitPosX[LegIndex], 
                              LegPosZ[LegIndex] + GaitPosZ[LegIndex], 
                              (LegPosY[LegIndex] - (short)(cInitPosY[LegIndex])) + GaitPosY[LegIndex], 
                              LegIndex);
            }
            BalanceBody();
        }

        IKSolution = 0;
        IKSolutionWarning = 0;
        IKSolutionError = 0;

        for (LegIndex = 0; LegIndex < (CNT_LEGS / 2); LegIndex++) {
            // OPT #5: DoBackgroundProcess removed from math loop
            BodyFK(-LegPosX[LegIndex] + g_InControlState.BodyPos.x + GaitPosX[LegIndex] - TotalTransX,
                   LegPosZ[LegIndex] + g_InControlState.BodyPos.z + GaitPosZ[LegIndex] - TotalTransX,
                   LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY,
                   GaitRotY[LegIndex], LegIndex);

            LegIK(LegPosX[LegIndex] - g_InControlState.BodyPos.x + BodyFKPosX - (GaitPosX[LegIndex] - TotalTransX),
                  LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
                  LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ,
                  LegIndex);
        }

        for (LegIndex = (CNT_LEGS / 2); LegIndex < CNT_LEGS; LegIndex++) {
            // OPT #5: DoBackgroundProcess removed from math loop
            BodyFK(LegPosX[LegIndex] - g_InControlState.BodyPos.x + GaitPosX[LegIndex] - TotalTransX,
                   LegPosZ[LegIndex] + g_InControlState.BodyPos.z + GaitPosZ[LegIndex] - TotalTransZ,
                   LegPosY[LegIndex] + g_InControlState.BodyPos.y + GaitPosY[LegIndex] - TotalTransY,
                   GaitRotY[LegIndex], LegIndex);

            LegIK(LegPosX[LegIndex] + g_InControlState.BodyPos.x - BodyFKPosX + GaitPosX[LegIndex] - TotalTransX,
                  LegPosY[LegIndex] + g_InControlState.BodyPos.y - BodyFKPosY + GaitPosY[LegIndex] - TotalTransY,
                  LegPosZ[LegIndex] + g_InControlState.BodyPos.z - BodyFKPosZ + GaitPosZ[LegIndex] - TotalTransZ,
                  LegIndex);
        }

        CheckAngles();
    }

    if (g_InControlState.fRobotOn) {
        if (!g_InControlState.fPrev_RobotOn) {
          
        showFeedbackLED(1, 500);

        }

        if (g_fHeightTransition) {
            ServoMoveTime = 600; // Set 600 for up/down transitions
            g_fHeightTransition = false; // Reset flag after use
        } else if ((abs(g_InControlState.TravelLength.x) > 0) || 
                   (abs(g_InControlState.TravelLength.z) > 0) || 
                   (abs(g_InControlState.TravelLength.y * 2) > 0)) {
            ServoMoveTime = g_InControlState.gaitCur.NomGaitSpeed + (g_InControlState.InputTimeDelay * 2) + g_InControlState.SpeedControl;
            if (g_InControlState.BalanceMode) {
                ServoMoveTime += BALANCE_DELAY;
            }
        } else {
            ServoMoveTime = SERVO_MOVE_TIME + g_InControlState.SpeedControl;
        }

        DoBackgroundProcess();
        StartUpdateServos();

        for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
            if ((GaitPosX[LegIndex] > cGPlimit) || (GaitPosX[LegIndex] < -cGPlimit) ||
                (GaitPosZ[LegIndex] > cGPlimit) || (GaitPosZ[LegIndex] < -cGPlimit) ||
                (GaitRotY[LegIndex] > cGPlimit) || (GaitRotY[LegIndex] < -cGPlimit)) {
                bExtraCycle = g_InControlState.gaitCur.NrLiftedPos + 1;
                break;
            }
        }

        if (bExtraCycle > 0) {
            bExtraCycle--;
            fWalking = (bExtraCycle != 0);
            lTimeWaitEnd = lTimerStart + PrevServoMoveTime;

#ifdef OPT_BACKGROUND_PROCESS
            DebugWrite(A1, HIGH);
            do {
                DoBackgroundProcess();
                delayMicroseconds(100);
            } while (millis() < lTimeWaitEnd);
            DebugWrite(A1, LOW);
#else
            long lDelay = max(lTimeWaitEnd - millis(), 1);
            delay(lDelay);
#endif
        } else {
            delay(max(lTimerStart + PrevServoMoveTime - millis(), 1));
        }

        DebugToggle(A2);

        if (g_ServoDriver.CommitServoDriver(ServoMoveTime)) {
            g_cNoServoChanged = 0;
        } else if (g_cNoServoChanged < CNT_NO_SERVO_CHANGE_IDLE) {
            g_cNoServoChanged++;
        }
    } else {
        if (g_InControlState.fPrev_RobotOn || !AllDown) {
            ServoMoveTime = 600; // Use 600 when turning off
            StartUpdateServos();
            g_ServoDriver.CommitServoDriver(ServoMoveTime);
           
            showFeedbackLED(1, 500);

            lTimeWaitEnd = millis() + 600;
            do {
                DoBackgroundProcess();
                delay(1);
            } while (millis() < lTimeWaitEnd);
        } else {
            g_ServoDriver.FreeServos();
        }
    }

    // OPT #14: Skip servo LED idle animation during active walking (reduces serial bus contention)
    if (!fWalking)
        g_ServoDriver.IdleTime();
    if (g_fPrintEnabled)
        fflush(stdout);

    // OPT #6: Only delay when robot is off (idle CPU saving)
    // When robot is active, the loop is already paced by PrevServoMoveTime.
    if (!g_InControlState.fRobotOn)
        delay(20);

    PrevServoMoveTime = ServoMoveTime;

    if (g_InControlState.fRobotOn) {
        g_InControlState.fPrev_RobotOn = 1;
    } else {
        g_InControlState.fPrev_RobotOn = 0;
    }
}

//==================================================================
// [StartUpdateServos] Start de servo-update
//==================================================================
void StartUpdateServos() {
    g_ServoDriver.BeginServoUpdate();

    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
        g_ServoDriver.OutputServoInfoForLeg(LegIndex, 
            cCoxaInv[LegIndex] ? -CoxaAngle1[LegIndex] : CoxaAngle1[LegIndex], 
            cFemurInv[LegIndex] ? -FemurAngle1[LegIndex] : FemurAngle1[LegIndex], 
            cTibiaInv[LegIndex] ? -TibiaAngle1[LegIndex] : TibiaAngle1[LegIndex]);   
    }
        // === NIEUWE CODE: Update turret (camera) servos gebaseerd op body rotaties in specifieke modi ===

        // PAN_SERVO volgt BodyRot1.x (X-rotatie, pitch)
        // TILT_SERVO volgt BodyRot1.y (Y-rotatie, yaw)
        // Gebruik negatief teken als je de richting wilt omkeren (bijv. voor compensatie)
        short sTilt = 3 * g_InControlState.BodyRot1.x;  // Beslisgraden (decidegrees)
        short sPan = - 1.2 * g_InControlState.BodyRot1.y;   // Beslisgraden (decidegrees)

        g_ServoDriver.OutputServoInfoForTurret(sTilt, sPan);
        // === EINDE NIEUWE CODE ===
}

//==================================================================
// [CHECK VOLTAGE] Leest de ingangsspanning en schakelt de robot uit bij lage spanning
//==================================================================
byte s_bLVBeepCnt;
boolean CheckVoltage() {
#ifdef cTurnOffVol
    Voltage = g_ServoDriver.GetBatteryVoltage();
    PRINT("Current Voltage: %d\n", Voltage);

    if (!g_fLowVoltageShutdown) {
        if (Voltage < cTurnOffVol) {
            g_InControlState.BodyPos.x = 0;
            g_InControlState.BodyPos.y = 0;
            g_InControlState.BodyPos.z = 0;
            g_InControlState.BodyRot1.x = 0;
            g_InControlState.BodyRot1.y = 0;
            g_InControlState.BodyRot1.z = 0;
            g_InControlState.TravelLength.x = 0;
            g_InControlState.TravelLength.z = 0;
            g_InControlState.TravelLength.y = 0;
            g_InControlState.SelectedLeg = 255;
            g_fLowVoltageShutdown = 1;
            s_bLVBeepCnt = 0;
            g_InControlState.fRobotOn = false;
            PRINT("Voltage too low, hexapod shut down ");
            PRINT("Current Voltage: %d\n", Voltage);
        }
#ifdef cTurnOnVol
    } else if (Voltage > cTurnOnVol) {
        g_fLowVoltageShutdown = 0;
        PRINT("Voltage restored, hexapod back up: ");
        PRINT("Current Voltage: %d\n", Voltage);
#endif
    } else {
        if (s_bLVBeepCnt < 5) {
            s_bLVBeepCnt++;
            PRINT("Current Voltage: %d\n", Voltage);      
            showFeedbackLED(1, 500);           
        }
        delay(2000);
    }
#endif
    return g_fLowVoltageShutdown;
}

//==================================================================
// [SINGLE LEG CONTROL] Besturing van een enkele poot
//==================================================================
void SingleLegControl(void) {
    AllDown = (LegPosY[cRF] == (short)(cInitPosY[cRF])) &&
              (LegPosY[cRR] == (short)(cInitPosY[cRR])) &&
              (LegPosY[cLR] == (short)(cInitPosY[cLR])) &&
              (LegPosY[cRM] == (short)(cInitPosY[cRM])) &&
              (LegPosY[cLM] == (short)(cInitPosY[cLM])) &&
              (LegPosY[cLF] == (short)(cInitPosY[cLF]));

    if (g_InControlState.SelectedLeg <= (CNT_LEGS - 1)) {
        if (g_InControlState.SelectedLeg != PrevSelectedLeg) {
            if (AllDown) {
                LegPosY[g_InControlState.SelectedLeg] = (short)(cInitPosY[g_InControlState.SelectedLeg]) - 20;
                PrevSelectedLeg = g_InControlState.SelectedLeg;
            } else {
                LegPosX[PrevSelectedLeg] = (short)(cInitPosX[PrevSelectedLeg]);
                LegPosY[PrevSelectedLeg] = (short)(cInitPosY[PrevSelectedLeg]);
                LegPosZ[PrevSelectedLeg] = (short)(cInitPosZ[PrevSelectedLeg]);
            }
        } else if (!g_InControlState.fSLHold) {
            LegPosY[g_InControlState.SelectedLeg] = (short)(cInitPosY[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.y;
            LegPosX[g_InControlState.SelectedLeg] = (short)(cInitPosX[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.x;
            LegPosZ[g_InControlState.SelectedLeg] = (short)(cInitPosZ[g_InControlState.SelectedLeg]) + g_InControlState.SLLeg.z;
        }
    } else {
        if (!AllDown) {
            for (LegIndex = 0; LegIndex <= (CNT_LEGS - 1); LegIndex++) {
                LegPosX[LegIndex] = (short)(cInitPosX[LegIndex]);
                LegPosY[LegIndex] = (short)(cInitPosY[LegIndex]);
                LegPosZ[LegIndex] = (short)(cInitPosZ[LegIndex]);
            }
        }
//        if (PrevSelectedLeg != 255) {
            PrevSelectedLeg = 255; //veranderd
//        }
    }
}

//==================================================================
// [GaitSelect] Selecteer de gait
//==================================================================
void GaitSelect(void) {
    if (g_InControlState.GaitType < NUM_GAITS) {
#ifdef ADD_GAITS
        if (g_InControlState.GaitType < (sizeof(APG_EXTRA) / sizeof(APG_EXTRA[0]))) {
            g_InControlState.gaitCur = APG_EXTRA[g_InControlState.GaitType];
        } else {
            g_InControlState.gaitCur = APG[g_InControlState.GaitType - (sizeof(APG_EXTRA) / sizeof(APG_EXTRA[0]))];
        }
#else
        g_InControlState.gaitCur = APG[g_InControlState.GaitType];
#endif
    }
}

//==================================================================
// [GAIT Sequence] Bereken de gait-sequentie
//==================================================================
void GaitSeq(void) {
    if (fWalking || (g_InControlState.ForceGaitStepCnt != 0)) {
        TravelRequest = true;
    } else {
        TravelRequest = (abs(g_InControlState.TravelLength.x) > 0) ||
                        (abs(g_InControlState.TravelLength.z) > 0) ||
                        (abs(g_InControlState.TravelLength.y) > 0);

        if (!TravelRequest) {
            // Only reset TravelLength if no forced steps remain
            if (g_InControlState.ForceGaitStepCnt == 0) {
                g_InControlState.TravelLength.x = 0;
                g_InControlState.TravelLength.z = 0;
                g_InControlState.TravelLength.y = 0;
            }
        }
    }

    // Pre-compute loop-invariant divisions once for all 6 Gait() calls
    s_tlx_ldf   = g_InControlState.TravelLength.x / g_InControlState.gaitCur.LiftDivFactor;
    s_tlz_ldf   = g_InControlState.TravelLength.z / g_InControlState.gaitCur.LiftDivFactor;
    s_tly_ldf   = g_InControlState.TravelLength.y / g_InControlState.gaitCur.LiftDivFactor;
    s_tlx_tldf  = g_InControlState.TravelLength.x / (short)g_InControlState.gaitCur.TLDivFactor;
    s_tlz_tldf  = g_InControlState.TravelLength.z / (short)g_InControlState.gaitCur.TLDivFactor;
    s_tly_tldf  = g_InControlState.TravelLength.y / (short)g_InControlState.gaitCur.TLDivFactor;
    s_lift_mid  = -3 * g_InControlState.LegLiftHeight / (3 + g_InControlState.gaitCur.HalfLiftHeight);

    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
        Gait(LegIndex);
    }

    g_InControlState.GaitStep++;
    if (g_InControlState.GaitStep > g_InControlState.gaitCur.StepsInGait) {
        g_InControlState.GaitStep = 1;
    }

    if (g_InControlState.ForceGaitStepCnt > 0) {
        g_InControlState.ForceGaitStepCnt--;
    }
}

//==================================================================
// [GAIT] Bereken de gait voor een specifieke poot
//==================================================================
void Gait(byte GaitCurrentLegNr) {
    short int LegStep = g_InControlState.GaitStep - g_InControlState.gaitCur.GaitLegNr[GaitCurrentLegNr];

    if ((TravelRequest && (g_InControlState.gaitCur.NrLiftedPos & 1) && LegStep == 0) ||
        (!TravelRequest && LegStep == 0 && ((abs(GaitPosX[GaitCurrentLegNr]) > 2) ||
                                            (abs(GaitPosZ[GaitCurrentLegNr]) > 2) ||
                                            (abs(GaitRotY[GaitCurrentLegNr]) > 2)))) {
        GaitPosX[GaitCurrentLegNr] = 0;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
        GaitPosZ[GaitCurrentLegNr] = 0;
        GaitRotY[GaitCurrentLegNr] = 0;
    } else if (((g_InControlState.gaitCur.NrLiftedPos == 2 && LegStep == 0) ||
                (g_InControlState.gaitCur.NrLiftedPos >= 3 &&
                 (LegStep == -1 || LegStep == (g_InControlState.gaitCur.StepsInGait - 1)))) &&
               TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = -s_tlx_ldf;
        GaitPosY[GaitCurrentLegNr] = s_lift_mid;
        GaitPosZ[GaitCurrentLegNr] = -s_tlz_ldf;
        GaitRotY[GaitCurrentLegNr] = -s_tly_ldf;
    } else if ((g_InControlState.gaitCur.NrLiftedPos >= 2) &&
               (LegStep == 1 || LegStep == -(g_InControlState.gaitCur.StepsInGait - 1)) &&
               TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = s_tlx_ldf;
        GaitPosY[GaitCurrentLegNr] = s_lift_mid;
        GaitPosZ[GaitCurrentLegNr] = s_tlz_ldf;
        GaitRotY[GaitCurrentLegNr] = s_tly_ldf;
    } else if ((g_InControlState.gaitCur.NrLiftedPos == 5 && LegStep == -2) && TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x / 2;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight / 2;
        GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z / 2;
        GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y / 2;
    } else if ((g_InControlState.gaitCur.NrLiftedPos == 5) &&
               (LegStep == 2 || LegStep == -(g_InControlState.gaitCur.StepsInGait - 2)) &&
               TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x / 2;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight / 2;
        GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z / 2;
        GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y / 2;
    } else if ((LegStep == g_InControlState.gaitCur.FrontDownPos ||
                LegStep == -(g_InControlState.gaitCur.StepsInGait - g_InControlState.gaitCur.FrontDownPos)) &&
               GaitPosY[GaitCurrentLegNr] < 0) {
        GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x / 2;
        GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z / 2;
        GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y / 2;
        GaitPosY[GaitCurrentLegNr] = 0;
    } else {
        GaitPosX[GaitCurrentLegNr] -= s_tlx_tldf;
        GaitPosY[GaitCurrentLegNr] = 0;
        GaitPosZ[GaitCurrentLegNr] -= s_tlz_tldf;
        GaitRotY[GaitCurrentLegNr] -= s_tly_tldf;
    }
}

//==================================================================
// [BalCalcOneLeg] Bereken balans voor één poot
//==================================================================
void BalCalcOneLeg(long PosX, long PosZ, long PosY, byte BalLegNr) {
    long CPR_X;  // Eind X-waarde voor rotatie-middelpunt
    long CPR_Y;  // Eind Y-waarde voor rotatie-middelpunt
    long CPR_Z;  // Eind Z-waarde voor rotatie-middelpunt
    long lAtan;

    CPR_Z = (short)(cOffsetZ[BalLegNr]) + PosZ;
    CPR_X = (short)(cOffsetX[BalLegNr]) + PosX;
    CPR_Y = 150 + PosY;

    TotalTransY += (long)PosY;
    TotalTransZ += (long)CPR_Z;
    TotalTransX += (long)CPR_X;

    lAtan = GetATan2(CPR_X, CPR_Z);
    TotalYBal1 += (lAtan * 1800) / 31415;

#ifdef DEBUG
    if (g_fDebugOutput) {
        DBGSerial.print(" ");
        DBGSerial.print(CPR_X, DEC);
        DBGSerial.print(":");
        DBGSerial.print(CPR_Y, DEC);
        DBGSerial.print(":");
        DBGSerial.print(CPR_Z, DEC);
        DBGSerial.print(":");
        DBGSerial.print(TotalYBal1, DEC);
    }    
#endif

    lAtan = GetATan2(CPR_X, CPR_Y);
    TotalZBal1 += ((lAtan * 1800) / 31415) - 900;

    lAtan = GetATan2(CPR_Z, CPR_Y);
    TotalXBal1 += ((lAtan * 1800) / 31415) - 900;
}

//==================================================================
// [BalanceBody] Balans van het lichaam aanpassen
//==================================================================
void BalanceBody(void) {
    TotalTransZ /= BalanceDivFactor;
    TotalTransX /= BalanceDivFactor;
    TotalTransY /= BalanceDivFactor;

    if (TotalYBal1 > 0) {
        TotalYBal1 -= 1800;
    } else {
        TotalYBal1 += 1800;
    }       

    if (TotalZBal1 < -1800) {
        TotalZBal1 += 3600;
    }

    if (TotalXBal1 < -1800) {
        TotalXBal1 += 3600;
    }

    TotalYBal1 = -TotalYBal1 / BalanceDivFactor;
    TotalXBal1 = -TotalXBal1 / BalanceDivFactor;
    TotalZBal1 = TotalZBal1 / BalanceDivFactor;

#ifdef DEBUG
    if (g_fDebugOutput) {
        DBGSerial.print(" L ");
        DBGSerial.print(BalanceDivFactor, DEC);
        DBGSerial.print(" TTrans: ");
        DBGSerial.print(TotalTransX, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(TotalTransY, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(TotalTransZ, DEC);
        DBGSerial.print(" TBal: ");
        DBGSerial.print(TotalXBal1, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(TotalYBal1, DEC);
        DBGSerial.print(" ");
        DBGSerial.println(TotalZBal1, DEC);
    }
#endif
}

//==================================================================
// [GETSINCOS] Haal sinus en cosinus op van een hoek
//==================================================================
void GetSinCos(short AngleDeg1) {
    short ABSAngleDeg1;

    if (AngleDeg1 < 0) {
        // Guard against INT16_MIN: -(-32768) overflows short; clamp to INT16_MAX
        ABSAngleDeg1 = (AngleDeg1 == -32768) ? 32767 : (short)(AngleDeg1 * -1);
    } else {
        ABSAngleDeg1 = AngleDeg1;
    }

    if (AngleDeg1 < 0) {
        AngleDeg1 = 3600 - (ABSAngleDeg1 % 3600);
    } else {
        AngleDeg1 = ABSAngleDeg1 % 3600;
    }

    if (AngleDeg1 >= 0 && AngleDeg1 <= 900) {
        sin4 = GetSin[AngleDeg1 / 5];
        cos4 = GetSin[(900 - AngleDeg1) / 5];
    } else if (AngleDeg1 > 900 && AngleDeg1 <= 1800) {
        sin4 = GetSin[(900 - (AngleDeg1 - 900)) / 5];
        cos4 = -GetSin[(AngleDeg1 - 900) / 5];
    } else if (AngleDeg1 > 1800 && AngleDeg1 <= 2700) {
        sin4 = -GetSin[(AngleDeg1 - 1800) / 5];
        cos4 = -GetSin[(2700 - AngleDeg1) / 5];
    } else if (AngleDeg1 > 2700 && AngleDeg1 <= 3600) {
        sin4 = -GetSin[(3600 - AngleDeg1) / 5];
        cos4 = GetSin[(AngleDeg1 - 2700) / 5];
    }
}

//==================================================================
// [GETARCCOS] Haal de arccosinus op van een waarde
//==================================================================
long GetArcCos(short cos4) {
    boolean NegativeValue;

    if (cos4 < 0) {
        cos4 = -cos4;
        NegativeValue = 1;
    } else {
        NegativeValue = 0;
    }

    cos4 = min(cos4, c4DEC);

    if (cos4 >= 0 && cos4 < 9000) {
        AngleRad4 = (byte)(GetACos[cos4 / 79]);
        AngleRad4 = ((long)AngleRad4 * 616) / c1DEC;
    } else if (cos4 >= 9000 && cos4 < 9900) {
        AngleRad4 = (byte)(GetACos[(cos4 - 9000) / 8 + 114]);
        AngleRad4 = (long)((long)AngleRad4 * 616) / c1DEC;
    } else if (cos4 >= 9900 && cos4 <= 10000) {
        AngleRad4 = (byte)(GetACos[(cos4 - 9900) / 2 + 227]);
        AngleRad4 = (long)((long)AngleRad4 * 616) / c1DEC;
    }

    if (NegativeValue) {
        AngleRad4 = 31416 - AngleRad4;
    }

    return AngleRad4;
}

//==================================================================
// [isqrt32] Bereken de vierkantswortel van een 32-bit getal
//==================================================================
// OPT #7: Integer Newton's method — avoids int→double→sqrt→long conversion
unsigned long isqrt32(unsigned long n) {
    if (n == 0) return 0;
    unsigned long x = n;
    unsigned long y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

//==================================================================
// [GETATAN2] Vereenvoudigde ArcTan2-functie gebaseerd op vaste-punt ArcCos
//==================================================================
short GetATan2(short AtanX, short AtanY) {
    XYhyp2 = isqrt32(((long)AtanX * AtanX * c4DEC) + ((long)AtanY * AtanY * c4DEC));
    GetArcCos(((long)AtanX * (long)c6DEC) / (long)XYhyp2);

    if (AtanY < 0) {
        Atan4 = -AngleRad4;
    } else {
        Atan4 = AngleRad4;
    }

    return Atan4;
}

//==================================================================
// [BODY INVERSE KINEMATICS] Bereken de lichaamspositie met rotatie
//==================================================================
void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) {
    short SinA4, CosA4, SinB4, CosB4, SinG4, CosG4;
    short CPR_X, CPR_Y, CPR_Z;

    CPR_X = (short)(cOffsetX[BodyIKLeg]) + PosX + g_InControlState.BodyRotOffset.x;
    CPR_Y = PosY + g_InControlState.BodyRotOffset.y;
    CPR_Z = (short)(cOffsetZ[BodyIKLeg]) + PosZ + g_InControlState.BodyRotOffset.z;

    GetSinCos(g_InControlState.BodyRot1.x + TotalXBal1);
    SinG4 = sin4;
    CosG4 = cos4;

    GetSinCos(g_InControlState.BodyRot1.z + TotalZBal1);
    SinB4 = sin4;
    CosB4 = cos4;
    
    GetSinCos(g_InControlState.BodyRot1.y + (RotationY * c1DEC) + TotalYBal1);
    SinA4 = sin4;
    CosA4 = cos4;

    BodyFKPosX = ((long)CPR_X * c2DEC - ((long)CPR_X * c2DEC * CosA4 / c4DEC * CosB4 / c4DEC - 
                                         (long)CPR_Z * c2DEC * CosB4 / c4DEC * SinA4 / c4DEC + 
                                         (long)CPR_Y * c2DEC * SinB4 / c4DEC)) / c2DEC;
    BodyFKPosZ = ((long)CPR_Z * c2DEC - ((long)CPR_X * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC + 
                                         (long)CPR_X * c2DEC * CosA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC + 
                                         (long)CPR_Z * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC - 
                                         (long)CPR_Z * c2DEC * SinA4 / c4DEC * SinB4 / c4DEC * SinG4 / c4DEC - 
                                         (long)CPR_Y * c2DEC * CosB4 / c4DEC * SinG4 / c4DEC)) / c2DEC;
    BodyFKPosY = ((long)CPR_Y * c2DEC - ((long)CPR_X * c2DEC * SinA4 / c4DEC * SinG4 / c4DEC - 
                                         (long)CPR_X * c2DEC * CosA4 / c4DEC * CosG4 / c4DEC * SinB4 / c4DEC + 
                                         (long)CPR_Z * c2DEC * CosA4 / c4DEC * SinG4 / c4DEC + 
                                         (long)CPR_Z * c2DEC * CosG4 / c4DEC * SinA4 / c4DEC * SinB4 / c4DEC + 
                                         (long)CPR_Y * c2DEC * CosB4 / c4DEC * CosG4 / c4DEC)) / c2DEC;
}

//==================================================================
// [LEG INVERSE KINEMATICS] Bereken de hoeken van coxa, femur en tibia
//==================================================================
void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr) {
    unsigned long IKSW2, IKA14, IKA24;
    short IKFeetPosXZ;

#define TarsOffsetXZ 0
#define TarsOffsetY  0
    long Temp1, Temp2, T3;

    GetATan2(IKFeetPosX, IKFeetPosZ);
    CoxaAngle1[LegIKLegNr] = (((long)Atan4 * 180) / 3141) + (short)(cCoxaAngle1[LegIKLegNr]);

    IKFeetPosXZ = XYhyp2 / c2DEC;

    IKA14 = GetATan2(IKFeetPosY - TarsOffsetY, IKFeetPosXZ - (byte)(cCoxaLength[LegIKLegNr]) - TarsOffsetXZ);
    IKSW2 = XYhyp2;

    Temp1 = ((((long)(byte)(cFemurLength[LegIKLegNr]) * (byte)(cFemurLength[LegIKLegNr])) - 
              ((long)(byte)(cTibiaLength[LegIKLegNr]) * (byte)(cTibiaLength[LegIKLegNr]))) * c4DEC + 
             ((long)IKSW2 * IKSW2));
    Temp2 = (long)(2 * (byte)(cFemurLength[LegIKLegNr])) * c2DEC * (unsigned long)IKSW2;
    if (Temp2 == 0 || (Temp2 / c4DEC) == 0) return;  // guard actual divisor, not just Temp2
    T3 = Temp1 / (Temp2 / c4DEC);
    IKA24 = GetArcCos(T3);

#ifdef DEBUG_IK
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print(" ");
        DBGSerial.print(Temp1, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(Temp2, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(T3, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(IKSW2, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(IKA14, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(IKA24, DEC);
    }
#endif

    FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900 + CFEMURHORNOFFSET1(LegIKLegNr);
    Temp1 = ((((long)(byte)(cFemurLength[LegIKLegNr]) * (byte)(cFemurLength[LegIKLegNr])) + 
              ((long)(byte)(cTibiaLength[LegIKLegNr]) * (byte)(cTibiaLength[LegIKLegNr]))) * c4DEC - 
             ((long)IKSW2 * IKSW2));
    Temp2 = 2 * ((long)((byte)(cFemurLength[LegIKLegNr]))) * (long)((byte)(cTibiaLength[LegIKLegNr]));
    if (Temp2 == 0) return;  // both femur and tibia lengths are non-zero in practice
    GetArcCos(Temp1 / Temp2);

#ifdef DEBUG_IK
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print("=");
        DBGSerial.print(Temp1, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(Temp2, DEC);
        DBGSerial.print(" ");
        DBGSerial.print(AngleRad4, DEC);
    }
#endif

    TibiaAngle1[LegIKLegNr] = -(900 - (long)AngleRad4 * 180 / 3141 + CTIBIAHORNOFFSET1(LegIKLegNr));

    if (IKSW2 < ((word)((byte)(cFemurLength[LegIKLegNr]) + (byte)(cTibiaLength[LegIKLegNr]) - 30) * c2DEC)) {
        IKSolution = 1;
    } else if (IKSW2 < ((word)((byte)(cFemurLength[LegIKLegNr]) + (byte)(cTibiaLength[LegIKLegNr])) * c2DEC)) {
        IKSolutionWarning = 1;
    } else {
        IKSolutionError = 1;
    }

#ifdef DEBUG
    if (g_fDebugOutput && g_InControlState.fRobotOn) {
        DBGSerial.print("(");
        DBGSerial.print(IKFeetPosX, DEC);
        DBGSerial.print(",");
        DBGSerial.print(IKFeetPosY, DEC);
        DBGSerial.print(",");
        DBGSerial.print(IKFeetPosZ, DEC);
        DBGSerial.print(")=<");
        DBGSerial.print(CoxaAngle1[LegIKLegNr], DEC);
        DBGSerial.print(",");
        DBGSerial.print(FemurAngle1[LegIKLegNr], DEC);
        DBGSerial.print(",");
        DBGSerial.print(TibiaAngle1[LegIKLegNr], DEC);
        DBGSerial.print(">");
        DBGSerial.print((IKSolutionError << 2) + (IKSolutionWarning << 1) + IKSolution, DEC);
        if (LegIKLegNr == (CNT_LEGS - 1)) {
            DBGSerial.println();
        }
    }
#endif  
}

//==================================================================
// [CheckServoAngleBounds] Controleer servo-hoekgrenzen
//==================================================================
short CheckServoAngleBounds(short sID, short sVal, const short sMin, const short sMax) {
    if (sVal < sMin) {
#ifdef DEBUG
        if (g_fDebugOutput) {
            DBGSerial.print(sID, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(sVal, DEC);
            DBGSerial.print("<");
            DBGSerial.println(sMin, DEC);
        }
#endif
        return sMin;
    }

    if (sVal > sMax) {
#ifdef DEBUG
        if (g_fDebugOutput) {
            DBGSerial.print(sID, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(sVal, DEC);
            DBGSerial.print(">");
            DBGSerial.println(sMax, DEC);
        }
#endif
        return sMax;
    }

    return sVal;
}

//==================================================================
// [CHECK ANGLES] Controleer mechanische limieten van de servo's
//==================================================================
void CheckAngles(void) {
    short s = 0;
    for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
        CoxaAngle1[LegIndex] = CheckServoAngleBounds(s++, CoxaAngle1[LegIndex], cCoxaMin1[LegIndex], cCoxaMax1[LegIndex]);
        FemurAngle1[LegIndex] = CheckServoAngleBounds(s++, FemurAngle1[LegIndex], cFemurMin1[LegIndex], cFemurMax1[LegIndex]);
        TibiaAngle1[LegIndex] = CheckServoAngleBounds(s++, TibiaAngle1[LegIndex], cTibiaMin1[LegIndex], cTibiaMax1[LegIndex]);
    }
}

//==================================================================
// [SmoothControl] Maakt rotatie en translatie van het lichaam vloeiender
//==================================================================
double SmoothControl(double CtrlMoveInp, double CtrlMoveOut, double CtrlDivider, double SmMrgn) {
    double inp = CtrlMoveInp;
    double out = CtrlMoveOut;
    double div = CtrlDivider;
    double margin = SmMrgn;

    // Snap from idle (output ~0) to first non-zero target so startup reaction is instant
    if (fabs(out) < 0.01 && fabs(inp) > 0.01)
        return inp;

    if (out < inp - margin) {
        out = out + fabs(out - inp) / div;
    } else if (out > inp + margin) {
        out = out - fabs(out - inp) / div;
    } else {
        out = inp;
    }

    return out;
}

//==================================================================
// [GetLegsXZLength] Bereken de XZ-lengte van de poten
//==================================================================
word g_wLegsXZLength = 0xffff;
word GetLegsXZLength(void) {
    if (g_wLegsXZLength != 0xffff) {
        return g_wLegsXZLength;
    }
    g_wLegsXZLength = (word)isqrt32((LegPosX[0] * LegPosX[0]) + (LegPosZ[0] * LegPosZ[0]));
    return g_wLegsXZLength;
}

//==================================================================
// [AdjustLegPositions] Pas de initiële posities van de poten aan
//==================================================================
#ifndef MIN_XZ_LEG_ADJUST 
#define MIN_XZ_LEG_ADJUST (cCoxaLength[0])
#endif

#ifndef MAX_XZ_LEG_ADJUST
#define MAX_XZ_LEG_ADJUST (cCoxaLength[0] + cTibiaLength[0] + cFemurLength[0] / 4)
#endif

void AdjustLegPositions(word XZLength1) {
    if (XZLength1 > MAX_XZ_LEG_ADJUST) {
        XZLength1 = MAX_XZ_LEG_ADJUST;
    }
    if (XZLength1 < MIN_XZ_LEG_ADJUST) {
        XZLength1 = MIN_XZ_LEG_ADJUST;
    }

    if (XZLength1 == g_wLegsXZLength) {
        return;
    }

    g_wLegsXZLength = XZLength1;

    for (uint8_t legIdx = 0; legIdx < CNT_LEGS; legIdx++) {
#ifdef DEBUG
        if (g_fDebugOutput) {
            DBGSerial.print("(");
            DBGSerial.print(LegPosX[legIdx], DEC);  
            DBGSerial.print(",");
            DBGSerial.print(LegPosZ[legIdx], DEC);  
            DBGSerial.print(")->");
        }
#endif
#ifdef ADJUSTABLE_LEG_ANGLES
        GetSinCos(g_InControlState.aCoxaInitAngle1[legIdx]);  
#else
#ifdef cRRInitCoxaAngle1
        GetSinCos((short)(cCoxaInitAngle1[legIdx]));  
#else
        GetSinCos((short)(cCoxaAngle1[legIdx]));  
#endif
#endif
        LegPosX[legIdx] = ((long)((long)cos4 * XZLength1)) / c4DEC;
        LegPosZ[legIdx] = -((long)((long)sin4 * XZLength1)) / c4DEC;
#ifdef DEBUG
        if (g_fDebugOutput) {
            DBGSerial.print("(");
            DBGSerial.print(LegPosX[legIdx], DEC);  
            DBGSerial.print(",");
            DBGSerial.print(LegPosZ[legIdx], DEC);  
            DBGSerial.print(") ");
        }
#endif
    }

#ifdef DEBUG
    if (g_fDebugOutput) {
        DBGSerial.println("");
    }
#endif

    g_InControlState.ForceGaitStepCnt = g_InControlState.gaitCur.StepsInGait;
}

//==================================================================
// [ResetLegInitAngles] Reset de initiële hoeken van de poten
//==================================================================
void ResetLegInitAngles(void) {
#ifdef ADJUSTABLE_LEG_ANGLES
    for (int legIdx = 0; legIdx < CNT_LEGS; legIdx++) {
#ifdef cRRInitCoxaAngle1
        g_InControlState.aCoxaInitAngle1[legIdx] += iDeltaAngle;
#else
        g_InControlState.aCoxaInitAngle1[legIdx] -= iDeltaAngle;
#endif
    }
    g_wLegsXZLength = 0xffff;
#endif
}

//==================================================================
// [RotateLegInitAngles] Roteer de initiële hoeken van de poten
//==================================================================
void RotateLegInitAngles(int iDeltaAngle) {
#ifdef ADJUSTABLE_LEG_ANGLES
    for (int legIdx = 0; legIdx < CNT_LEGS; legIdx++) {
        if ((short)(cCoxaAngle1[legIdx]) > 0) {
            g_InControlState.aCoxaInitAngle1[legIdx] += iDeltaAngle;
        } else if ((short)(cCoxaAngle1[legIdx]) < 0) {
            g_InControlState.aCoxaInitAngle1[legIdx] -= iDeltaAngle;
        }

        if (g_InControlState.aCoxaInitAngle1[legIdx] > 700) {
            g_InControlState.aCoxaInitAngle1[legIdx] = 700;
        } else if (g_InControlState.aCoxaInitAngle1[legIdx] < -700) {
            g_InControlState.aCoxaInitAngle1[legIdx] = -700;
        }
    }
    g_wLegsXZLength = 0xffff;
#endif
}

//==================================================================
// [AdjustLegPositionsToBodyHeight] Pas pootposities aan op lichaamshoogte
//==================================================================
uint8_t g_iLegInitIndex = 0x00;

void AdjustLegPositionsToBodyHeight() {
#ifdef CNT_HEX_INITS
    if (g_InControlState.BodyPos.y > (short)(g_abHexMaxBodyY[CNT_HEX_INITS - 1])) {
        g_InControlState.BodyPos.y = (short)(g_abHexMaxBodyY[CNT_HEX_INITS - 1]);
    }

    uint8_t i;
    word XZLength1 = (g_abHexIntXZ[CNT_HEX_INITS - 1]);
    for (i = 0; i < (CNT_HEX_INITS - 1); i++) {
        if (g_InControlState.BodyPos.y <= (short)(g_abHexMaxBodyY[i])) {
            XZLength1 = (g_abHexIntXZ[i]);
            break;
        }
    }

    if (i != g_iLegInitIndex) {
        g_iLegInitIndex = i;

#ifdef DEBUG
        if (g_fDebugOutput) {
            DBGSerial.print("ALPTBH: ");
            DBGSerial.print(g_InControlState.BodyPos.y, DEC);
            DBGSerial.print(" ");
            DBGSerial.print(XZLength1, DEC);
        }
#endif
        AdjustLegPositions(XZLength1);
    }
#endif
}