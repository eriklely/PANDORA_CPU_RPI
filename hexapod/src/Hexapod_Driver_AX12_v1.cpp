//=======================================================================================
// Servo Driver - Deze versie is ingesteld om een USB2AX te gebruiken om
// Dynamixel servo's zoals de AX-12 of AX-18 te besturen
//=======================================================================================


#include "Hex_Cfg.h"
#include "Hexapod.h"
#include "WrapperSerial.h"

// Servo configuratie
#define NUMSERVOSPERLEG 3  // 3 graden van vrijheid per poot
#define NUMTURRETSERVOS 2

#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS + NUMTURRETSERVOS)    

#ifndef DXL_BAUD
#define DXL_BAUD 1000000L  // Standaard baudrate voor Dynamixel
#endif

#define cPwmMult      128   // PWM multiplier
#define cPwmDiv       375   // PWM deler
#define cPFConst      512   // Halve PWM bereik (1024/2)

#include <ax12.h>

#define USE_BIOLOIDEX            // Gebruik Bioloid code voor AX12 besturing
#define USE_AX12_SPEED_CONTROL   // Experimentele snelheidscontrole
boolean g_fAXSpeedControl;      // Vlag voor snelheidscontrole methode
#include <BioloidEX.h>

#ifdef USE_AX12_SPEED_CONTROL
word g_awCurAXPos[NUMSERVOS];   // Huidige posities in AX coördinaten
word g_awGoalAXPos[NUMSERVOS];  // Doelposities in AX coördinaten
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled (g_fEnableServos)
#else
#define ServosEnabled (true)  // Altijd waar, compiler optimaliseert dit
#endif

//================================================================================================
// Globale variabelen - Lokaal voor dit bestand
//================================================================================================
static const byte cPinTable[] = {
    cRRCoxaPin, cRMCoxaPin, cRFCoxaPin, cLRCoxaPin, cLMCoxaPin, cLFCoxaPin,
    cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
    cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin,
    cCMTiltPin, cCMPanPin
};

#define FIRSTCOXAPIN     0         // Eerste coxa pin index
#define FIRSTFEMURPIN    (CNT_LEGS) // Eerste femur pin index
#define FIRSTTIBIAPIN    (CNT_LEGS*2) // Eerste tibia pin index
#define FIRST_TILT_SERVO (NUMSERVOSPERLEG * CNT_LEGS)      // Index for pan (after all legs)
#define FIRST_PAN_SERVO  (FIRST_TILT_SERVO + 1)             // Index for tilt

BioloidControllerEx bioloid = BioloidControllerEx(DXL_BAUD); // Bioloid controller
boolean g_fServosFree;    // Zijn de servo's vrij?
uint8_t g_id_controller = AX_ID_DEVICE; // Controller ID

extern void MakeSureServosAreOn(void);
extern void SetRegOnAllServos(uint8_t bReg, uint8_t bVal);
extern void DoPyPose(byte *psz);

//--------------------------------------------------------------------
// Initialisatie
//--------------------------------------------------------------------
void ServoDriver::Init(void) {
    g_fServosFree = true;

    // Detecteer servo controller
    word wModel = ax12GetRegister(g_id_controller, AX_MODEL_NUMBER_L, 2);
    if (wModel == 0xffff) {
        g_id_controller = 200; // Probeer CM730 controller
        wModel = ax12GetRegister(g_id_controller, AX_MODEL_NUMBER_L, 2);
    }

    if (wModel != 0xffff)
        PRINT("Controller model %x op %x(%d)\n", wModel, g_id_controller, g_id_controller);
    else {
        PRINT("Controller niet gevonden\n");
        g_id_controller = 0;
    }

    if (g_id_controller == 200)
        ax12SetRegister(g_id_controller, AX_TORQUE_ENABLE, 0x1);

    // Controleer alle servo's
    bioloid.poseSize(NUMSERVOS);
    uint16_t w;
    int count_missing = 0;
    int missing_servo = -1;
    bool servo_1_in_table = false;

    for (int i = 0; i < NUMSERVOS; i++) {
        bioloid.setId(i, cPinTable[i]);
        if (cPinTable[i] == 1) servo_1_in_table = true;

        w = ax12GetRegister(cPinTable[i], AX_PRESENT_POSITION_L, 2);
        if (w == 0xffff) {
            delay(25);
            w = ax12GetRegister(cPinTable[i], AX_PRESENT_POSITION_L, 2);
            if (w == 0xffff) {
                PRINT("Servo(%d): %d niet gevonden\n", i, cPinTable[i]);
                if (++count_missing == 1) missing_servo = cPinTable[i];
            }
            // Experimentele compliance instelling
            ax12SetRegister(cPinTable[i], AX_CW_COMPLIANCE_SLOPE, COMPLIANCE_SLOPE);
            ax12SetRegister(cPinTable[i], AX_CCW_COMPLIANCE_SLOPE, COMPLIANCE_SLOPE);
        }
        delay(25);
    }

    if (count_missing)
        PRINT("FOUT: Servo driver init: %d servo's ontbreken\n", count_missing);

    if (count_missing == 1 && !servo_1_in_table) {
        if (dxl_read_word(1, AX_PRESENT_POSITION_L) != 0xffff) {
            PRINT("Servo herstel: Servo 1 gevonden - ID instellen op %d\n", missing_servo);
            dxl_write_byte(1, AX_ID, missing_servo);
        }
    }

    g_fAXSpeedControl = false;
#ifdef OPT_GPPLAYER
    _fGPEnabled = true;
#endif
}

//--------------------------------------------------------------------
// Opruimen
//--------------------------------------------------------------------
void ServoDriver::Cleanup(void) {
    PRINT("ServoDriver::Opruimen\n\r");
    for (int iServo = 0; iServo < NUMSERVOS; iServo++) {
        dxl_write_byte(cPinTable[iServo], AX_LED, 0);
        dxl_get_result();
    }
    bioloid.end();
}

//--------------------------------------------------------------------
// Spanning opvragen - Minimaliseer oproepen vanwege seriële poort gebruik
//--------------------------------------------------------------------
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 250  // Max 4 keer per seconde
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000 // Minstens 1 keer per seconde
#define VOLTAGE_TIME_TO_ERROR 3000          // Fout na 3 seconden
#define CM730_P_VOLTAGE 50                  // Voltage register voor CM730

word g_wLastVoltage = 0xffff;  // Laatste gemeten spanning
byte g_bLegVoltage = 0;        // Laatst gecontroleerde poot
unsigned long g_ulTimeLastBatteryVoltage = 0;

word ServoDriver::GetBatteryVoltage(void) {
    unsigned long uldt = millis() - g_ulTimeLastBatteryVoltage;
    if (uldt > VOLTAGE_MAX_TIME_BETWEEN_CALLS || 
        (uldt > VOLTAGE_MIN_TIME_BETWEEN_CALLS && !bioloid.interpolating())) {
        word wVoltage;
        if (g_id_controller == 200)
            wVoltage = (word)ax12GetRegister(g_id_controller, CM730_P_VOLTAGE, 1);
        else {
            wVoltage = (word)ax12GetRegister(cPinTable[FIRSTFEMURPIN], AX_PRESENT_VOLTAGE, 1);
            if (g_bLegVoltage == CNT_LEGS) g_bLegVoltage = 0;
        }

        if (wVoltage && wVoltage != 0xffff) {
            g_ulTimeLastBatteryVoltage = millis();
            if (wVoltage != g_wLastVoltage) {
                PRINT("Spanning: %d\n\r", wVoltage);
                g_wLastVoltage = wVoltage;
#ifdef SHOW_COMPLIANCE_SLOPE
                PRINT("Compliance slope: %d\n\r", (ax12GetRegister(1, AX_CW_COMPLIANCE_SLOPE, 32), HEX));
#endif
            }
        } else if (uldt > VOLTAGE_TIME_TO_ERROR && g_wLastVoltage != 0xffff) {
            PRINT("Spanning: fout timeout\n");
            g_wLastVoltage = 0xffff;
        }
    }
    return (g_wLastVoltage != (word)-1) ? g_wLastVoltage * 10 : (word)-1;
}

//--------------------------------------------------------------------
// Servo update starten
//--------------------------------------------------------------------
void ServoDriver::BeginServoUpdate(void) {
    MakeSureServosAreOn();
}

//--------------------------------------------------------------------
// Servo info voor poot uitvoeren
//--------------------------------------------------------------------
void ServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
{
    word wCoxaSDV, wFemurSDV, wTibiaSDV;

    wCoxaSDV = (((long)sCoxaAngle1) * cPwmMult) / cPwmDiv + cPFConst;
    wFemurSDV = (((long)sFemurAngle1) * cPwmMult) / cPwmDiv + cPFConst;
    wTibiaSDV = (((long)sTibiaAngle1) * cPwmMult) / cPwmDiv + cPFConst;

    if (ServosEnabled) {
        if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
            g_awGoalAXPos[FIRSTCOXAPIN + LegIndex] = wCoxaSDV;
            g_awGoalAXPos[FIRSTFEMURPIN + LegIndex] = wFemurSDV;
            g_awGoalAXPos[FIRSTTIBIAPIN + LegIndex] = wTibiaSDV;
#endif
        } else {
            bioloid.setNextPoseByIndex(FIRSTCOXAPIN + LegIndex, wCoxaSDV);
            bioloid.setNextPoseByIndex(FIRSTFEMURPIN + LegIndex, wFemurSDV);
            bioloid.setNextPoseByIndex(FIRSTTIBIAPIN + LegIndex, wTibiaSDV);
        }
    }

#ifdef DEBUG_SERVOS
    if (g_fDebugOutput) {
        DBGSerial.print(LegIndex, DEC);
        DBGSerial.print("("); DBGSerial.print(sCoxaAngle1, DEC); DBGSerial.print("="); DBGSerial.print(wCoxaSDV, DEC);
        DBGSerial.print("),("); DBGSerial.print(sFemurAngle1, DEC); DBGSerial.print("="); DBGSerial.print(wFemurSDV, DEC);
        DBGSerial.print("),("); DBGSerial.print(sTibiaAngle1, DEC); DBGSerial.print("="); DBGSerial.print(wTibiaSDV, DEC);
        DBGSerial.print(") :");
    }
#endif
    g_WifiInputController.AllowControllerInterrupts(true);
}

//--------------------------------------------------------------------
// Servo info voor turret uitvoeren
//--------------------------------------------------------------------
void ServoDriver::OutputServoInfoForTurret(short sTiltAngle1, short sPanAngle1) {
    word wTiltSDV, wPanSDV;

    // Clamp to min/max (add this for safety, using defines from Hex_Cfg.h)
    if (sTiltAngle1 < cCMTiltMin1) sTiltAngle1 = cCMTiltMin1;
    if (sTiltAngle1 > cCMTiltMax1) sTiltAngle1 = cCMTiltMax1;
    if (sPanAngle1 < cCMPanMin1) sPanAngle1 = cCMPanMin1;
    if (sPanAngle1 > cCMPanMax1) sPanAngle1 = cCMPanMax1;

    wTiltSDV = (((long)sTiltAngle1*1.5) * cPwmMult) / cPwmDiv + cPFConst;    
    wPanSDV = (((long)sPanAngle1*2) * cPwmMult) / cPwmDiv + cPFConst;

    if (ServosEnabled) {
        if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
        g_awGoalAXPos[FIRST_TILT_SERVO] = wTiltSDV;
        g_awGoalAXPos[FIRST_PAN_SERVO] = wPanSDV;
#endif
        } else {
            bioloid.setNextPoseByIndex(FIRST_TILT_SERVO, wTiltSDV);
            bioloid.setNextPoseByIndex(FIRST_PAN_SERVO, wPanSDV);
        }
    }
    g_WifiInputController.AllowControllerInterrupts(true);
}


#ifdef DEBUG_SERVOS
    if (g_fDebugOutput) {
        DBGSerial.print("Turret(Pan="); DBGSerial.print(sPanAngle1, DEC); DBGSerial.print("="); DBGSerial.print(wPanSDV, DEC);
        DBGSerial.print("), (Tilt="); DBGSerial.print(sTiltAngle1, DEC); DBGSerial.print("="); DBGSerial.print(wTiltSDV, DEC);
        DBGSerial.print(") :");
    }
#endif

//--------------------------------------------------------------------
// AX-12 snelheid berekenen
//--------------------------------------------------------------------
#ifdef USE_AX12_SPEED_CONTROL
word CalculateAX12MoveSpeed(word wCurPos, word wGoalPos, word wTime) {
    word wTravel = (wGoalPos > wCurPos) ? wGoalPos - wCurPos : wCurPos - wGoalPos;
    uint32_t factor = (uint32_t)848 * wTravel;
    word wSpeed = (uint16_t)(factor / wTime);
    if (wSpeed > 1023) wSpeed = 1023;
    if (wSpeed < 26) wSpeed = 26;
    return wSpeed;
}
#endif

//--------------------------------------------------------------------
// Servo posities updaten
//--------------------------------------------------------------------
bool ServoDriver::CommitServoDriver(word wMoveTime) {
    g_WifiInputController.AllowControllerInterrupts(false);
    if (ServosEnabled) {
        bioloid.interpolateSetup(wMoveTime);
    }
#ifdef DEBUG_SERVOS
    if (g_fDebugOutput) DBGSerial.println(wMoveTime, DEC);
#endif
    g_WifiInputController.AllowControllerInterrupts(true);
    return true;
}

//--------------------------------------------------------------------
// Servo's vrijmaken
//--------------------------------------------------------------------
void ServoDriver::FreeServos(void) {
    if (ServosEnabled) {
        g_WifiInputController.AllowControllerInterrupts(false);
        SetRegOnAllServos(AX_TORQUE_ENABLE, 0);
        g_WifiInputController.AllowControllerInterrupts(true);
        g_fServosFree = true;
    }
}

//--------------------------------------------------------------------
// Idle tijd functie
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum = (uint8_t)-1;
static uint8_t g_iIdleLedState = 1;

void ServoDriver::IdleTime(void) {
    g_iIdleServoNum++;
    if (g_iIdleServoNum >= NUMSERVOS) {
        g_iIdleServoNum = 0;
        g_iIdleLedState = 1 - g_iIdleLedState;
    }
    dxl_write_byte(cPinTable[g_iIdleServoNum], AX_LED, g_iIdleLedState);
    dxl_get_result();
}
 
//--------------------------------------------------------------------
// Register instellen op alle servo's
//--------------------------------------------------------------------
void SetRegOnAllServos(uint8_t bReg, uint8_t bVal) {
    dxl_set_txpacket_id(AX_ID_BROADCAST);
    dxl_set_txpacket_instruction(AX_CMD_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, bReg);
    dxl_set_txpacket_parameter(1, 1);
    dxl_set_txpacket_length(2 * NUMSERVOS + 4);

    for (byte i = 0; i < NUMSERVOS; i++) {
        dxl_set_txpacket_parameter(2 + i * 2, cPinTable[i]);
        dxl_set_txpacket_parameter(3 + i * 2, bVal);
    }
    dxl_txrx_packet();
}

//--------------------------------------------------------------------
// Zorgen dat servo's aan zijn
//--------------------------------------------------------------------
void MakeSureServosAreOn(void) {
    if (ServosEnabled && g_fServosFree) {
        g_WifiInputController.AllowControllerInterrupts(false);
        bioloid.readPose();
        SetRegOnAllServos(AX_TORQUE_ENABLE, 1);
        g_WifiInputController.AllowControllerInterrupts(true);
        g_fServosFree = false;
    }
}

#ifdef OPT_BACKGROUND_PROCESS
//--------------------------------------------------------------------
// Achtergrondproces
//--------------------------------------------------------------------
void ServoDriver::BackgroundProcess(void) {
    bioloid.interpolateStep(false);
}
#endif