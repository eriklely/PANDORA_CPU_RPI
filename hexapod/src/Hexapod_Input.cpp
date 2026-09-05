//=======================================================================================
// Hexapod-besturing via WiFi (UDP)
// Bestand: Hexapod_Input.cpp
// Huidige datum: 27 februari 2025
//=======================================================================================


#include "Hex_Cfg.h"       // Configuratiebestand voor de hexapod
#include "Hexapod.h"       // Hoofddefinities en functies voor de hexapod

#include <ax12.h>          // Bibliotheek voor AX-12 servomotoren
#include <BioloidEX.h>     // Bibliotheek voor Bioloid-robotbesturing
#include <sys/socket.h>    // Sockets voor netwerkcommunicatie
#include <netinet/in.h>    // Internetadresstructuren
#include <arpa/inet.h>     // Hulpfuncties voor IP-adressen
#include <unistd.h>        // POSIX-besturingssysteemfuncties
#include <string.h>        // Stringmanipulatie
#include <stdio.h>         // Standaard I/O-functies
#include <signal.h>        // Signaalverwerking
#include <errno.h>         // Foutcodes
#include <fcntl.h>         // Bestandscontrole
#include <stdint.h>        // Standaard integer types
#include <algorithm>       // Algoritmen zoals min/max
#include <cmath>           // Wiskundige functies
#include <stdarg.h>        // Voor variabele argumentlijsten
#include <iostream>        // Standaard I/O-streams
#include <cstdlib>         // Algemene hulpfuncties
#include <unistd.h>        // POSIX-besturingssysteemfuncties (herhaald)
#include <gpiod.h>         // GPIO voor led aansturing
#include <mutex>  // For face_mutex

// Structuur voor het binaire commando-pakket (moet overeenkomen met C# CommandPacket, inclusief checksum)
struct CommandPacket {
    uint32_t magic;         // Validatiewaarde: 0xDEADBEEF
    uint8_t version;        // Protocolversie: 1
    uint8_t special_command;// 0 = normaal, 1 = STOP, 2 = HALT
    int16_t sustained_axes_lx; // Linker X-as (-32768 tot 32767)
    int16_t sustained_axes_ly; // Linker Y-as
    int16_t sustained_axes_rx; // Rechter X-as
    int16_t sustained_axes_ry; // Rechter Y-as
    int16_t sustained_axes_sx; // Secundaire X-as
    int16_t sustained_axes_sy; // Secundaire Y-as
    uint16_t buttons;       // Bitveld voor knopstatussen
    uint32_t duration_ms;   // Duur van aanhoudende beweging in milliseconden
    uint16_t checksum;      // 16-bit checksum (som van alle voorgaande bytes)
} __attribute__((packed));  // Zorgt voor compacte opslag zonder padding

// Functie om de checksum te berekenen (overeenkomend met de afzenderlogica)
static uint16_t CalculateChecksum(const CommandPacket& packet) {
    CommandPacket temp_packet = packet;
    temp_packet.checksum = 0; // Zet checksum op 0 voor berekening

    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&temp_packet);
    size_t length = sizeof(CommandPacket) - sizeof(uint16_t); // 24 bytes (exclusief checksum)

    uint16_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += bytes[i];
    }
    return sum;
}

// [CONSTANTEN]
// Enumeratie voor besturingsmodi
enum {
    WALKMODE=0,     // Loopmodus
    BODYMOVEMODE,   // Lichaamsbewegingsmodus
    MIXEDMODE,      // Gemengde modus
    CAMERAMODE,      // Gemengde modus
    NOMODE,         // Geen modus
    MODECNT         // Aantal modi
};

// Enumeratie voor hoogte- en snelheidsmodi
enum {
    NORM_NORM=0,    // Normaal normaal
    NORM_LONG,      // Normaal lang
    HIGH_NORM,      // Hoog normaal
    HIGH_LONG       // Hoog lang
};

// Lijst van gaitnamen opgeslagen in PROGMEM voor efficiënt geheugengebruik
extern "C" {
    const char s_sGN1[] PROGMEM = "Ripple 12";    // "Rimpel 12"
    const char s_sGN2[] PROGMEM = "Tripod 8";     // "Driepoot 8"
    const char s_sGN3[] PROGMEM = "Tripple 12";   // "Drievoudig 12" (mogelijk typo voor "Triple")
    const char s_sGN4[] PROGMEM = "Tripple 16";   // "Drievoudig 16"
    const char s_sGN5[] PROGMEM = "Wave 24";      // "Golf 24"
    const char s_sGN6[] PROGMEM = "Tripod 6";     // "Driepoot 6"
    PGM_P s_asGateNames[] PROGMEM = {
        s_sGN1, s_sGN2, s_sGN3, s_sGN4, s_sGN5, s_sGN6
    };
}

//================================================================================================
// Globale variabelen - Lokaal voor dit bestand
//================================================================================================
int udpSocket = -1;             // Socket voor het ontvangen van UDP-pakketten
int sendUdpSocket = -1;         // Socket voor het verzenden van UDP-responsen

struct sockaddr_in serverAddr = {0}, clientAddr = {0}; // Netwerkadressen, nul-geïnitialiseerd
socklen_t clientLen = sizeof(clientAddr); // Lengte van clientadres

unsigned long g_ulLastMsgTime = 0;          // last received packet timestamp
static bool g_fFirstPacketReceived = false; // prevents false timeout before any packet arrives
boolean g_fDynamicLegXZLength = false; // Vlag voor dynamische poot XZ-lengte
boolean variablelegliftheight = false; // Vlag voor variabele pootlifthoogte
boolean command_single_leg = false;    // Vlag voor enkel poot commando

//#define WifiInputController WifiInputController
WifiInputController g_WifiInputController; // Instantie van WifiInputController

static short g_BodyYOffset;     // Offset van de lichaams Y-positie
static short g_BodyYShift;      // Shift van de lichaams Y-positie
static byte ControlMode;        // Huidige besturingsmodus
static byte HeightSpeedMode;    // Modus voor hoogte en snelheid
static bool DoubleTravelOn;     // Vlag voor dubbele staplengte

// Aanhoudende aswaarden en duur
static int sustained_axes_lx = 0; // Linker X-as waarde
static int sustained_axes_ly = 0; // Linker Y-as waarde
static int sustained_axes_rx = 0; // Rechter X-as waarde
static int sustained_axes_ry = 0; // Rechter Y-as waarde
static int sustained_axes_sx = 0; // Secundaire X-as waarde
static int sustained_axes_sy = 0; // Secundaire Y-as waarde
static unsigned long sustained_start_time = 0; // Starttijd van aanhoudende beweging
static unsigned long sustained_duration = 0;   // Duur van aanhoudende beweging
static unsigned long last_udp_send_time = 0; // Time of the last UDP send

static unsigned long last_response_time = 0; // Tijd van laatste respons
struct sockaddr_in controllerAddr = {0};    // Adres van de controller

extern void ControllerTurnRobotOff(void);   // Schakelt de robot uit
extern void StartUpdateServos(void);        // Start servo-updates

double SmDiv = 5.0;  // startwaarde voor smoohtcontrol
double SmMargin = 20.0;

// Global libgpiod structures
struct gpiod_chip *gpio_chip = NULL;
struct gpiod_line *led_pin1 = NULL;
struct gpiod_line *led_pin2 = NULL;
struct gpiod_line *led_red_pin = NULL;

// LED state tracking
static int ledState = 0;              // Current state of the red LED (0 = off, 1 = on)
static unsigned long ledTurnOffTime = 0; // Time (in ms) when the LED should turn off
static const int LED_PIN1 = 24;  
static const int LED_PIN2 = 23;  
static const int LED_RED_PIN = 16; 


//================================================================================================
// Hulpfunctie om UDP-responsen te verzenden (gebruikt voor poort 8888)
//================================================================================================
static void sendUdpResponse(const char* message, struct sockaddr_in* addr, int socket) {
    if (socket == -1 || addr->sin_family != AF_INET) {
        PRINT("Cannot send response: invalid socket or address");
        return;
    }

    unsigned long now = millis(); // Get current time
    const unsigned long MIN_INTERVAL_MS = 50; // 100 ms interval for 10 Hz

    // Check if enough time has passed since the last send
    if (now - last_udp_send_time < MIN_INTERVAL_MS) {
        return; // Skip sending to enforce rate limit
    }

    int send_result = sendto(socket, message, strlen(message), 0, 
                            reinterpret_cast<struct sockaddr*>(addr), sizeof(*addr));
    if (send_result < 0) {
        PRINT("Failed to send response '%s': %s (errno: %d)", message, strerror(errno), errno);
    } else {
        last_udp_send_time = now; // Update last send time only on success
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr->sin_addr, ip_str, INET_ADDRSTRLEN);
    }
}

//================================================================================================
// Functie om periodiek looptijd en spanning te verzenden naar poort 8888
//================================================================================================
void sendPeriodicResponse() {
    unsigned long now = millis();
    const unsigned long MIN_INTERVAL_MS = 50; // 100 ms interval for 10 Hz
    if ((now - last_response_time >= g_loop_time) && (now - last_response_time >= MIN_INTERVAL_MS)) {
        char response[64];
        snprintf(response, sizeof(response), "%lu,%u", 
                g_loop_time, 
                g_ServoDriver.GetBatteryVoltage());

        // Verzend naar vast controlleradres
        sendUdpResponse(response, &controllerAddr, udpSocket);
        last_response_time = now;
    }
}

//================================================================================================
// Functie om een bericht te verzenden via UDP naar poort 8889
//================================================================================================
void sendUdpOnly(const char* format, ...) {
    if (clientAddr.sin_port == 0 || sendUdpSocket < 0)
        return;

    // Check format string before expensive vsnprintf; format strings contain keywords directly
    bool contains_command = (strstr(format, "Command") != nullptr || strstr(format, "command") != nullptr);
    bool is_error = (strstr(format, "error") != nullptr || strstr(format, "failed") != nullptr ||
                     strstr(format, "invalid") != nullptr || strstr(format, "cannot") != nullptr);

    if (!g_fDebugEnabled && !contains_command)
        return;

    char response[512];
    va_list args;
    va_start(args, format);
    vsnprintf(response, sizeof(response), format, args);
    va_end(args);

    // Voeg "ERROR: " of "INFO: " prefix toe
    char prefixed_response[512];
    size_t prefix_len = is_error ? snprintf(prefixed_response, sizeof(prefixed_response), "ERROR: ")
                                : snprintf(prefixed_response, sizeof(prefixed_response), "INFO: ");

    // Voeg het originele bericht toe
    if (prefix_len < sizeof(prefixed_response)) {
        size_t remaining_space = sizeof(prefixed_response) - prefix_len - 1;
        snprintf(prefixed_response + prefix_len, sizeof(prefixed_response) - prefix_len, "%.*s", (int)remaining_space, response);
    }

    // Verzend via UDP
    struct sockaddr_in reply_addr = clientAddr;
    reply_addr.sin_port = htons(SEND_UDP_PORT);
    sendUdpResponse(prefixed_response, &reply_addr, sendUdpSocket);
    PRINT(prefixed_response, &reply_addr, sendUdpSocket);
}

//=================================================================================================
// Initialisatiefunctie voor UDP-controller
//=================================================================================================
void WifiInputController::Init(void) {
    g_BodyYOffset = 0;      // Reset lichaam Y-offset
    g_BodyYShift = 0;       // Reset lichaam Y-shift
    ControlMode = WALKMODE; // Stel standaard modus in op loopmodus
    HeightSpeedMode = NORM_NORM; // Stel standaard hoogte/snelheid in
    DoubleTravelOn = false; // Schakel dubbele staplengte uit

    // Stel de compliance slope in voor alle 18 servos (1-18)
    for (int i = 1; i < 19; i++) {
        ax12SetRegister(i, AX_CW_COMPLIANCE_SLOPE, COMPLIANCE_SLOPE);
        ax12SetRegister(i, AX_CCW_COMPLIANCE_SLOPE, COMPLIANCE_SLOPE);
    }
    PRINT("Compliance slope set to 0x%02X", COMPLIANCE_SLOPE);
    PRINTUDP("Compliance slope set to 0x%02X command", COMPLIANCE_SLOPE);

    // Maak en configureer udpSocket (poort 8888)
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        PRINT("Failed to create UDP socket: %s", strerror(errno));
        PRINTUDP("Failed to create UDP socket: %s", strerror(errno));
        return;
    }
    PRINT("Socket created: %d", udpSocket);
    PRINTUDP("Socket created: %d", udpSocket);

    // Stel socket in als non-blocking
    int flags = fcntl(udpSocket, F_GETFL, 0);
    if (fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
        PRINT("Failed to set non-blocking: %s", strerror(errno));
        PRINTUDP("Failed to set non-blocking: %s", strerror(errno));
        close(udpSocket);
        udpSocket = -1;
        return;
    }

    // Open GPIO chip
    gpio_chip = gpiod_chip_open_by_name("gpiochip0");
    if (!gpio_chip) {
        PRINT("Failed to open GPIO chip: %s", strerror(errno));
        PRINTUDP("Failed to open GPIO chip: %s", strerror(errno));
    } else {
        // Request LED_PIN1 (GPIO 24)
        led_pin1 = gpiod_chip_get_line(gpio_chip, LED_PIN1);
        if (!led_pin1 || gpiod_line_request_output(led_pin1, "hexapod_led1", 1) < 0) {
            PRINT("Failed to initialize LED pin %d: %s", LED_PIN1, strerror(errno));
            PRINTUDP("Failed to initialize LED pin %d: %s", LED_PIN1, strerror(errno));
        }

        // Request LED_PIN2 (GPIO 23)
        led_pin2 = gpiod_chip_get_line(gpio_chip, LED_PIN2);
        if (!led_pin2 || gpiod_line_request_output(led_pin2, "hexapod_led2", 0) < 0) {
            PRINT("Failed to initialize LED pin %d: %s", LED_PIN2, strerror(errno));
            PRINTUDP("Failed to initialize LED pin %d: %s", LED_PIN2, strerror(errno));
        }

        // Request LED_RED_PIN
        led_red_pin = gpiod_chip_get_line(gpio_chip, LED_RED_PIN);
        if (!led_red_pin || gpiod_line_request_output(led_red_pin, "hexapod_led_red", 0) < 0) {
            PRINT("Failed to initialize red LED pin %d: %s", LED_RED_PIN, strerror(errno));
            PRINTUDP("Failed to initialize red LED pin %d: %s", LED_RED_PIN, strerror(errno));
        }

        if (led_pin1 && led_pin2 && led_red_pin) {
            PRINT("GPIO initialized: LED pins %d, %d, and red LED %d", LED_PIN1, LED_PIN2, LED_RED_PIN);
            PRINTUDP("GPIO initialized: LED pins %d, %d, and red LED %d", LED_PIN1, LED_PIN2, LED_RED_PIN);
        } else {
            // Cleanup if any pin failed
            Cleanup();
        }
    }

    // Schakel hergebruik van adres in
    int reuse = 1;
    if (setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
        PRINT("setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
        PRINTUDP("setsockopt(SO_REUSEADDR) failed: %s", strerror(errno));
        close(udpSocket);
        udpSocket = -1;
        return;
    }

    // Configureer serveradres
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(UDP_PORT);

    // Bind de socket aan het adres
    if (bind(udpSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        PRINT("UDP bind failed: %s", strerror(errno));
        PRINTUDP("UDP bind failed: %s", strerror(errno));
        close(udpSocket);
        udpSocket = -1;
        return;
    }
    PRINT("Socket bound successfully to port %d", UDP_PORT);
    PRINTUDP("Socket bound successfully to port %d", UDP_PORT);

    // Maak en configureer sendUdpSocket (poort 8889)
    sendUdpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendUdpSocket < 0) {
        PRINT("Failed to create send UDP socket: %s", strerror(errno));
        PRINTUDP("Failed to create send UDP socket: %s", strerror(errno));
        close(udpSocket);
        udpSocket = -1;
        return;
    }
    PRINT("Send socket created: %d", sendUdpSocket);
    PRINTUDP("Send socket created: %d", sendUdpSocket);

    // Stel verzendsocket in als non-blocking
    flags = fcntl(sendUdpSocket, F_GETFL, 0);
    if (fcntl(sendUdpSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
        PRINT("Failed to set sendUdpSocket non-blocking: %s", strerror(errno));
        PRINTUDP("Failed to set sendUdpSocket non-blocking: %s", strerror(errno));
        close(sendUdpSocket);
        sendUdpSocket = -1;
        close(udpSocket);
        udpSocket = -1;
        return;
    }

    // Schakel hergebruik van adres in voor verzendsocket
    if (setsockopt(sendUdpSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
        PRINT("setsockopt(SO_REUSEADDR) failed for sendUdpSocket: %s", strerror(errno));
        PRINTUDP("setsockopt(SO_REUSEADDR) failed for sendUdpSocket: %s", strerror(errno));
        close(sendUdpSocket);
        sendUdpSocket = -1;
        close(udpSocket);
        udpSocket = -1;
        return;
    }

    // Configureer verzendadres
    struct sockaddr_in sendAddr = {0};
    sendAddr.sin_family = AF_INET;
    sendAddr.sin_addr.s_addr = INADDR_ANY;
    sendAddr.sin_port = htons(SEND_UDP_PORT);
    if (bind(sendUdpSocket, (struct sockaddr*)&sendAddr, sizeof(sendAddr)) < 0) {
        PRINT("Failed to bind sendUdpSocket to port %d: %s", SEND_UDP_PORT, strerror(errno));
        PRINTUDP("Failed to bind sendUdpSocket to port %d: %s", SEND_UDP_PORT, strerror(errno));
        close(sendUdpSocket);
        sendUdpSocket = -1;
        close(udpSocket);
        udpSocket = -1;
        return;
    }
    PRINT("sendUdpSocket bound successfully to port %d", SEND_UDP_PORT);
    PRINTUDP("sendUdpSocket bound successfully to port %d", SEND_UDP_PORT);

    // Initialiseer controlleradres voor periodieke verzending
    controllerAddr.sin_family = AF_INET;
    controllerAddr.sin_port = htons(UDP_PORT); // Verzend naar 8888
    if (inet_pton(AF_INET, CONTROLLER_IP, &controllerAddr.sin_addr) <= 0) {
        PRINT("Failed to set controller IP %s: %s", CONTROLLER_IP, strerror(errno));
        PRINTUDP("Failed to set controller IP %s: %s", CONTROLLER_IP, strerror(errno));
    } else {
        PRINT("Controller address set to %s:%d", CONTROLLER_IP, UDP_PORT);
        PRINTUDP("Controller address set to %s:%d", CONTROLLER_IP, UDP_PORT);
    }

    // Controleer op initialisatiefouten
    if (udpSocket < 0 || sendUdpSocket < 0) {
        PRINT("Critical error: UDP socket initialization failed. Exiting.");
        PRINTUDP("Critical error: UDP socket initialization failed. Exiting.");
        exit(1);
    }
    PRINT("UDP server listening on port %d, sending responses on port %d", UDP_PORT, SEND_UDP_PORT);
    PRINTUDP("UDP server listening on port %d, sending responses on port %d", UDP_PORT, SEND_UDP_PORT);
    PRINT("Hexapod initialized with WiFi control");
    PRINTUDP("Hexapod initialized with WiFi control command");
}

//=================================================================================================
// Functie om controller-interrupts toe te staan (momenteel geen actie)
//=================================================================================================
void WifiInputController::AllowControllerInterrupts(boolean fAllow) {
    // Geen actie nodig
}

//=================================================================================================
// Hoofdinputfunctie voor WiFi-besturing
//=================================================================================================
void WifiInputController::ControlInput(void) {
    if (udpSocket < 0) {
        PRINT("UDP socket is invalid, cannot receive data");
        PRINTUDP("UDP socket is invalid, cannot receive data");
        return;
    }
    if (sendUdpSocket < 0) {
        PRINT("Send UDP socket is invalid, cannot send responses");
        PRINTUDP("Send UDP socket is invalid, cannot send responses");
        return;
    }

    // Verzend periodieke looptijd/spanning respons
    sendPeriodicResponse();

    struct CommandPacket packet = {0};
    memset(&clientAddr, 0, sizeof(clientAddr));
    clientLen = sizeof(clientAddr);
    PRINT("Calling recvfrom with socket %d", udpSocket);
    PRINTUDP("Calling recvfrom with socket %d", udpSocket);
    int bytesReceived = recvfrom(udpSocket, (char*)&packet, sizeof(packet), 0,
                                 (struct sockaddr*)&clientAddr, &clientLen);
    PRINT("recvfrom returned %d bytes", bytesReceived);
    PRINTUDP("recvfrom returned %d bytes", bytesReceived);

    unsigned long currentTime = millis();
    // Definieer knopvariabelen (must be at function scope — used after packet-received block)
    bool button_start = false, button_mode_select = false, button_gait_select = false, button_camera_mode = false,
        button_Debug = false, button_r2 = false, button_balance_mode = false, button_updown_select = false,
        button_Relax = false, button_values_reset = false, button_attack_mode = false, button_travel_select = false,
        button_single_leg = false, button_up = false, button_down = false, button_left = false, button_right = false;
    boolean fAdjustLegPositions = false;
    short sLegInitXZAdjust = 0;
    short sLegInitAngleAdjust = 0;

//=================================================================================================
// Inputfunctie voor UDP
//=================================================================================================
    if (bytesReceived > 0) {
        // Controleer pakketgrootte, magische waarde en versie
        if (bytesReceived == sizeof(packet) && packet.magic == 0xDEADBEEF && packet.version == 1) {
            if (static_cast<size_t>(bytesReceived) > sizeof(packet)) {
                PRINT("Oversized packet received: %d bytes", bytesReceived);
                PRINTUDP("Oversized packet received: %d bytes", bytesReceived);
                return;
            }
            uint16_t calculated_checksum = CalculateChecksum(packet);
            if (calculated_checksum == packet.checksum) {
                PRINT("Processing valid packet=%d", packet.special_command);
                PRINTUDP("Processing valid packet=%d", packet.special_command);
                g_ulLastMsgTime = currentTime;
                g_fFirstPacketReceived = true;
                PRINT("Received valid packet: %d", packet.special_command);
                PRINTUDP("Received valid packet: %d", packet.special_command);

                switch (packet.special_command) {
                    case 1: { // STOP
                        PRINT("STOP command");
                        PRINTUDP("STOP command");
                        ControllerTurnRobotOff();
                        PRINT("After ControllerTurnRobotOff");
                        PRINTUDP("After ControllerTurnRobotOff command");
                        if (led_pin2 && gpiod_line_set_value(led_pin2, 0) < 0) {
                            PRINT("Failed to set LED_PIN2: %s", strerror(errno));
                        }
                        showFeedbackLED(1, 100);
                        return;
                    }
                    case 2: { // HALT
                        PRINT("HALT command");
                        PRINTUDP("HALT command");
                        // Reset aanhoudende bewegingen
                        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 
                        sustained_axes_ry = sustained_axes_sx = sustained_axes_sy = 0;
                        PRINT("HALT command received: Sustained movement stopped");
                        PRINTUDP("HALT command received: Sustained movement stopped");
                        showFeedbackLED(1, 100); 
                        break;
                    }
                    case 3: { // POWER
                        PRINT("POWER command");
                        PRINTUDP("POWER command");
                        showFeedbackLED(1, 100);
                        ControllerTurnRobotOff();
                        if (led_pin1 && gpiod_line_set_value(led_pin1, 0) < 0) {
                            PRINT("Failed to set LED_PIN1: %s", strerror(errno));
                        }
                        if (led_pin2 && gpiod_line_set_value(led_pin2, 0) < 0) {
                            PRINT("Failed to set LED_PIN2: %s", strerror(errno));
                        }
                        PRINT("LED turned off with robot shutdown");
                        PRINTUDP("LED turned off with robot shutdown");
                        sleep(2);
                        Cleanup();
                        sleep(2);
                        PRINT("POWER command received: shutting down");
                        PRINTUDP("POWER command received: shutting down");
                        system("sync");
                        sleep(5);
                        system("sudo shutdown now");
                        sleep(1000);
                        break;
                    }
                    case 4: { // RESET
                        PRINT("RESET command");
                        PRINTUDP("RESET command");
                        ResetHexapod();
                        PRINT("RESET command received: Position reset");
                        PRINTUDP("RESET command received: Position reset command");
                        showFeedbackLED(1, 100); 
                        break;
                    }
                    case 5: { // PAUSE
                        PRINT("PAUSE command");
                        PRINTUDP("PAUSE command");    
                        sustained_duration = packet.duration_ms;                   
                        PRINT("PAUSE command received: just pausing");
                        PRINTUDP("PAUSE command received: just pausing");
                        showFeedbackLED(1, 100); 
                        break;
                    }
                    case 6: { // CHECK
                        PRINT("CHECK command");
                        PRINTUDP("CHECK command");                       
                        PRINT("CHECK command received: just checking");
                        PRINTUDP("CHECK command received: just checking");
                        showFeedbackLED(1, 20); 
                        break;
                    }                     
                    case 11: { // Modus selectie 1
                        ControlMode = 0;
                        PRINT("Command Mode Select 1");
                        PRINTUDP("Command Mode Select 1");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 12: { // Modus selectie 2
                        ControlMode = 1;
                        PRINT("Command Mode Select 2");
                        PRINTUDP("Command Mode Select 2");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 13: { // Modus selectie 3
                        ControlMode = 2;
                        PRINT("Command Mode Select 3");
                        PRINTUDP("Command Mode Select 3");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 14: { // Modus selectie 4
                        ControlMode = 3;
                        PRINT("Command Mode Select 4");
                        PRINTUDP("Command Mode Select 4");              
                        showFeedbackLED(1, 100);   
                        break;
                    }  
                    case 15: { // Modus selectie 5 Camera
                        ControlMode = 4;
                        GaitSelect();
                        PRINT("Command Mode Select 5");
                        PRINTUDP("Command Mode Select 5");              
                        showFeedbackLED(1, 100);   
                        break;
                    }                                    
                    case 21: { // gait selectie 1
                        g_InControlState.GaitType = 0;
                        PRINT("Command Gait Select 1");
                        PRINTUDP("Command Gait Select 1");        
                        GaitSelect();       
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 22: { // gait selectie 2
                        g_InControlState.GaitType = 1;
                        PRINT("Command Gait Select 2");
                        PRINTUDP("Command Gait Select 2");
                        GaitSelect();              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 23: { // gait selectie 3
                        g_InControlState.GaitType = 2;
                        PRINT("Command Gait Select 3");
                        PRINTUDP("Command Gait Select 3");              
                        GaitSelect();
                        showFeedbackLED(1, 100);                       
                        break;
                    }
                    case 24: { // gait selectie 4
                        g_InControlState.GaitType = 3;
                        PRINT("Command Gait Select 4");
                        PRINTUDP("Command Gait Select 4");              
                        GaitSelect();
                        showFeedbackLED(1, 100);                       
                        break;
                    }
                    case 25: { // gait selectie 5
                        g_InControlState.GaitType = 4;
                        PRINT("Command Gait Select 5");
                        PRINTUDP("Command Gait Select 5");              
                        GaitSelect();
                        showFeedbackLED(1, 100);           
                        break;
                    }
                    case 26: { // gait selectie 6
                        g_InControlState.GaitType = 5;
                        PRINT("Command Gait Select 6");
                        PRINTUDP("Command Gait Select 6");              
                        GaitSelect();
                        showFeedbackLED(1, 100);   
                        break;
                    }    
                    case 31: { // staplengte selectie 1
                        DoubleTravelOn = false;
                        variablelegliftheight = false;
                        PRINT("Command Travel Select 1");
                        PRINTUDP("Command Travel Select 1");              
                        showFeedbackLED(1, 100);   
                        break;
                    } 
                    case 32: { // staplengte selectie 2
                        DoubleTravelOn = true;
                        variablelegliftheight = false;                    
                        PRINT("Command Travel Select 2");
                        PRINTUDP("Command Travel Select 2");              
                        showFeedbackLED(1, 100);   
                        break;
                    } 
                    case 33: { // staplengte selectie 3
                               DoubleTravelOn = false;
                        variablelegliftheight = true;                    
                        PRINT("Command Travel Select 3");
                        PRINTUDP("Command Travel Select 3");              
                        showFeedbackLED(1, 100);   
                        break;
                    }                                                                                                                                                                                            
                    case 34: { // staplengte selectie 4
                        g_InControlState.GaitType = 4;
                        DoubleTravelOn = true;
                        variablelegliftheight = true;                    
                        PRINT("Command Travel Select 4");
                        PRINTUDP("Command Travel Select 4");              
                        showFeedbackLED(1, 100);   
                        break;
                    } 
                    case 41: { // Enkel poot 1
                        g_InControlState.SelectedLeg = 0;
                        command_single_leg = true;
                        PRINT("Command Single Leg 1");
                        PRINTUDP("Command Single Leg 1");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 42: { // Enkel poot 2
                        g_InControlState.SelectedLeg = 1;
                        command_single_leg = true;
                        PRINT("Command Single Leg 2");
                        PRINTUDP("Command Single Leg 2");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 43: { // Enkel poot 3
                        g_InControlState.SelectedLeg = 2;
                        command_single_leg = true;
                        PRINT("Command Single Leg 3");
                        PRINTUDP("Command Single Leg 3");              
                        showFeedbackLED(1, 100);                       
                        break;
                    }
                    case 44: { // Enkel poot 4
                        g_InControlState.SelectedLeg = 3;
                        command_single_leg = true;
                        PRINT("Command Single Leg 4");
                        PRINTUDP("Command Single Leg 4");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 45: { // Enkel poot 5
                        g_InControlState.SelectedLeg = 4;
                        command_single_leg = true;
                        PRINT("Command Single Leg 5");
                        PRINTUDP("Command Single Leg 5");              
                        showFeedbackLED(1, 100);   
                        break;
                    }
                    case 46: { // Enkel poot 6
                        g_InControlState.SelectedLeg = 5;
                        command_single_leg = true;
                        PRINT("Command Single Leg 6");
                        PRINTUDP("Command Single Leg 6");           
                        showFeedbackLED(1, 100);   
                        break;
                    }                 
                    case 0: { // Normaal commando
                        // PRINT("Move command");
                        // PRINTUDP("Move command");
                        for (int i = 0; i < bytesReceived; i++) {
                            PRINT("Byte %d: 0x%02X", i, ((uint8_t*)&packet)[i]);
                        }
                        // Clamp axes and negate; guard INT16_MIN (-32768): negating it overflows int16_t
                        auto clamp_negate = [](int16_t v) -> int {
                            int16_t c = std::min(std::max(v, (int16_t)-32767), (int16_t)32767);
                            return -(int)c;
                        };
                        auto clamp_only = [](int16_t v) -> int {
                            return (int)std::min(std::max(v, (int16_t)-32767), (int16_t)32767);
                        };
                        sustained_axes_lx = clamp_negate(packet.sustained_axes_lx);
                        sustained_axes_ly = clamp_negate(packet.sustained_axes_ly);
                        sustained_axes_rx = clamp_only(packet.sustained_axes_rx);
                        sustained_axes_ry = clamp_only(packet.sustained_axes_ry);
                        sustained_axes_sx = clamp_negate(packet.sustained_axes_sx);
                        sustained_axes_sy = clamp_negate(packet.sustained_axes_sy);

                        sustained_start_time = currentTime;
                        sustained_duration = packet.duration_ms;
                        
                        // Stel knopstatussen in op basis van bitveld
                        button_start = packet.buttons & (1 << 0);
                        button_mode_select = packet.buttons & (1 << 1);
                        button_gait_select = packet.buttons & (1 << 2);
                        button_travel_select = packet.buttons & (1 << 3);
                        button_updown_select = packet.buttons & (1 << 4);
                        button_balance_mode = packet.buttons & (1 << 5);
                        button_attack_mode = packet.buttons & (1 << 6);
                        button_single_leg = packet.buttons & (1 << 7);
                        button_up = packet.buttons & (1 << 8);
                        button_down = packet.buttons & (1 << 9);
                        button_left = packet.buttons & (1 << 10);
                        button_right = packet.buttons & (1 << 11);
                        button_camera_mode = packet.buttons & (1 << 12);
                        button_Debug = packet.buttons & (1 << 13);
                        button_r2 = packet.buttons & (1 << 14);
                        button_Relax = packet.buttons & (1 << 15);

                        // PRINT("Move command: sustained_axes_lx=%d, buttons=0x%X, duration=%lu",
                        //      sustained_axes_lx, packet.buttons, sustained_duration);
                        // PRINTUDP("Move command: sustained_axes_lx=%d, buttons=0x%X, duration=%lu",
                        //         sustained_axes_lx, packet.buttons, sustained_duration);
                        break;
                    }
                }
            } else {
                PRINT("Received packet with invalid checksum: received=%u, calculated=%u", packet.checksum, calculated_checksum);
                PRINTUDP("Received packet with invalid checksum: received=%u, calculated=%u", packet.checksum, calculated_checksum);
            }
        } else {
            PRINT("Received incomplete or invalid packet: size=%d", bytesReceived);
            PRINTUDP("Received incomplete or invalid packet: size=%d", bytesReceived);
        }
    } else if (bytesReceived == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            PRINT("No data available right now (non-blocking socket)");
            PRINTUDP("No data available right now (non-blocking socket)");
        } else {
            PRINT("recvfrom error: %s", strerror(errno));
            PRINTUDP("recvfrom error: %s", strerror(errno));
        }
    } else {
        PRINT("Unexpected return value from recvfrom: %d", bytesReceived);
        PRINTUDP("Unexpected return value from recvfrom: %d", bytesReceived);
    }
    // Controleer op timeout en stop aanhoudende bewegingen
    if (g_fFirstPacketReceived && (currentTime - g_ulLastMsgTime > TIMEOUT_MS)) {
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 
        sustained_axes_ry = sustained_axes_sx = sustained_axes_sy = 0;
        sustained_duration = 0;
        PRINT("No commands received for %d ms, stopping sustained movement", TIMEOUT_MS);
        PRINTUDP("No commands received for %d ms, stopping sustained movement", TIMEOUT_MS);
    }

    // Recognition Mode (button_r2 / bit 14): toggle face detection on rising edge
    {
        static bool prev_button_r2 = false;
        if (button_r2 && !prev_button_r2) {
            g_fFaceDetectionEnabled = !g_fFaceDetectionEnabled;
            PRINT("Face detection %s", g_fFaceDetectionEnabled ? "enabled" : "disabled");
            PRINTUDP("Face detection %s", g_fFaceDetectionEnabled ? "enabled" : "disabled");
        }
        prev_button_r2 = button_r2;
    }

    // Bepaal of de controller in gebruik is
    g_InControlState.fControllerInUse = button_start || button_mode_select || button_gait_select ||  
        button_camera_mode || button_Debug || button_r2 || button_balance_mode || button_updown_select || button_Relax ||  
        button_values_reset || button_attack_mode || button_travel_select || button_single_leg || 
        button_up || button_down || button_left || button_right ||
        (abs((sustained_axes_lx/256)) > 0) || (abs((sustained_axes_ly/256)) > 0) ||
        (abs((sustained_axes_rx/256)) > 0) || (abs((sustained_axes_ry/256)) > 0);

//=================================================================================================
// Knop Start
//=================================================================================================        
    if (button_start) {
        if (!g_InControlState.fRobotOn) {
            g_InControlState.fRobotOn = true;
            ResetHexapod();
            g_BodyYOffset = 0;
            ControlMode = NOMODE;
            if (led_pin2 && gpiod_line_set_value(led_pin2, 1) < 0) {
                PRINT("Failed to set LED_PIN2: %s", strerror(errno));
            }
            PRINT("LED turned on with hexapod start");
            PRINTUDP("LED turned on with hexapod start");
        }
        showFeedbackLED(1, 100);
    }

    // Stop hier als de hexapod niet aan staat
    if (!g_InControlState.fRobotOn) {
        return;
    }

//=================================================================================================
// Knop Modus Selecteren
//=================================================================================================       
    if (button_mode_select) {
        if (++ControlMode >= MODECNT) {
            ControlMode = WALKMODE;
            PRINT("Walk mode");
            PRINTUDP("Walk mode command");
        } else {
            if (ControlMode == 1) {
                PRINT("Body movement mode");
                PRINTUDP("Body movement mode command");
            }
            if (ControlMode == 2) {
                PRINT("Mixed mode");
                PRINTUDP("Mixed mode command");
            }
            if (ControlMode == 3) {
                PRINT("No mode");
                PRINTUDP("No mode command");
            }  
            if (ControlMode == 4) {
                ResetHexapod();
                PRINT("No mode");
                PRINTUDP("No mode command");
            }            
        }
        showFeedbackLED(1, 100); 
    }

//=================================================================================================
// Knop gait Selecteren
//=================================================================================================       
    if (button_gait_select &&  
        abs(g_InControlState.TravelLength.x) == 0 &&
        abs(g_InControlState.TravelLength.z) == 0 &&
        abs(g_InControlState.TravelLength.y*2) == 0) {
        g_InControlState.GaitType = (g_InControlState.GaitType + 1) % NUM_GAITS;
        PRINT("%s", s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0]);
        PRINTUDP("%s command", s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0]);
        GaitSelect();
        showFeedbackLED(1, 100); 
    }

//=================================================================================================
// Knop staplengte Selecteren
//=================================================================================================   
    if (button_travel_select) {
        HeightSpeedMode = (HeightSpeedMode + 1) & 0x3;
        DoubleTravelOn = HeightSpeedMode & 0x1;
        variablelegliftheight = HeightSpeedMode & 0x2;
        g_InControlState.LegLiftHeight = variablelegliftheight ? max(g_BodyYOffset + g_BodyYShift, 0) : 50;
        PRINT("%s", DoubleTravelOn ? "Double travel on" : "Double travel off");
        PRINT("%s", variablelegliftheight ? "Leg lift height high command" : "Leg lift height low command");
        PRINTUDP("%s%s", 
            DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
            variablelegliftheight ? "Leg lift height high command" : "Leg lift height low command");
        showFeedbackLED(1, 100); 
    }

//=================================================================================================
// Knop Omhoog/Omlaag Selecteren
//=================================================================================================   
    if (button_updown_select) {
        if (g_BodyYOffset <= 0) {
            g_BodyYOffset = StandingHeight;
            g_fHeightTransition = true; // Vlag voor neer-naar-op transitie
            fAdjustLegPositions = true;
            g_fDynamicLegXZLength = false;
            sLegInitXZAdjust = 1;
            g_BodyYShift = 0;
            g_InControlState.RobotUp = true;
            PRINT("Hexapod standing up");
            PRINTUDP("Hexapod standing up command");
        } else {
            g_BodyYOffset = 0;
            g_fHeightTransition = true; // Vlag voor neer-naar-op transitie
            fAdjustLegPositions = false;
            g_fDynamicLegXZLength = false;
            g_InControlState.RobotUp = false;
            PRINT("Hexapod sitting down");
            PRINTUDP("Hexapod sitting down command");
        }
        showFeedbackLED(1, 100); 
    } 

//=================================================================================================
// Knop Links
//=================================================================================================       
    if (button_left) {
        fAdjustLegPositions = true;
        sLegInitXZAdjust = -4;
#ifdef ADJUSTABLE_LEG_ANGLES
        // Verminder sLegInitAngleAdjust (nabootsing negatieve sustained_axes_ly)
        sLegInitAngleAdjust -= 0.1f;
#endif
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 0;
        showFeedbackLED(1, 100); 
    }  

//=================================================================================================
// Knop Rechts
//=================================================================================================         
    if (button_right) {
        fAdjustLegPositions = true;
        sLegInitXZAdjust = 4;
#ifdef ADJUSTABLE_LEG_ANGLES
        // Verhoog sLegInitAngleAdjust (nabootsing positieve sustained_axes_ly)
        sLegInitAngleAdjust += 0.1f;
#endif
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 0;
        showFeedbackLED(1, 100); 
    }  

//=================================================================================================
// Knop Omhoog
//=================================================================================================       
    if (button_up) {
        fAdjustLegPositions = true;
        sLegInitXZAdjust = 4;
        g_BodyYOffset += 4;
#ifdef ADJUSTABLE_LEG_ANGLES
        // Verminder sLegInitAngleAdjust (nabootsing negatieve sustained_axes_ly)
        sLegInitAngleAdjust -= 0.1f;
#endif
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 0;
        showFeedbackLED(1, 100); 
    }  

//=================================================================================================
// Knop Omlaag
//=================================================================================================         
    if (button_down) {
        fAdjustLegPositions = true;
        sLegInitXZAdjust = -4;
        g_BodyYOffset -= 4;
#ifdef ADJUSTABLE_LEG_ANGLES        
        sLegInitAngleAdjust += 0.1f; // Verhoog sLegInitAngleAdjust (nabootsing positieve sustained_axes_ly)
#endif
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 0;
        showFeedbackLED(1, 100); 
    }     

//=================================================================================================
// Knop Debug
//=================================================================================================       
    if (button_Debug) {
        g_fPrintEnabled = !g_fPrintEnabled;
        g_fDebugEnabled = !g_fDebugEnabled; // Debug vlag
        g_fDebugOutput = !g_fDebugOutput; 
        PRINT("Debugging %s", g_fPrintEnabled ? "enabled" : "disabled");
        PRINTUDP("Debugging %s command", g_fPrintEnabled ? "enabled" : "disabled");   
        showFeedbackLED(1, 100);   
    }    

//=================================================================================================
// Knop Camera modus
//=================================================================================================       
if (button_camera_mode) {
    ControlMode = CAMERAMODE;
    PRINT("Camera on/off");
    PRINTUDP("Camera on/off"); 
    showFeedbackLED(1, 100);   
}    

//=================================================================================================
// Knop Ontspannen
//=================================================================================================       
    if (button_Relax) {
        PRINT("Leg adjustment");
        PRINTUDP("Leg adjustment command");
        fAdjustLegPositions = false;
        g_fDynamicLegXZLength = false;
        sLegInitXZAdjust = 1;
        showFeedbackLED(1, 100); 
    }

    // Stop hier als de hexapod niet aan staat en status niet veranderd is
    if ((!g_InControlState.RobotUp) && (g_InControlState.RobotUp == g_InControlState.fPrev_RobotUp)) {
        return;
    }

//=================================================================================================
// Knop Aanvalsmodus
//=================================================================================================     
    if (button_attack_mode) {
        g_InControlState.BodyPos = {0, 0, 40};
        g_InControlState.BodyRot1 = {-60, 0, 0};
        g_InControlState.TravelLength = {0, 0, 0};
        g_BodyYShift = 0;
        g_BodyYOffset = StandingHeight;
        fAdjustLegPositions = true;
        g_InControlState.BalanceMode = true;
        g_InControlState.GaitType = 4;
        DoubleTravelOn = true;
        GaitSelect();
        sustained_axes_lx = sustained_axes_ly = sustained_axes_rx = 
        sustained_axes_ry = sustained_axes_sx = sustained_axes_sy = 0;
        sustained_duration = 0;
        PRINT("Attack stance");
        PRINTUDP("Attack stance command");
        showFeedbackLED(1, 100); 
    }

//=================================================================================================
// Knop Enkel poot Modus
//=================================================================================================     
    if (button_single_leg || command_single_leg) {
        SmDiv = 50;   //Smoothingwaarde divider
        SmMargin =3;
                
        if (g_InControlState.SelectedLeg == 255) {
            g_InControlState.SelectedLeg = 2;
            PRINT("Single-leg control enabled: Leg 0 selected");
            PRINTUDP("Single-leg control enabled: Leg 0 selected command");
        } else if ((g_InControlState.SelectedLeg < (CNT_LEGS - 1)) && (!command_single_leg)) {
            g_InControlState.SelectedLeg += 3;
            PRINT("Single-leg control: Leg %d selected", g_InControlState.SelectedLeg);
            PRINTUDP("Single-leg control: Leg %d selected command", g_InControlState.SelectedLeg);
        } else if (!command_single_leg) {
            g_InControlState.SelectedLeg = 255;
            PRINT("Single-leg control disabled");
            PRINTUDP("Single-leg control disabled command");
        }

        PRINT("Body shift applied for leg %d: x=%ld, z=%ld, rot_x=%ld, rot_z=%ld\n",
        g_InControlState.SelectedLeg,
        g_InControlState.BodyPos.x, g_InControlState.BodyPos.z,
        g_InControlState.BodyRot1.x, g_InControlState.BodyRot1.z);        
        g_InControlState.SLLeg.x = 0;
        g_InControlState.SLLeg.y = 0;
        g_InControlState.SLLeg.z = 0;
        g_InControlState.fSLHold = false;
        command_single_leg = false;
        showFeedbackLED(1, 100); 
    }
    if (g_InControlState.SelectedLeg < CNT_LEGS) {
        g_InControlState.SLLeg.x = (sustained_axes_lx / 256);
        g_InControlState.SLLeg.y = std::min(static_cast<float>(-sustained_axes_ry) / 256.0f, -20.0f);
        g_InControlState.SLLeg.z = (sustained_axes_ly / 256);
        PRINT("Single-leg %d: SLLeg.x=%ld, SLLeg.y=%ld, SLLeg.z=%ld", 
            g_InControlState.SelectedLeg, g_InControlState.SLLeg.x, 
            g_InControlState.SLLeg.y, g_InControlState.SLLeg.z);
        PRINTUDP("Single-leg %d: SLLeg.x=%ld, SLLeg.y=%ld, SLLeg.z=%ld", 
            g_InControlState.SelectedLeg, g_InControlState.SLLeg.x, 
            g_InControlState.SLLeg.y, g_InControlState.SLLeg.z);
    }  

//=================================================================================================
// Knop Balansmodus
//=================================================================================================      
    if (button_balance_mode) {
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        PRINT(g_InControlState.BalanceMode ? "Balance mode on" : "Balance mode off");
        PRINTUDP(g_InControlState.BalanceMode ? "Balance mode on command" : "Balance mode off command");
        showFeedbackLED(1, 100); 
    }    

//=================================================================================================
// Loopmodus
//=================================================================================================      
    if (ControlMode == WALKMODE) {
        if (g_InControlState.SelectedLeg == 255) {
#ifdef EXTENDED_MOVEMENT
            SmDiv = 2;   //Smoothingwaarde divider
            SmMargin = 20;

            // Constanten voor normalisatie en schaling
            double min_val = -32767.0;  // Minimale joystick invoerwaarde
            double max_val = 32767.0;   // Maximale joystick invoerwaarde
            double factor_min = 1.0;    // Minimale schalingsfactor

            // Tel het aantal actieve assen
            int num_engaged = 0;
            if (sustained_axes_lx != 0) num_engaged++;
            if (sustained_axes_ly != 0) num_engaged++;
            if (sustained_axes_rx != 0) num_engaged++;
            
            double factor_max;
            if (num_engaged <= 1) {
                factor_max = 1.0;  // Minimale schaling bij 0 of 1 actieve as
            } else if (num_engaged == 2) {
                factor_max = 1.5;  // Matige schaling bij 2 actieve assen
            } else {  // num_engaged == 3
                factor_max = 2.0;  // Sterkere schaling bij 3 actieve assen
            }        

            // Stap 1: Normaliseer input-assen naar bereik [0, 1]
            double norm_lx = (sustained_axes_lx - min_val) / (max_val - min_val);
            double norm_ly = (sustained_axes_ly - min_val) / (max_val - min_val);
            double norm_rx = (sustained_axes_rx - min_val) / (max_val - min_val);

            // Stap 2: Bereken activiteitsniveaus (0 tot 1)
            double activity_lx = fabs(norm_lx - 0.5) / 0.5;
            double activity_ly = fabs(norm_ly - 0.5) / 0.5;
            double activity_rx = fabs(norm_rx - 0.5) / 0.5;

            // Stap 3: Bereken schalingsfactoren gebaseerd op andere actieve assen
            // Voor x (beïnvloed door ly en rx)
            double sum_activity_x = 0.0;
            int count_active_x = 0;
            if (activity_ly > 0) { sum_activity_x += activity_ly; count_active_x++; }
            if (activity_rx > 0) { sum_activity_x += activity_rx; count_active_x++; }
            double avg_activity_x = (count_active_x > 0) ? (sum_activity_x / count_active_x) : 0.0;
            double factor_x = factor_min + (factor_max - factor_min) * avg_activity_x;

            // Voor z (beïnvloed door lx en rx)
            double sum_activity_z = 0.0;
            int count_active_z = 0;
            if (activity_lx > 0) { sum_activity_z += activity_lx; count_active_z++; }
            if (activity_rx > 0) { sum_activity_z += activity_rx; count_active_z++; }
            double avg_activity_z = (count_active_z > 0) ? (sum_activity_z / count_active_z) : 0.0;
            double factor_z = factor_min + (factor_max - factor_min) * avg_activity_z;

            // Voor y (beïnvloed door lx en ly)
            double sum_activity_y = 0.0;
            int count_active_y = 0;
            if (activity_lx > 0) { sum_activity_y += activity_lx; count_active_y++; }
            if (activity_ly > 0) { sum_activity_y += activity_ly; count_active_y++; }
            double avg_activity_y = (count_active_y > 0) ? (sum_activity_y / count_active_y) : 0.0;
            double factor_y = factor_min + (factor_max - factor_min) * avg_activity_y;

            // Stap 4: Bereken basiswaarden
            double base_x = (-sustained_axes_lx / 256.0);  // Bijv. -32767 / 256 ≈ -127.996
            double base_z = (sustained_axes_ly / 256.0);   // Bijv. 32767 / 256 ≈ 127.996
            double base_y = (-sustained_axes_rx / 256.0);

            // Stap 5: Bereken geschaalde inputs (voor smoothing)
            double scaled_x = base_x / factor_x;
            double scaled_z = base_z / factor_z;
            double scaled_y = base_y / factor_y;

            // Stap 6: Bereken InputTimeDelay met verhoudingen van basiswaarden tot max_base
            double max_base = 32767.0 / 256.0; // Maximale basiswaarde
            double max_scaled_x = max_base / factor_x; // Maximale geschaalde waarde voor x
            double max_scaled_z = max_base / factor_z; // Maximale geschaalde waarde voor z
            double max_scaled_y = max_base / factor_y; // Maximale geschaalde waarde voor y

            double ratio_x = fabs(scaled_x) / max_scaled_x;
            double ratio_z = fabs(scaled_z) / max_scaled_z;
            double ratio_y = fabs(scaled_y) / max_scaled_y;
            double max_ratio = fmax(fmax(ratio_x, ratio_z), ratio_y);

            g_InControlState.InputTimeDelay = 128 * (1 - max_ratio);

            // Stap 7: Werk TravelLength bij met gesmoothde en aangepaste waarden
            g_InControlState.TravelLength.x = SmoothControl(scaled_x, g_InControlState.TravelLength.x, SmDiv, SmMargin) * 1.0;
            if (g_InControlState.GaitType == 5) { 
                g_InControlState.TravelLength.z = SmoothControl(scaled_z, g_InControlState.TravelLength.z, SmDiv, SmMargin) * 1.15;
            }
            else
            {
                g_InControlState.TravelLength.z = SmoothControl(scaled_z, g_InControlState.TravelLength.z, SmDiv, SmMargin) * 1.1;
            }

            //g_InControlState.TravelLength.x = (scaled_x / factor_x) * 1.0;
            //g_InControlState.TravelLength.z = (scaled_z / factor_z) * 1.1;            
            g_InControlState.TravelLength.y = (scaled_y / factor_y) / 4.3;   

#else            
            g_InControlState.TravelLength.x = SmoothControl((-sustained_axes_lx / 256) / 1.15, g_InControlState.TravelLength.x, SmDiv, SmMargin);
            g_InControlState.TravelLength.z = SmoothControl((sustained_axes_ly / 256) / 1.15, g_InControlState.TravelLength.z, SmDiv, SmMargin);
            g_InControlState.TravelLength.y = -((sustained_axes_rx / 256) / 5.3);
#endif
            if (!DoubleTravelOn) {
                g_InControlState.TravelLength.x /= 1.3;
                g_InControlState.TravelLength.z /= 1.3;
                g_InControlState.TravelLength.y = -((sustained_axes_rx / 256) / 6.5);         
            }        

            PRINT("Command walk mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, factor_max=%.1f - %s - %s%s",  
                sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, factor_max,
                s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
                DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
                variablelegliftheight ? "Leg lift height high command" : "Leg lift height low"); 
            PRINTUDP("Command walk mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, factor_max=%.1f - %s - %s%s",  
                sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, factor_max,
                s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
                DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
                variablelegliftheight ? "Leg lift height high command" : "Leg lift height low"); 

            g_InControlState.InputTimeDelay = 128 - max(max(abs((sustained_axes_lx/256)), abs((sustained_axes_ly/256))), abs((sustained_axes_rx/256))); 
        }
    }

//=================================================================================================
// Lichaamsbewegingsmodus
//=================================================================================================    
    if (ControlMode == BODYMOVEMODE) {
        SmDiv = 6;   //Smoothingwaarde divider
        SmMargin = 3;

        // Constanten
        double min_val = -32767.0;    // Minimale invoerwaarde
        double max_val = 32767.0;     // Maximale invoerwaarde
        double factor_min = 1.0;      // Minimale schalingsfactor (geen demping)
        double base_factor_max = 1.0; // Basis maximale schalingsfactor
        double increment_per_axis = 0.30; // Increment per actieve as

        // Normaliseer elke as (0 tot 1)
        double norm_lx = (sustained_axes_lx - min_val) / (max_val - min_val);
        double norm_ly = (sustained_axes_ly - min_val) / (max_val - min_val);
        double norm_rx = (sustained_axes_rx - min_val) / (max_val - min_val);
        double norm_ry = (sustained_axes_ry - min_val) / (max_val - min_val);
        double norm_sx = (sustained_axes_sx - min_val) / (max_val - min_val);
        double norm_sy = (sustained_axes_sy - min_val) / (max_val - min_val);

        // Bereken activiteitsniveaus (0 tot 1)
        double activity_lx = fabs(norm_lx - 0.5) / 0.5;
        double activity_ly = fabs(norm_ly - 0.5) / 0.5;
        double activity_rx = fabs(norm_rx - 0.5) / 0.5;
        double activity_ry = fabs(norm_ry - 0.5) / 0.5;
        double activity_sx = fabs(norm_sx - 0.5) / 0.5;
        double activity_sy = fabs(norm_sy - 0.5) / 0.5;

        // Tel totaal aantal actieve assen (voor dynamische factor_max)
        int total_active_axes = 0;
        if (activity_lx > 0) total_active_axes++;
        if (activity_ly > 0) total_active_axes++;
        if (activity_rx > 0) total_active_axes++;
        if (activity_ry > 0) total_active_axes++;
        if (activity_sx > 0) total_active_axes++;
        if (activity_sy > 0) total_active_axes++;

        // Bereken dynamische factor_max gebaseerd op aantal actieve assen
        double factor_max = base_factor_max + (increment_per_axis * total_active_axes);

        // Bereken schalingsfactoren gebaseerd op andere actieve assen
        // Voor lx
        double sum_activity_lx = 0.0;
        int count_active_lx = 0;
        if (activity_ly > 0) { sum_activity_lx += activity_ly; count_active_lx++; }
        if (activity_rx > 0) { sum_activity_lx += activity_rx; count_active_lx++; }
        if (activity_ry > 0) { sum_activity_lx += activity_ry; count_active_lx++; }
        if (activity_sx > 0) { sum_activity_lx += activity_sx; count_active_lx++; }
        if (activity_sy > 0) { sum_activity_lx += activity_sy; count_active_lx++; }
        double avg_activity_lx = (count_active_lx > 0) ? (sum_activity_lx / count_active_lx) : 0.0;
        double factor_lx = factor_min + (factor_max - factor_min) * avg_activity_lx;

        // Voor ly
        double sum_activity_ly = 0.0;
        int count_active_ly = 0;
        if (activity_lx > 0) { sum_activity_ly += activity_lx; count_active_ly++; }
        if (activity_rx > 0) { sum_activity_ly += activity_rx; count_active_ly++; }
        if (activity_ry > 0) { sum_activity_ly += activity_ry; count_active_ly++; }
        if (activity_sx > 0) { sum_activity_ly += activity_sx; count_active_ly++; }
        if (activity_sy > 0) { sum_activity_ly += activity_sy; count_active_ly++; }
        double avg_activity_ly = (count_active_ly > 0) ? (sum_activity_ly / count_active_ly) : 0.0;
        double factor_ly = factor_min + (factor_max - factor_min) * avg_activity_ly;

        // Voor rx
        double sum_activity_rx = 0.0;
        int count_active_rx = 0;
        if (activity_lx > 0) { sum_activity_rx += activity_lx; count_active_rx++; }
        if (activity_ly > 0) { sum_activity_rx += activity_ly; count_active_rx++; }
        if (activity_ry > 0) { sum_activity_rx += activity_ry; count_active_rx++; }
        if (activity_sx > 0) { sum_activity_rx += activity_sx; count_active_rx++; }
        if (activity_sy > 0) { sum_activity_rx += activity_sy; count_active_rx++; }
        double avg_activity_rx = (count_active_rx > 0) ? (sum_activity_rx / count_active_rx) : 0.0;
        double factor_rx = factor_min + (factor_max - factor_min) * avg_activity_rx;

        // Voor ry
        double sum_activity_ry = 0.0;
        int count_active_ry = 0;
        if (activity_lx > 0) { sum_activity_ry += activity_lx; count_active_ry++; }
        if (activity_ly > 0) { sum_activity_ry += activity_ly; count_active_ry++; }
        if (activity_rx > 0) { sum_activity_ry += activity_rx; count_active_ry++; }
        if (activity_sx > 0) { sum_activity_ry += activity_sx; count_active_ry++; }
        if (activity_sy > 0) { sum_activity_ry += activity_sy; count_active_ry++; }
        double avg_activity_ry = (count_active_ry > 0) ? (sum_activity_ry / count_active_ry) : 0.0;
        double factor_ry = factor_min + (factor_max - factor_min) * avg_activity_ry;

        // Voor sx
        double sum_activity_sx = 0.0;
        int count_active_sx = 0;
        if (activity_lx > 0) { sum_activity_sx += activity_lx; count_active_sx++; }
        if (activity_ly > 0) { sum_activity_sx += activity_ly; count_active_sx++; }
        if (activity_rx > 0) { sum_activity_sx += activity_rx; count_active_sx++; }
        if (activity_ry > 0) { sum_activity_sx += activity_ry; count_active_sx++; }
        if (activity_sy > 0) { sum_activity_sx += activity_sy; count_active_sx++; }
        double avg_activity_sx = (count_active_sx > 0) ? (sum_activity_sx / count_active_sx) : 0.0;
        double factor_sx = factor_min + (factor_max - factor_min) * avg_activity_sx;

        // Voor sy
        double sum_activity_sy = 0.0;
        int count_active_sy = 0;
        if (activity_lx > 0) { sum_activity_sy += activity_lx; count_active_sy++; }
        if (activity_ly > 0) { sum_activity_sy += activity_ly; count_active_sy++; }
        if (activity_rx > 0) { sum_activity_sy += activity_rx; count_active_sy++; }
        if (activity_ry > 0) { sum_activity_sy += activity_ry; count_active_sy++; }
        if (activity_sx > 0) { sum_activity_sy += activity_sx; count_active_sy++; }
        double avg_activity_sy = (count_active_sy > 0) ? (sum_activity_sy / count_active_sy) : 0.0;
        double factor_sy = factor_min + (factor_max - factor_min) * avg_activity_sy;

        // Pas dynamische schaling toe op de uitvoer
        g_InControlState.BodyPos.x = SmoothControl((sustained_axes_lx / 256.0 / factor_lx) / 1.8, g_InControlState.BodyPos.x, SmDiv, SmMargin);
        g_InControlState.BodyPos.z = SmoothControl(-(sustained_axes_ly / 256.0 / factor_ly) / 1.8, g_InControlState.BodyPos.z, SmDiv, SmMargin);
        g_InControlState.BodyRot1.y = SmoothControl((sustained_axes_rx / 256.0 / factor_rx) * 1.2, g_InControlState.BodyRot1.y, SmDiv, SmMargin);
        g_InControlState.BodyRot1.x = SmoothControl((sustained_axes_sy / 256.0 / factor_sy) * 1.2, g_InControlState.BodyRot1.x, SmDiv, SmMargin);
        g_InControlState.BodyRot1.z = SmoothControl((sustained_axes_sx / 256.0 / factor_sx) * 1.2, g_InControlState.BodyRot1.z, SmDiv, SmMargin);        
        g_BodyYShift = SmoothControl((sustained_axes_ry / 256.0 / factor_ry) / 2.5, g_BodyYShift, SmDiv, SmMargin);

        g_InControlState.InputTimeDelay = (128 - max(max(abs((sustained_axes_lx/256)), abs((sustained_axes_ly/256))), abs((sustained_axes_rx/256))))*2;

        PRINT("Command bodymove mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, sustained_axes_ry=%d, sustained_axes_sx=%d, sustained_axes_sy=%d, factor_max=%.2f",  
            sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, sustained_axes_rx, sustained_axes_sx, sustained_axes_sy, factor_max); 
        PRINTUDP("Command bodymove mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, sustained_axes_ry=%d, sustained_axes_sx=%d, sustained_axes_sy=%d, factor_max=%.2f",  
            sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, sustained_axes_rx, sustained_axes_sx, sustained_axes_sy, factor_max);          
    }

//=================================================================================================
// Gemengde Modus
//=================================================================================================    
    if (ControlMode == MIXEDMODE) {
        SmDiv = 4;   //Smoothingwaarde divider
        SmMargin = 20;

        if (g_InControlState.SelectedLeg == 255) {
            // Constanten voor normalisatie en schaling
            double min_val = -32767.0;  // Minimale joystick invoerwaarde
            double max_val = 32767.0;   // Maximale joystick invoerwaarde
            double factor_min = 1.0;    // Minimale schalingsfactor

            // Tel het aantal actieve assen
            int num_engaged = 0;
            if (sustained_axes_lx != 0) num_engaged++;
            if (sustained_axes_ly != 0) num_engaged++;
            if (sustained_axes_rx != 0) num_engaged++;
            
            double factor_max;
            if (num_engaged <= 1) {
                factor_max = 1.0;  // Minimale schaling bij 0 of 1 actieve as
            } else if (num_engaged == 2) {
                factor_max = 1.5;  // Matige schaling bij 2 actieve assen
            } else {  // num_engaged == 3
                factor_max = 2.0;  // Sterkere schaling bij 3 actieve assen
            }        

            // Stap 1: Normaliseer input-assen naar bereik [0, 1]
            double norm_lx = (sustained_axes_lx - min_val) / (max_val - min_val);
            double norm_ly = (sustained_axes_ly - min_val) / (max_val - min_val);
            double norm_rx = (sustained_axes_rx - min_val) / (max_val - min_val);

            // Stap 2: Bereken activiteitsniveaus (0 tot 1)
            double activity_lx = fabs(norm_lx - 0.5) / 0.5;
            double activity_ly = fabs(norm_ly - 0.5) / 0.5;
            double activity_rx = fabs(norm_rx - 0.5) / 0.5;

            // Stap 3: Bereken schalingsfactoren gebaseerd op andere actieve assen
            // Voor x (beïnvloed door ly en rx)
            double sum_activity_x = 0.0;
            int count_active_x = 0;
            if (activity_ly > 0) { sum_activity_x += activity_ly; count_active_x++; }
            if (activity_rx > 0) { sum_activity_x += activity_rx; count_active_x++; }
            double avg_activity_x = (count_active_x > 0) ? (sum_activity_x / count_active_x) : 0.0;
            double factor_x = factor_min + (factor_max - factor_min) * avg_activity_x;

            // Voor z (beïnvloed door lx en rx)
            double sum_activity_z = 0.0;
            int count_active_z = 0;
            if (activity_lx > 0) { sum_activity_z += activity_lx; count_active_z++; }
            if (activity_rx > 0) { sum_activity_z += activity_rx; count_active_z++; }
            double avg_activity_z = (count_active_z > 0) ? (sum_activity_z / count_active_z) : 0.0;
            double factor_z = factor_min + (factor_max - factor_min) * avg_activity_z;

            // Voor y (beïnvloed door lx en ly)
            double sum_activity_y = 0.0;
            int count_active_y = 0;
            if (activity_lx > 0) { sum_activity_y += activity_lx; count_active_y++; }
            if (activity_ly > 0) { sum_activity_y += activity_ly; count_active_y++; }
            double avg_activity_y = (count_active_y > 0) ? (sum_activity_y / count_active_y) : 0.0;
            double factor_y = factor_min + (factor_max - factor_min) * avg_activity_y;

            // Stap 4: Bereken basiswaarden
            double base_x = (-sustained_axes_lx / 256.0);  // Bijv. -32767 / 256 ≈ -127.996
            double base_z = (sustained_axes_ly / 256.0);   // Bijv. 32767 / 256 ≈ 127.996
            double base_y = (-sustained_axes_rx / 256.0);

            // Stap 5: Bereken geschaalde inputs (voor smoothing)
            double scaled_x = base_x / factor_x;
            double scaled_z = base_z / factor_z;
            double scaled_y = base_y / factor_y;

            // Stap 6: Bereken InputTimeDelay met verhoudingen van basiswaarden tot max_base
            double max_base = 32767.0 / 256.0; // Maximale basiswaarde
            double max_scaled_x = max_base / factor_x; // Maximale geschaalde waarde voor x
            double max_scaled_z = max_base / factor_z; // Maximale geschaalde waarde voor z
            double max_scaled_y = max_base / factor_y; // Maximale geschaalde waarde voor y

            double ratio_x = fabs(scaled_x) / max_scaled_x;
            double ratio_z = fabs(scaled_z) / max_scaled_z;
            double ratio_y = fabs(scaled_y) / max_scaled_y;
            double max_ratio = fmax(fmax(ratio_x, ratio_z), ratio_y);

            g_InControlState.InputTimeDelay = 128 * (1 - max_ratio);

            // Stap 7: Werk TravelLength en rotaties bij met gesmoothde waarden
            g_InControlState.TravelLength.x = SmoothControl(scaled_x, g_InControlState.TravelLength.x, SmDiv, SmMargin) * 1.0;
            g_InControlState.TravelLength.z = SmoothControl(scaled_z, g_InControlState.TravelLength.z, SmDiv, SmMargin) * 1.1;
            g_InControlState.TravelLength.y = (scaled_y / factor_y) / 3.9;   
            g_InControlState.BodyRot1.y = SmoothControl((sustained_axes_rx / 256.0), g_InControlState.BodyRot1.y, SmDiv, SmMargin);
            g_InControlState.BodyRot1.x = SmoothControl((sustained_axes_sy / 256.0), g_InControlState.BodyRot1.x, SmDiv, SmMargin);
            g_InControlState.BodyRot1.z = SmoothControl((sustained_axes_sx / 256.0), g_InControlState.BodyRot1.z, SmDiv, SmMargin);
            g_BodyYShift = SmoothControl((sustained_axes_ry / 256.0) / 5.0, g_BodyYShift, SmDiv, SmMargin);         

            if (!DoubleTravelOn) {
                g_InControlState.TravelLength.x /= 1.3;
                g_InControlState.TravelLength.z /= 1.3;
                g_InControlState.TravelLength.y = -((sustained_axes_rx / 256) / 6.5);
            }

            PRINT("Command mixed mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, factor_max=%.1f - %s - %s%s",  
                sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, factor_max,
                s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
                DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
                variablelegliftheight ? "Leg lift height high command" : "Leg lift height low"); 
            PRINTUDP("Command mixed mode - sustained_axes_lx=%d: sustained_axes_ly=%d, sustained_axes_rx=%d, factor_max=%.1f - %s - %s%s",  
                sustained_axes_lx, sustained_axes_ly, sustained_axes_rx, factor_max,
                s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
                DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
                variablelegliftheight ? "Leg lift height high command" : "Leg lift height low");             
        }

        g_InControlState.InputTimeDelay = (128 - max(max(abs((sustained_axes_lx/256)), abs((sustained_axes_ly/256))), abs((sustained_axes_rx/256))))/2;            
    }

//=================================================================================================
// Camera Modus
//=================================================================================================    
if (ControlMode == CAMERAMODE) {
    SmDiv = 2;   //Smoothingwaarde divider
    SmMargin = 20;

    FaceData current_faces;
    {
        std::lock_guard<std::mutex> lock(face_mutex);
        current_faces = latest_faces;
    }    

    // Resolution-independent: center is (VIDEO_STREAM_WIDTH/2, VIDEO_STREAM_HEIGHT/2).
    // Offset from center: negative X = face left → turn left; negative Y = face above → tilt up.
    const double center_x = (double)VIDEO_STREAM_WIDTH  / 2.0;
    const double center_y = (double)VIDEO_STREAM_HEIGHT / 2.0;

    // === PI CONTROLLER (resolution-independent: offsets normalized to ~[-1,1]) ===
    static double integral_turn  = 0.0;
    static double integral_tilt  = 0.0;
    static double last_control_turn = 0.0;
    static double last_control_tilt = 0.0;
    static int    no_face_count  = 0;

    double raw_x_offset = 0.0;
    double raw_y_offset = 0.0;
    double control_turn = 0.0;
    double control_tilt = 0.0;

    bool faceDetected = (current_faces.num_faces > 0);

    if (faceDetected && center_x > 0 && center_y > 0) {
        raw_x_offset = (current_faces.centroid_x - center_x) / center_x;
        raw_y_offset = (current_faces.centroid_y - center_y) / center_y;
        no_face_count = 0;
    } else {
        no_face_count++;   // Bug fix: was never incremented — controller never returned to center
    }

    // PI gains for normalized offset [-1, 1]: same response regardless of VIDEO_STREAM_WIDTH/HEIGHT
    const double Kp_turn = FACE_KP_TURN;
    const double Ki_turn = FACE_KI_TURN;
    
    const double Kp_tilt = FACE_KP_TILT;
    const double Ki_tilt = FACE_KI_TILT;
    
    const double MAX_INTEGRAL_NORM = 5.0;

    const int MAX_NO_FACE_FRAMES = 24;

    if (faceDetected || no_face_count <= MAX_NO_FACE_FRAMES) {
        if (faceDetected) {
            integral_turn += raw_x_offset;
            integral_tilt += raw_y_offset;

            if (integral_turn > MAX_INTEGRAL_NORM)  integral_turn = MAX_INTEGRAL_NORM;
            if (integral_turn < -MAX_INTEGRAL_NORM) integral_turn = -MAX_INTEGRAL_NORM;
            if (integral_tilt > MAX_INTEGRAL_NORM)  integral_tilt = MAX_INTEGRAL_NORM;
            if (integral_tilt < -MAX_INTEGRAL_NORM) integral_tilt = -MAX_INTEGRAL_NORM;

            control_turn = Kp_turn * raw_x_offset + Ki_turn * integral_turn;
            control_tilt = Kp_tilt * raw_y_offset + Ki_tilt * integral_tilt;
        } else {
            control_turn = last_control_turn;
            control_tilt = last_control_tilt;
        }
    } else {
        integral_turn = integral_tilt = 0.0;
        control_turn = control_tilt = 0.0;
    }

    last_control_turn = control_turn;
    last_control_tilt = control_tilt;

    if (control_turn > 128.0)  control_turn = 128.0;
    if (control_turn < -128.0) control_turn = -128.0;
    if (control_tilt > 128.0)  control_tilt = 128.0;
    if (control_tilt < -128.0) control_tilt = -128.0;

    g_InControlState.BodyRot1.y = (long)control_turn * 1.5;
    g_InControlState.BodyRot1.x = (long)control_tilt * 1.5;
    g_InControlState.BodyPos.z = abs((long)(control_turn * 1.5) / 2);
    
#ifdef CAMERAWALKENABLE       
    if (abs(g_InControlState.BodyRot1.y) >= 96) {
        if (DoubleTravelOn) {
            g_InControlState.TravelLength.y  = - ((control_turn) / 3);
        }
        else {
            g_InControlState.TravelLength.y  = - ((control_turn) / 5);
        }
    }
    if (abs(g_InControlState.BodyRot1.y) < 32) {
        g_InControlState.TravelLength.y  = 0;
    }
    if (!faceDetected) {
        g_InControlState.TravelLength.y  = 0;
    }
#endif  

    g_InControlState.InputTimeDelay = 0;

    int log_x = faceDetected ? (int)(current_faces.centroid_x - center_x) : 0;
    int log_y = faceDetected ? (int)(current_faces.centroid_y - center_y) : 0;
    PRINT("Command camera mode -  Face offset: X=%d (turn), Y=%d (tilt), control_turn=%d, sustcontrol_tilt=%d - %s - %s%s",
        log_x, log_y, (int)control_turn, (int)control_tilt,
        s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
        DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
        variablelegliftheight ? "Leg lift height high command" : "Leg lift height low");       
    PRINTUDP("Command camera mode -  Face offset: X=%d (turn), Y=%d (tilt), control_turn=%d, sustcontrol_tilt=%d - %s - %s%s",
        log_x, log_y, (int)control_turn, (int)control_tilt,
        s_asGateNames[g_InControlState.GaitType < NUM_GAITS ? g_InControlState.GaitType : 0],
        DoubleTravelOn ? "Double travel on : " : "Double travel off : ",
        variablelegliftheight ? "Leg lift height high command" : "Leg lift height low");     
}

//=================================================================================================
// Geen Modus
//=================================================================================================    
    if (ControlMode == NOMODE) {
        fAdjustLegPositions = false;
        PRINT("Command no mode selected");
        PRINTUDP("Command no mode selected command");
    }    
    
//=================================================================================================
// Andere functies
//=================================================================================================    
    g_InControlState.BodyPos.y = max(g_BodyYOffset + g_BodyYShift, 0);

    // Pas pootposities aan indien nodig
    if (sLegInitXZAdjust || sLegInitAngleAdjust) {
        if (!g_InControlState.ForceGaitStepCnt) {
            if (sLegInitXZAdjust) {
                g_fDynamicLegXZLength = true;
                sLegInitXZAdjust += GetLegsXZLength();
            }
#ifdef ADJUSTABLE_LEG_ANGLES
            if (sLegInitAngleAdjust)
                RotateLegInitAngles(sLegInitAngleAdjust);
#endif
            AdjustLegPositions((word)(sLegInitXZAdjust < 0 ? 0 : sLegInitXZAdjust));
        }
    }

    if (fAdjustLegPositions && !g_fDynamicLegXZLength)
        AdjustLegPositionsToBodyHeight();

    // Schakel robot uit bij timeout
    if (currentTime - g_ulLastMsgTime > TIMEOUT_SERVO && g_InControlState.fRobotOn) {
        g_InControlState.fControllerInUse = true;
        if (led_pin2 && gpiod_line_set_value(led_pin2, 0) < 0) {
            PRINT("Failed to set LED_PIN2: %s", strerror(errno));
        }
        showFeedbackLED(1, 100);
        ControllerTurnRobotOff();
    }

    g_InControlState.fPrev_RobotUp = g_InControlState.RobotUp;
}

//=================================================================================================
// LED control function
//=================================================================================================
static void setLEDState(int redState, int duration_ms) {
#ifdef OPT_LED
    if (!led_red_pin) {
        PRINT("Red LED pin not initialized");
        return;
    }
    PRINT("Setting red LED (GPIO %d) to %d for %d ms", LED_RED_PIN, redState, duration_ms);
    ledState = redState;
    if (gpiod_line_set_value(led_red_pin, ledState) < 0) {
        PRINT("Failed to set red LED: %s", strerror(errno));
    }
    if (redState == 1) {
        ledTurnOffTime = millis() + duration_ms;
    } else {
        ledTurnOffTime = 0;
    }
#endif
}

void showFeedbackLED(int red1, int dur1) {
#ifdef OPT_LED
    if (!led_red_pin) {
        PRINT("Red LED pin not initialized");
        return;
    }
    setLEDState(red1, dur1);
#endif
}

void updateLEDState() {
#ifdef OPT_LED
    if (!led_red_pin) {
        return;
    }
    if (ledState == 1 && ledTurnOffTime > 0 && millis() >= ledTurnOffTime) {
        ledState = 0;
        if (gpiod_line_set_value(led_red_pin, ledState) < 0) {
            PRINT("Failed to turn off red LED: %s", strerror(errno));
        }
        ledTurnOffTime = 0;
        PRINT("Red LED (GPIO %d) turned off", LED_RED_PIN);
    }
#endif
}
    
//=================================================================================================
// Opruimfunctie
//=================================================================================================
void WifiInputController::Cleanup(void) {
    if (udpSocket != -1) {
        PRINT("UDP socket closed");
        PRINTUDP("UDP socket closed");
        close(udpSocket);
        udpSocket = -1;
    }
    if (sendUdpSocket != -1) {
        PRINT("Sending UDP socket closed");
        PRINTUDP("Sending UDP socket closed");
        close(sendUdpSocket);
        sendUdpSocket = -1;
    }    
    if (led_red_pin) {
        gpiod_line_set_value(led_red_pin, 0); // Turn off red LED
        gpiod_line_release(led_red_pin);
        led_red_pin = NULL;
    }
    if (led_pin1) {
        gpiod_line_set_value(led_pin1, 0); // Turn off LED1
        gpiod_line_release(led_pin1);
        led_pin1 = NULL;
    }
    if (led_pin2) {
        gpiod_line_set_value(led_pin2, 0); // Turn off LED2
        gpiod_line_release(led_pin2);
        led_pin2 = NULL;
    }
    if (gpio_chip) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = NULL;
        PRINT("GPIO resources released");
        PRINTUDP("GPIO resources released");
    }
}

//=================================================================================================
// Functie om de robot te resetten
//=================================================================================================
void ResetHexapod() {
    g_BodyYShift = 0;                   // Reset Y-shift
    ControlMode = NOMODE;               // Stel modus in op geen
    HeightSpeedMode = NORM_NORM;        // Reset hoogte/snelheid
    DoubleTravelOn = false;             // Schakel dubbele staplengte uit
    g_fDynamicLegXZLength = false;      // Schakel dynamische XZ-lengte uit
    variablelegliftheight = false;      // Schakel variabele lifthoogte uit
    g_InControlState.BodyPos = {0, 0, 0};      // Reset lichaamspositie
    g_InControlState.BodyRot1 = {0, 0, 0};     // Reset lichaamsrotatie
    g_InControlState.TravelLength = {0, 0, 0}; // Reset staplengte
    g_InControlState.GaitType = 0;      // Stel gait in op standaard
    GaitSelect();                       // Selecteer gait
    g_InControlState.SpeedControl = 0;  // Reset snelheid
    g_InControlState.InputTimeDelay = 0;// Reset invoervertraging
    g_InControlState.ForceGaitStepCnt = 0; // Reset geforceerde gaitstappen
    g_InControlState.GaitStep = 1;      // Reset gaitstap
    g_InControlState.LegLiftHeight = 50;// Stel standaard lifthoogte in
    g_InControlState.SelectedLeg = 255; // Geen poot geselecteerd
    g_InControlState.BalanceMode = false;// Schakel balansmodus uit
    g_InControlState.fControllerInUse = false; // Controller niet in gebruik
    g_InControlState.SLLeg.x = 0;       // Reset enkel poot X
    g_InControlState.SLLeg.y = 0;       // Reset enkel poot Y
    g_InControlState.SLLeg.z = 0;       // Reset enkel poot Z
    g_InControlState.fSLHold = false;   // Schakel enkel poot vasthouden uit
    g_iLegInitIndex = 0x00;             // Reset pootinitialisatie-index

    PRINT("Hexapod reset to initial state");
    PRINTUDP("Hexapod reset to initial state command");
    showFeedbackLED(1, 100);
}

//=================================================================================================
// Functie om de robot uit te schakelen
//=================================================================================================
void ControllerTurnRobotOff(void) {
    ControlMode = NOMODE;
    g_InControlState.BodyPos = {0, 0, 0};
    g_InControlState.BodyRot1 = {0, 0, 0};
    g_InControlState.TravelLength = {0, 0, 0};
    g_BodyYOffset = 0;
    g_BodyYShift = 0;
    g_InControlState.BalanceMode = false;
    g_InControlState.GaitType = 0;
    GaitSelect();
    g_InControlState.SelectedLeg = 255;
    g_InControlState.fRobotOn = 0;
    showFeedbackLED(1, 100); 
    PRINT("Hexapod off");
    PRINTUDP("Hexapod off command"); 
}