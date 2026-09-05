//=======================================================================================
// Hexapod-besturing via WiFi (UDP)
// Bestand: Hexapod_Main.cpp
// Huidige datum: 19 november 2025
//=======================================================================================

// DEFINE_HEX_GLOBALS is defined in Hexapod_Code.cpp (single definition)

#if ARDUINO > 99
#include <Arduino.h>
#else
#endif

#include "Hex_Cfg.h"
#include "Hexapod.h"

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <systemd/sd-daemon.h>
#include <thread>          // <<< NEW
#include <atomic>          // <<< NEW

std::atomic<bool> g_fSignaled{false};
unsigned long g_loop_time = 0;

//=======================================================================================
// Declaratie van de camera-functie uit rpicam_udp.cpp
//=======================================================================================
extern int hexapod_rpicam_main(int argc, char** argv);

//=======================================================================================
// Globale vlag en thread voor de camera (alleen hier)
//=======================================================================================
std::thread g_camera_thread;          // assigned in setup(), joined at exit
std::atomic<bool> g_camera_running{false};

//=======================================================================================
// Signaalhandler (ongewijzigd)
//=======================================================================================
void SignalHandler(int sig){
    (void)sig;
    if (g_fSignaled) {
        // Second signal: clean shutdown is stuck — restore default and re-raise to die now
        signal(SIGINT,  SIG_DFL);
        signal(SIGQUIT, SIG_DFL);
        raise(sig);
        return;
    }
    g_fSignaled = true;
}

extern void setup(void);
extern void loop(void);
extern void cleanup(void);

//=======================================================================================
// Hoofdfunctie van het programma
//=======================================================================================
int main()
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = SignalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT,  &sigIntHandler, NULL);
    sigaction(SIGQUIT, &sigIntHandler, NULL);

    sd_notify(0, "READY=1");

    setup();   // ← camera will start here

    unsigned long previous_loop_start = 0;
    unsigned long last_watchdog_ping = 0;

    while (!g_fSignaled) {
        unsigned long loop_start = millis();
        if (previous_loop_start != 0) g_loop_time = loop_start - previous_loop_start;
        previous_loop_start = loop_start;
        // Rate-limit watchdog to 1 Hz (systemd doesn't need it every loop tick)
        if (loop_start - last_watchdog_ping >= 1000) {
            sd_notify(0, "WATCHDOG=1");
            last_watchdog_ping = loop_start;
        }
        loop();
    }

    cleanup();
    // Signal and join camera thread for clean shutdown
    hexapod_rpicam_stop();
    if (g_camera_thread.joinable())
        g_camera_thread.join();
    return 0;
}