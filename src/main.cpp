/**
 * Beat Analyzer — Hauptprogramm
 *
 * Schlanker Entry-Point: Signal-Handler + App-Lifecycle.
 * Alle Logik lebt in BeatAnalyzerApp (app/).
 */

#include "app/beat_analyzer_app.h"
#include "config/config_files.h"

#include <iostream>
#include <csignal>

using namespace BeatAnalyzer;
using namespace BeatAnalyzer::Util;

// Globaler Shutdown-Flag (deklariert in beat_analyzer_app.h)
std::atomic<bool> BeatAnalyzer::g_running(true);

static void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        LOG_INFO("Shutdown signal empfangen...");
        g_running = false;
    }
}

// ============================================================================
// Hilfe
// ============================================================================

static void printUsage(const char* programName) {
    std::cout << "\nBeat Analyzer - Echtzeit Beat Detection mit OSC Output\n\n";
    std::cout << "Usage: " << programName << " [--config FILE]\n\n";
    std::cout << "Config: FILE, else ~/.config/beat-analyzer/beat-analyzer.env (or ./.env),\n";
    std::cout << "then ~/.config/beat-analyzer/conf.d/*.env; a later file wins:\n";
    std::cout << "  NUM_BPM_CHANNELS=1   Anzahl BPM-Eingänge (bpm_1 bis bpm_N)\n";
    std::cout << "  NUM_VU_CHANNELS=56   VU inputs (JACK ports vu_analog1_L … vu_in4_post_R; the patchbay wires REAPER outs to them, see the OSC truth's vu meaning)\n";
    std::cout << "\nOSC Ziele (beliebig viele):\n";
    std::cout << "  OSC_HOST_Name=host:port\n";
    std::cout << "  Beispiel: OSC_HOST_Protokol=127.0.0.1:9000\n";
    std::cout << "\nOSC Adressen:\n";
    std::cout << "  /beat           Beat Clock: iif beat(1-4), bar, bpm\n";
    std::cout << "  /vu/1-40        VU-Meter: ff peak, rms\n";
    std::cout << "\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════╗\n";
    std::cout << "║          BEAT ANALYZER v1.2.0             ║\n";
    std::cout << "║  Echtzeit Beat Detection + OSC Output     ║\n";
    std::cout << "╚═══════════════════════════════════════════╝\n";
    std::cout << "\n";
    
    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        printUsage(argv[0]);
        return 0;
    }
    
    std::optional<std::string> configFile;
    try {
        configFile = Config::configArgument(argc, argv);
    } catch (const std::invalid_argument& error) {
        std::cerr << error.what() << "\n";
        printUsage(argv[0]);
        return 2;
    }
    
    BeatAnalyzerApp app;
    
    if (!app.initialize(configFile)) {
        LOG_ERROR("Initialisierung fehlgeschlagen");
        return 1;
    }
    
    if (!app.run()) {
        LOG_ERROR("Ausführung fehlgeschlagen");
        return 1;
    }
    
    app.shutdown();
    
    return 0;
}
