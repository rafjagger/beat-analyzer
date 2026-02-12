#pragma once

/**
 * Pioneer Pro DJ Link Beat Receiver
 * 
 * Empfängt Beat-Pakete direkt vom Pioneer-Netzwerk (CDJ/DJM).
 * Implementiert ein Virtual CDJ um Master-Status zu erkennen.
 * 
 * Protokoll (UDP Broadcast):
 *   Port 50000: Keep-Alive (Virtual CDJ Announcement, gesendet alle 1.5s)
 *   Port 50001: Beat-Pakete (0x28, 96 Bytes, bei jedem Beat vom Player)
 *   Port 50002: Status-Pakete (CDJ ~200ms, enthält Master-Flag)
 * 
 * Architektur:
 *   - Ein Thread mit poll() über 3 Sockets
 *   - Keep-Alive alle 1.5s senden (Virtual CDJ, Device Nr. 0x07)
 *   - Beat-Pakete vom Master-Player an Callback weiterleiten
 *   - Status-Pakete parsen um Master-Player zu tracken
 *   - Fallback: Ohne Master-Info alle Beat-Pakete weiterleiten
 * 
 * Pioneer Beat-Paket (Type 0x28, 96 Bytes):
 *   Magic Header:  0x00-0x09  "Qspt1WmJOL"
 *   Packet Type:   0x0a       0x28 = Beat
 *   Device Name:   0x0b-0x1e  ASCII, null-padded
 *   Device Number: 0x21       Player 1-4, Mixer=0x21
 *   Pitch:         0x55-0x57  3 Bytes BE, neutral=0x100000
 *   BPM:           0x5a-0x5b  2 Bytes BE, ×100
 *   Beat-in-Bar:   0x5c       1-4
 * 
 * CDJ Status-Paket (Type 0x0a, ~0xd4 Bytes):
 *   Device Number: 0x21       Player 1-4
 *   Flags (F):     0x89       Bit 5 = Master
 *   BPM:           0x92-0x93  2 Bytes BE, ×100
 */

#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <cstdint>
#include <array>

namespace BeatAnalyzer {
namespace OSC {

/**
 * Empfangene Pioneer Beat-Daten
 */
struct PioneerBeat {
    uint8_t  deviceNumber = 0;     // Player 1-4, Mixer=0x21
    char     deviceName[21] = {};  // ASCII name
    double   trackBpm = 0.0;       // Track-BPM (ohne Pitch)
    double   effectiveBpm = 0.0;   // Effektives BPM (mit Pitch)
    double   pitchPercent = 0.0;   // Pitch in %
    uint8_t  beatWithinBar = 0;    // 1-4
    bool     isMaster = false;     // Ob dieser Player Master ist
    int64_t  timestamp = 0;        // ms since epoch
};

/**
 * Pioneer Pro DJ Link Receiver
 * 
 * Passiver Listener auf Port 50001 (Beat-Pakete) +
 * Virtual CDJ auf Port 50000/50002 für Master-Tracking.
 */
class PioneerReceiver {
public:
    using BeatCallback = std::function<void(const PioneerBeat&)>;
    
    PioneerReceiver();
    ~PioneerReceiver();
    
    PioneerReceiver(const PioneerReceiver&) = delete;
    PioneerReceiver& operator=(const PioneerReceiver&) = delete;
    
    /** Setze Virtual CDJ Device-Nummer (default: 0x07, vermeidet Konflikte mit CDJ 1-4) */
    void setDeviceNumber(uint8_t num) { m_deviceNumber = num; }
    
    /** Setze Virtual CDJ Name (max 20 Zeichen) */
    void setDeviceName(const std::string& name);
    
    /** Callback für empfangene Beats (nur vom Master-Player) */
    void setCallback(BeatCallback callback) { m_callback = callback; }
    
    /** Starte den Receiver (3 Sockets, 1 Thread) */
    bool start();
    
    /** Stoppe den Receiver */
    void stop();
    
    bool isRunning() const { return m_running; }
    
    /** Aktueller Master-Player (0 = keiner bekannt) */
    uint8_t getMasterPlayer() const { return m_masterPlayer.load(); }
    
    /** Letzte bekannte BPM vom Master */
    double getMasterBpm() const { return m_masterBpm.load(); }
    
    /** Anzahl empfangener Beat-Pakete seit Start */
    uint64_t getBeatCount() const { return m_beatCount.load(); }
    
    /** Anzahl erkannter Devices */
    int getDeviceCount() const;
    
private:
    // Pioneer Pro DJ Link Ports (Protokoll-Standard, nicht konfigurierbar)
    static constexpr int PORT_ANNOUNCE = 50000;
    static constexpr int PORT_BEAT     = 50001;
    static constexpr int PORT_STATUS   = 50002;
    
    // Magic Header: "Qspt1WmJOL"
    static constexpr uint8_t MAGIC_HEADER[10] = {
        0x51, 0x73, 0x70, 0x74, 0x31, 0x57, 0x6d, 0x4a, 0x4f, 0x4c
    };
    
    // Packet Types
    static constexpr uint8_t PKT_TYPE_BEAT      = 0x28;
    static constexpr uint8_t PKT_TYPE_CDJ_STATUS = 0x0a;
    static constexpr uint8_t PKT_TYPE_KEEPALIVE  = 0x06;
    
    // Beat Packet Offsets
    static constexpr int BEAT_PKT_SIZE       = 0x60;  // 96 bytes
    static constexpr int OFF_DEVICE_NAME     = 0x0b;
    static constexpr int OFF_DEVICE_NUM      = 0x21;
    static constexpr int OFF_PITCH           = 0x55;   // 3 bytes BE
    static constexpr int OFF_BPM             = 0x5a;   // 2 bytes BE, ×100
    static constexpr int OFF_BEAT_IN_BAR     = 0x5c;
    
    // CDJ Status Offsets
    static constexpr int OFF_STATUS_FLAGS    = 0x89;   // Bit 5 = Master
    static constexpr int OFF_STATUS_BPM      = 0x92;   // 2 bytes BE, ×100
    static constexpr int STATUS_MIN_SIZE     = 0xA0;   // Minimum für relevante Felder
    
    // Virtual CDJ Keep-Alive
    static constexpr int KEEPALIVE_SIZE      = 0x36;   // 54 bytes
    static constexpr int KEEPALIVE_INTERVAL_MS = 1500;
    
    // Device-Tracker: welche Player sind online und wer ist Master?
    struct DeviceInfo {
        uint8_t number = 0;
        char name[21] = {};
        bool isMaster = false;
        int64_t lastSeen = 0;    // ms timestamp
        double bpm = 0.0;
    };
    static constexpr int MAX_DEVICES = 8;
    
    // Sockets
    int m_sockAnnounce = -1;   // Port 50000 (send keep-alive, recv announcements)
    int m_sockBeat = -1;       // Port 50001 (recv beat packets)
    int m_sockStatus = -1;     // Port 50002 (recv status packets)
    
    // Thread
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    
    // Virtual CDJ Identity
    uint8_t m_deviceNumber = 0x07;
    char m_deviceName[21] = "beat-analyzer\0\0\0\0\0\0\0";
    
    // Network
    uint32_t m_localIp = 0;        // In network byte order
    uint8_t  m_localMac[6] = {};
    uint32_t m_broadcastAddr = 0;  // In network byte order
    
    // Master Tracking
    std::atomic<uint8_t> m_masterPlayer{0};   // 0 = unbekannt
    std::atomic<double> m_masterBpm{0.0};
    std::atomic<uint64_t> m_beatCount{0};
    
    // Device Table (geschützt durch m_deviceMutex)
    mutable std::mutex m_deviceMutex;
    std::array<DeviceInfo, MAX_DEVICES> m_devices{};
    int m_deviceCount = 0;
    
    // Callback
    BeatCallback m_callback;
    
    // ── Methoden ──
    
    /** Hauptschleife: poll() über 3 Sockets + Keep-Alive Timer */
    void recvLoop();
    
    /** Keep-Alive Paket senden (Virtual CDJ Announcement) */
    void sendKeepAlive();
    
    /** Beat-Paket (Type 0x28) parsen */
    void handleBeatPacket(const uint8_t* data, int len);
    
    /** Status-Paket (Type 0x0a) parsen — Master-Flag extrahieren */
    void handleStatusPacket(const uint8_t* data, int len);
    
    /** Keep-Alive-Paket (Type 0x06) parsen — Device-Discovery */
    void handleKeepAlivePacket(const uint8_t* data, int len);
    
    /** Device in Tabelle eintragen/aktualisieren */
    void updateDevice(uint8_t number, const char* name, bool isMaster, double bpm);
    
    /** Netzwerk-Interface erkennen (IP, MAC, Broadcast) */
    bool detectNetworkInterface();
    
    /** Keep-Alive Paket in Buffer schreiben */
    int buildKeepAlivePacket(uint8_t* buf);
    
    /** Prüfe Magic Header */
    static bool checkMagic(const uint8_t* data, int len);
    
    /** Lese 2-Byte big-endian unsigned */
    static uint16_t readBE16(const uint8_t* data);
    
    /** Lese 3-Byte big-endian unsigned (Pitch) */
    static uint32_t readBE24(const uint8_t* data);
    
    /** Lese 4-Byte big-endian unsigned */
    static uint32_t readBE32(const uint8_t* data);
};

} // namespace OSC
} // namespace BeatAnalyzer
