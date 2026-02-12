#include "osc/pioneer_receiver.h"
#include "util/logging.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <chrono>
#include <algorithm>

// Linux: MAC-Adresse über ioctl
#include <sys/ioctl.h>
#include <net/if.h>

namespace BeatAnalyzer {
namespace OSC {

// Static constexpr members
constexpr uint8_t PioneerReceiver::MAGIC_HEADER[10];

// ============================================================================
// Constructor / Destructor
// ============================================================================

PioneerReceiver::PioneerReceiver() = default;

PioneerReceiver::~PioneerReceiver() {
    stop();
}

void PioneerReceiver::setDeviceName(const std::string& name) {
    std::memset(m_deviceName, 0, sizeof(m_deviceName));
    std::strncpy(m_deviceName, name.c_str(), 20);
}

int PioneerReceiver::getDeviceCount() const {
    std::lock_guard<std::mutex> lock(m_deviceMutex);
    return m_deviceCount;
}

// ============================================================================
// Byte reading helpers
// ============================================================================

bool PioneerReceiver::checkMagic(const uint8_t* data, int len) {
    if (len < 11) return false;
    return std::memcmp(data, MAGIC_HEADER, 10) == 0;
}

uint16_t PioneerReceiver::readBE16(const uint8_t* data) {
    return (static_cast<uint16_t>(data[0]) << 8) |
            static_cast<uint16_t>(data[1]);
}

uint32_t PioneerReceiver::readBE24(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 16) |
           (static_cast<uint32_t>(data[1]) <<  8) |
            static_cast<uint32_t>(data[2]);
}

uint32_t PioneerReceiver::readBE32(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) <<  8) |
            static_cast<uint32_t>(data[3]);
}

// ============================================================================
// Network Interface Detection
// ============================================================================

bool PioneerReceiver::detectNetworkInterface() {
    // Finde erstes nicht-Loopback IPv4 Interface
    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) < 0) {
        LOG_ERROR("Pioneer: getifaddrs() fehlgeschlagen");
        return false;
    }
    
    bool found = false;
    std::string ifaceName;
    
    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        if (ifa->ifa_flags & IFF_LOOPBACK) continue;
        if (!(ifa->ifa_flags & IFF_UP)) continue;
        if (!(ifa->ifa_flags & IFF_BROADCAST)) continue;
        
        auto* sa = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
        m_localIp = sa->sin_addr.s_addr;
        
        // Broadcast-Adresse
        if (ifa->ifa_broadaddr) {
            auto* bcast = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_broadaddr);
            m_broadcastAddr = bcast->sin_addr.s_addr;
        } else {
            // Fallback: Subnet-Broadcast berechnen
            auto* mask = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_netmask);
            m_broadcastAddr = m_localIp | ~mask->sin_addr.s_addr;
        }
        
        ifaceName = ifa->ifa_name;
        found = true;
        break;
    }
    
    freeifaddrs(ifaddr);
    
    if (!found) {
        LOG_ERROR("Pioneer: Kein geeignetes Netzwerk-Interface gefunden");
        return false;
    }
    
    // MAC-Adresse über ioctl holen
    int tmpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tmpSock >= 0) {
        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, ifaceName.c_str(), IFNAMSIZ - 1);
        if (ioctl(tmpSock, SIOCGIFHWADDR, &ifr) == 0) {
            std::memcpy(m_localMac, ifr.ifr_hwaddr.sa_data, 6);
        }
        close(tmpSock);
    }
    
    char ipStr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &m_localIp, ipStr, sizeof(ipStr));
    char bcastStr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &m_broadcastAddr, bcastStr, sizeof(bcastStr));
    
    LOG_INFO("Pioneer: Interface " + ifaceName + 
             " IP=" + ipStr + " Broadcast=" + bcastStr +
             " MAC=" + 
             std::to_string(m_localMac[0]) + ":" +
             std::to_string(m_localMac[1]) + ":" +
             std::to_string(m_localMac[2]) + ":" +
             std::to_string(m_localMac[3]) + ":" +
             std::to_string(m_localMac[4]) + ":" +
             std::to_string(m_localMac[5]));
    
    return true;
}

// ============================================================================
// Start / Stop
// ============================================================================

bool PioneerReceiver::start() {
    if (m_running) return true;
    
    // Netzwerk-Interface erkennen
    if (!detectNetworkInterface()) {
        return false;
    }
    
    // ── Socket 1: Port 50000 (Announce / Keep-Alive) ──
    m_sockAnnounce = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockAnnounce < 0) {
        LOG_ERROR("Pioneer: Socket 50000 Fehler");
        return false;
    }
    {
        int optval = 1;
        setsockopt(m_sockAnnounce, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        setsockopt(m_sockAnnounce, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval));
        
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT_ANNOUNCE);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(m_sockAnnounce, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            LOG_ERROR("Pioneer: Bind 50000 fehlgeschlagen (Port belegt?)");
            close(m_sockAnnounce);
            m_sockAnnounce = -1;
            return false;
        }
    }
    
    // ── Socket 2: Port 50001 (Beat-Pakete) ──
    m_sockBeat = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockBeat < 0) {
        LOG_ERROR("Pioneer: Socket 50001 Fehler");
        close(m_sockAnnounce); m_sockAnnounce = -1;
        return false;
    }
    {
        int optval = 1;
        setsockopt(m_sockBeat, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT_BEAT);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(m_sockBeat, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            LOG_ERROR("Pioneer: Bind 50001 fehlgeschlagen (Port belegt?)");
            close(m_sockAnnounce); m_sockAnnounce = -1;
            close(m_sockBeat); m_sockBeat = -1;
            return false;
        }
    }
    
    // ── Socket 3: Port 50002 (Status-Pakete) ──
    m_sockStatus = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockStatus < 0) {
        LOG_ERROR("Pioneer: Socket 50002 Fehler");
        close(m_sockAnnounce); m_sockAnnounce = -1;
        close(m_sockBeat); m_sockBeat = -1;
        return false;
    }
    {
        int optval = 1;
        setsockopt(m_sockStatus, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(PORT_STATUS);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(m_sockStatus, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            LOG_ERROR("Pioneer: Bind 50002 fehlgeschlagen (Port belegt?)");
            close(m_sockAnnounce); m_sockAnnounce = -1;
            close(m_sockBeat); m_sockBeat = -1;
            close(m_sockStatus); m_sockStatus = -1;
            return false;
        }
    }
    
    m_running = true;
    m_thread = std::thread(&PioneerReceiver::recvLoop, this);
    
    LOG_INFO("Pioneer Pro DJ Link Receiver gestartet");
    LOG_INFO("  Port 50000: Keep-Alive (Virtual CDJ #" + std::to_string(m_deviceNumber) + ")");
    LOG_INFO("  Port 50001: Beat-Pakete");
    LOG_INFO("  Port 50002: Status-Pakete (Master-Tracking)");
    
    return true;
}

void PioneerReceiver::stop() {
    if (!m_running) return;
    m_running = false;
    
    // Sockets schließen (unblocked recvfrom)
    if (m_sockAnnounce >= 0) { ::shutdown(m_sockAnnounce, SHUT_RDWR); close(m_sockAnnounce); m_sockAnnounce = -1; }
    if (m_sockBeat >= 0) { ::shutdown(m_sockBeat, SHUT_RDWR); close(m_sockBeat); m_sockBeat = -1; }
    if (m_sockStatus >= 0) { ::shutdown(m_sockStatus, SHUT_RDWR); close(m_sockStatus); m_sockStatus = -1; }
    
    if (m_thread.joinable()) {
        m_thread.join();
    }
    
    uint64_t beats = m_beatCount.load();
    uint8_t master = m_masterPlayer.load();
    LOG_INFO("Pioneer Receiver gestoppt (" + std::to_string(beats) + " Beats empfangen" +
             (master > 0 ? ", Master war Player " + std::to_string(master) : "") + ")");
}

// ============================================================================
// Keep-Alive Packet Builder
// ============================================================================

int PioneerReceiver::buildKeepAlivePacket(uint8_t* buf) {
    // CDJ Keep-Alive Paket: 54 Bytes (0x36)
    // Struktur gemäß dysentery Protokoll-Analyse:
    //   0x00-0x09: Magic Header
    //   0x0a:      Packet Type = 0x06 (Keep-Alive)
    //   0x0b-0x1e: Device Name (20 Bytes, null-padded)
    //   0x1f:      0x01
    //   0x20:      Subtype = 0x02
    //   0x21-0x22: 0x00 + length
    //   0x24:      Device Number
    //   0x25:      0x00
    //   0x26-0x2b: MAC Address (6 Bytes)
    //   0x2c-0x2f: IP Address (4 Bytes)
    //   0x30-0x35: Padding
    
    std::memset(buf, 0, KEEPALIVE_SIZE);
    
    // Magic Header
    std::memcpy(buf, MAGIC_HEADER, 10);
    
    // Packet Type: Keep-Alive
    buf[0x0a] = PKT_TYPE_KEEPALIVE;
    
    // Device Name (20 Bytes)
    std::memcpy(buf + OFF_DEVICE_NAME, m_deviceName, 20);
    
    // Required structure bytes
    buf[0x1f] = 0x01;
    buf[0x20] = 0x02;  // Subtype for keep-alive
    buf[0x21] = 0x00;
    buf[0x22] = 0x00;
    buf[0x23] = 0x11;  // Length remaining = 0x11 (17 bytes)
    
    // Device Number
    buf[0x24] = m_deviceNumber;
    
    // MAC Address
    std::memcpy(buf + 0x26, m_localMac, 6);
    
    // IP Address
    std::memcpy(buf + 0x2c, &m_localIp, 4);
    
    return KEEPALIVE_SIZE;
}

void PioneerReceiver::sendKeepAlive() {
    if (m_sockAnnounce < 0) return;
    
    uint8_t buf[KEEPALIVE_SIZE];
    int len = buildKeepAlivePacket(buf);
    
    struct sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(PORT_ANNOUNCE);
    dest.sin_addr.s_addr = m_broadcastAddr;
    
    sendto(m_sockAnnounce, buf, len, 0,
           reinterpret_cast<struct sockaddr*>(&dest), sizeof(dest));
}

// ============================================================================
// Receive Loop (3 Sockets + Keep-Alive Timer)
// ============================================================================

void PioneerReceiver::recvLoop() {
    uint8_t buf[600];  // CDJ-3000 status packets can be up to 0x200 (512) bytes
    
    // Sofort erstes Keep-Alive senden
    sendKeepAlive();
    auto lastKeepAlive = std::chrono::steady_clock::now();
    
    while (m_running) {
        // poll() über alle 3 Sockets
        struct pollfd pfds[3];
        pfds[0].fd = m_sockAnnounce;
        pfds[0].events = POLLIN;
        pfds[1].fd = m_sockBeat;
        pfds[1].events = POLLIN;
        pfds[2].fd = m_sockStatus;
        pfds[2].events = POLLIN;
        
        int ret = poll(pfds, 3, 50);  // 50ms timeout für Keep-Alive Timer
        
        // Keep-Alive senden (alle 1.5s)
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastKeepAlive).count() >= KEEPALIVE_INTERVAL_MS) {
            sendKeepAlive();
            lastKeepAlive = now;
        }
        
        if (ret <= 0) continue;
        
        // ── Port 50000: Announcements/Keep-Alive von anderen Devices ──
        if (pfds[0].revents & POLLIN) {
            ssize_t n = recvfrom(m_sockAnnounce, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n > 0 && checkMagic(buf, static_cast<int>(n))) {
                if (buf[0x0a] == PKT_TYPE_KEEPALIVE && n >= KEEPALIVE_SIZE) {
                    handleKeepAlivePacket(buf, static_cast<int>(n));
                }
            }
        }
        
        // ── Port 50001: Beat-Pakete ──
        if (pfds[1].revents & POLLIN) {
            ssize_t n = recvfrom(m_sockBeat, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n > 0 && checkMagic(buf, static_cast<int>(n))) {
                if (buf[0x0a] == PKT_TYPE_BEAT && n >= BEAT_PKT_SIZE) {
                    handleBeatPacket(buf, static_cast<int>(n));
                }
            }
        }
        
        // ── Port 50002: Status-Pakete ──
        if (pfds[2].revents & POLLIN) {
            ssize_t n = recvfrom(m_sockStatus, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n > 0 && checkMagic(buf, static_cast<int>(n))) {
                if (buf[0x0a] == PKT_TYPE_CDJ_STATUS && n >= STATUS_MIN_SIZE) {
                    handleStatusPacket(buf, static_cast<int>(n));
                }
            }
        }
    }
}

// ============================================================================
// Packet Handlers
// ============================================================================

void PioneerReceiver::handleBeatPacket(const uint8_t* data, int len) {
    if (len < BEAT_PKT_SIZE) return;
    
    PioneerBeat beat;
    
    // Device Number
    beat.deviceNumber = data[OFF_DEVICE_NUM];
    
    // Device Name (20 Bytes, null-terminated)
    std::memcpy(beat.deviceName, data + OFF_DEVICE_NAME, 20);
    beat.deviceName[20] = '\0';
    
    // BPM: 2 Bytes BE, ÷100
    uint16_t rawBpm = readBE16(data + OFF_BPM);
    if (rawBpm == 0xFFFF) return;  // Kein Track geladen
    beat.trackBpm = rawBpm / 100.0;
    
    // Pitch: 3 Bytes BE, neutral = 0x100000
    uint32_t rawPitch = readBE24(data + OFF_PITCH);
    double pitchMultiplier = static_cast<double>(rawPitch) / 0x100000;
    beat.pitchPercent = (pitchMultiplier - 1.0) * 100.0;
    beat.effectiveBpm = beat.trackBpm * pitchMultiplier;
    
    // Beat-within-bar (1-4)
    beat.beatWithinBar = data[OFF_BEAT_IN_BAR];
    if (beat.beatWithinBar < 1 || beat.beatWithinBar > 4) {
        beat.beatWithinBar = 1;  // Fallback
    }
    
    // Timestamp
    beat.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    // Master-Check: Ist dieser Player der aktuelle Master?
    uint8_t currentMaster = m_masterPlayer.load(std::memory_order_acquire);
    beat.isMaster = (currentMaster == 0) ||  // Kein Master bekannt → alle akzeptieren
                    (currentMaster == beat.deviceNumber);
    
    m_beatCount.fetch_add(1, std::memory_order_relaxed);
    
    // Nur Beats vom Master-Player weiterleiten
    // Oder von allen, wenn noch kein Master bekannt ist
    if (beat.isMaster) {
        m_masterBpm.store(beat.effectiveBpm, std::memory_order_release);
        
        if (m_callback) {
            m_callback(beat);
        }
    }
}

void PioneerReceiver::handleStatusPacket(const uint8_t* data, int len) {
    if (len < STATUS_MIN_SIZE) return;
    
    uint8_t deviceNum = data[OFF_DEVICE_NUM];
    if (deviceNum == 0 || deviceNum == m_deviceNumber) return;  // Eigene Pakete ignorieren
    
    // Status-Flags (Byte 0x89): Bit 5 = Master
    uint8_t flags = data[OFF_STATUS_FLAGS];
    bool isMaster = (flags & 0x20) != 0;  // Bit 5
    bool isPlaying = (flags & 0x40) != 0;  // Bit 6
    
    // BPM aus Status-Paket
    uint16_t rawBpm = readBE16(data + OFF_STATUS_BPM);
    double statusBpm = (rawBpm != 0xFFFF) ? (rawBpm / 100.0) : 0.0;
    
    // Device Name
    char name[21];
    std::memcpy(name, data + OFF_DEVICE_NAME, 20);
    name[20] = '\0';
    
    // Device-Tabelle aktualisieren
    updateDevice(deviceNum, name, isMaster, statusBpm);
    
    // Master-Tracking
    if (isMaster && isPlaying) {
        uint8_t oldMaster = m_masterPlayer.load(std::memory_order_relaxed);
        if (oldMaster != deviceNum) {
            m_masterPlayer.store(deviceNum, std::memory_order_release);
            LOG_INFO("Pioneer: Neuer Tempo-Master → Player " + std::to_string(deviceNum) + 
                     " (" + std::string(name) + ")" +
                     (statusBpm > 0 ? " BPM=" + std::to_string(static_cast<int>(statusBpm + 0.5)) : ""));
        }
    } else if (!isMaster) {
        // Wenn dieser Player Master war, aber jetzt nicht mehr
        uint8_t oldMaster = m_masterPlayer.load(std::memory_order_relaxed);
        if (oldMaster == deviceNum) {
            m_masterPlayer.store(0, std::memory_order_release);
            LOG_INFO("Pioneer: Player " + std::to_string(deviceNum) + " ist nicht mehr Master");
        }
    }
}

void PioneerReceiver::handleKeepAlivePacket(const uint8_t* data, int len) {
    if (len < KEEPALIVE_SIZE) return;
    
    uint8_t deviceNum = data[0x24];
    if (deviceNum == 0 || deviceNum == m_deviceNumber) return;  // Eigene Pakete ignorieren
    
    char name[21];
    std::memcpy(name, data + OFF_DEVICE_NAME, 20);
    name[20] = '\0';
    
    // Device registrieren (ohne Master-Info, nur Presence)
    updateDevice(deviceNum, name, false, 0.0);
}

void PioneerReceiver::updateDevice(uint8_t number, const char* name, bool isMaster, double bpm) {
    std::lock_guard<std::mutex> lock(m_deviceMutex);
    
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    // Existierendes Device suchen
    for (int i = 0; i < m_deviceCount; ++i) {
        if (m_devices[i].number == number) {
            m_devices[i].lastSeen = now;
            std::memcpy(m_devices[i].name, name, 21);
            // Master und BPM nur updaten wenn die Info da ist (Status-Paket)
            if (isMaster || bpm > 0) {
                m_devices[i].isMaster = isMaster;
            }
            if (bpm > 0) {
                m_devices[i].bpm = bpm;
            }
            return;
        }
    }
    
    // Neues Device
    if (m_deviceCount < MAX_DEVICES) {
        auto& dev = m_devices[m_deviceCount];
        dev.number = number;
        std::memcpy(dev.name, name, 21);
        dev.isMaster = isMaster;
        dev.bpm = bpm;
        dev.lastSeen = now;
        m_deviceCount++;
        
        LOG_INFO("Pioneer: Device entdeckt → #" + std::to_string(number) + " \"" + std::string(name) + "\"");
    }
}

} // namespace OSC
} // namespace BeatAnalyzer
