#pragma once

#include <vector>
#include <memory>
#include <cstring>
#include "audio_types.h"

namespace BeatAnalyzer {
namespace Audio {

/**
 * Circular audio buffer for managing audio frames.
 * Supports multi-channel audio with automatic downmixing.
 */
class AudioBuffer {
public:
    explicit AudioBuffer(int channels, int capacity);
    ~AudioBuffer() = default;
    
    // Non-copyable
    AudioBuffer(const AudioBuffer&) = delete;
    AudioBuffer& operator=(const AudioBuffer&) = delete;
    
    // Write samples to buffer
    void write(const CSAMPLE* samples, int frameCount);
    
    // Read samples from buffer
    void read(CSAMPLE* samples, int frameCount);
    
    // Get mono downmix from all channels
    void readMono(CSAMPLE* monoOut, int frameCount);
    
    // Get specific channel
    void readChannel(int channel, CSAMPLE* out, int frameCount);
    
    // Clear the buffer
    void clear();
    
    // Available samples to read
    int available() const;
    
    // Space available to write
    int writable() const;
    
    // Properties
    int getChannels() const { return m_channels; }
    int getCapacity() const { return m_capacity; }
    
private:
    std::vector<CSAMPLE> m_buffer;
    int m_channels;
    int m_capacity;
    int m_readPos;
    int m_writePos;
    int m_count;
    
    // Helper for circular access
    int advance(int pos, int count) const {
        return (pos + count) % m_capacity;
    }
};

} // namespace Audio
} // namespace BeatAnalyzer
