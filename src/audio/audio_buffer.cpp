#include "audio/audio_buffer.h"
#include <algorithm>
#include <cstring>

namespace BeatAnalyzer {
namespace Audio {

AudioBuffer::AudioBuffer(int channels, int capacity)
    : m_channels(channels),
      m_capacity(capacity),
      m_readPos(0),
      m_writePos(0),
      m_count(0) {
    m_buffer.resize(channels * capacity, 0.0f);
}

void AudioBuffer::write(const CSAMPLE* samples, int frameCount) {
    for (int i = 0; i < frameCount * m_channels; ++i) {
        m_buffer[m_writePos] = samples[i];
        m_writePos = advance(m_writePos, 1);
        m_count++;
        if (m_count > m_capacity) {
            m_count = m_capacity;
            m_readPos = advance(m_readPos, 1);
        }
    }
}

void AudioBuffer::read(CSAMPLE* samples, int frameCount) {
    int samplesToRead = std::min(frameCount * m_channels, m_count);
    for (int i = 0; i < samplesToRead; ++i) {
        samples[i] = m_buffer[m_readPos];
        m_readPos = advance(m_readPos, 1);
    }
    m_count -= samplesToRead / m_channels;
}

void AudioBuffer::readMono(CSAMPLE* monoOut, int frameCount) {
    int samplesToRead = std::min(frameCount, m_count / m_channels);
    
    for (int frame = 0; frame < samplesToRead; ++frame) {
        CSAMPLE sum = 0.0f;
        for (int ch = 0; ch < m_channels; ++ch) {
            sum += m_buffer[m_readPos];
            m_readPos = advance(m_readPos, 1);
        }
        monoOut[frame] = sum / m_channels;
    }
    m_count -= samplesToRead * m_channels;
}

void AudioBuffer::readChannel(int channel, CSAMPLE* out, int frameCount) {
    if (channel >= m_channels) return;
    
    int pos = m_readPos + channel;
    for (int i = 0; i < frameCount; ++i) {
        out[i] = m_buffer[pos % (m_capacity * m_channels)];
        pos += m_channels;
    }
}

void AudioBuffer::clear() {
    std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    m_readPos = 0;
    m_writePos = 0;
    m_count = 0;
}

int AudioBuffer::available() const {
    return m_count / m_channels;
}

int AudioBuffer::writable() const {
    return (m_capacity - m_count) / m_channels;
}

} // namespace Audio
} // namespace BeatAnalyzer
