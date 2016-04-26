#include "slave_status.h"

namespace midi_chimes
{

slave_status::slave_status()
    : _valid(false)
    , _pwm_bits(0)
    , _num_channels(0)
    , _num_connected_channels(0)
    , ps_mV(0)
    , channel_state_buffer {}
    , strikes_remaining_buffer {}
    , channel_voltage_buffer {}
{}

slave_status::slave_status(const uint8_t* buffer, const size_t& buffer_len)
    : _valid(false)
    , _pwm_bits(0)
    , _num_channels(0)
    , _num_connected_channels(0)
    , ps_mV(0)
    , channel_state_buffer {}
    , strikes_remaining_buffer {}
    , channel_voltage_buffer {}
{
    load_slave_status(buffer, buffer_len);
}

slave_status::~slave_status()
{}

void slave_status::load_slave_status(const uint8_t* buffer, const size_t& buffer_len)
{
    // Verify we got at least a header
    const size_t header_size(6);
    if (buffer_len < header_size)
    {
        return;
    }

    // Determine message version
    const uint8_t version(buffer[0]);

    // Handle version 0
    if (version == 0)
    {
        _valid = true;

        // Parse header
        _pwm_bits               = buffer[1];
        _num_channels           = buffer[2];
        _num_connected_channels = buffer[3];
        memcpy(&ps_mV, buffer + 4, sizeof(ps_mV));

        // Verify we have enough storage
        if (_num_channels > max_channels)
        {
            _valid = false;
            _num_channels = max_channels;
        }

        // Read per-channel data
        for (size_t channel(0); channel < _num_channels; ++channel)
        {
            // Sizes of things (should be optimized out)
            const size_t channel_state_size(sizeof(*channel_state_buffer));
            const size_t strikes_remaining_size(sizeof(*strikes_remaining_buffer));
            const size_t channel_voltage_size(sizeof(*channel_voltage_buffer));

            // Generate addresses in the buffer (should be optimized nicely)
            const size_t channel_state_addr(    header_size +                                                                 + (channel * channel_state_size));
            const size_t strikes_remaining_addr(header_size + (_num_channels * (channel_state_size))                          + (channel * strikes_remaining_size));
            const size_t channel_voltage_addr(  header_size + (_num_channels * (channel_state_size + strikes_remaining_size)) + (channel * channel_voltage_size));

            // Channel state
            channel_state_buffer[channel] = static_cast<channel_state_t>(buffer[channel_state_addr]);

            // Strikes remaining
            strikes_remaining_buffer[channel] = buffer[strikes_remaining_addr];

            // Last measured voltage
            memcpy(
                channel_voltage_buffer + (channel_voltage_size * channel),
                buffer + channel_voltage_addr,
                channel_voltage_size);
        }
    }
    // Unknown version
    else
    {
        return;
    }
}

bool slave_status::valid() const
{
    return _valid;
}

uint8_t slave_status::pwm_bits() const
{
    return _pwm_bits;
}

uint8_t slave_status::num_channels() const
{
    return _num_channels;
}

uint8_t slave_status::num_connected_channels() const
{
    return _num_connected_channels;
}

float slave_status::ps_voltage() const
{
    return static_cast<float>(ps_mV) / 1000.0f;
}

channel_state_t slave_status::channel_state(const uint8_t& channel) const
{
    // Sanity check
    if (!_valid || channel > _num_channels)
    {
        return channel_disconnected;
    }

    // Get the channel state
    return channel_state_buffer[channel];
}

int8_t slave_status::strikes_remaining(const uint8_t& channel) const
{
    // Sanity check
    if (!_valid || channel > _num_channels)
    {
        return 0;
    }

    // Get the strikes remaining
    return strikes_remaining_buffer[channel];
}

float slave_status::channel_voltage(const uint8_t& channel) const
{
    // Sanity check
    if (!_valid || channel > _num_channels)
    {
        return 0;
    }

    // Get the voltage of the channel
    return static_cast<float>(channel_voltage_buffer[channel]) / 1000.0f;
}

} // namespace midi_chimes