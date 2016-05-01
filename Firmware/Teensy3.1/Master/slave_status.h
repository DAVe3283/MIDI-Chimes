// Slave board status object

// Header guard
#ifndef _SLAVE_STATUS_H
#define _SLAVE_STATUS_H

#include <Arduino.h>
#include <stdint.h>

namespace midi_chimes
{

// Channel states of slave channels
enum channel_state_t : uint8_t
{
  channel_working = 0,
  channel_disconnected = 1,
  channel_failed_short = 2,
  channel_failed_open = 3,
};

class slave_status
{
public:
    slave_status();
    slave_status(const uint8_t* buffer, const size_t& buffer_len);
    ~slave_status();

    // Load slave state into the class
    void load_slave_status(const uint8_t* buffer, const size_t& buffer_len);

    // Is the slave status valid
    bool valid() const;

    // How many bits of PWM resolution does the slave expect
    uint8_t pwm_bits() const;

    // Get the number of channels attached to the slave
    uint8_t num_channels() const;

    // Gets the number of connected channels
    uint8_t num_connected_channels() const;

    // Gets power supply voltage
    float ps_voltage() const;

    // Gets the channel state for the specified channel
    channel_state_t channel_state(const uint8_t& channel) const;

    // Get strikes remaining for the specified channel
    int8_t strikes_remaining(const uint8_t& channel) const;

    // Gets the last measured voltage on the specified channel
    float channel_voltage(const uint8_t& channel) const;

    // Prints a channel state to the specified output
    static void print_channel_state(Print& out, const channel_state_t& state);

private:
    // Internal validation status
    bool _valid;

    // Header values
    uint8_t _pwm_bits;
    uint8_t _num_channels;
    uint8_t _num_connected_channels;
    uint16_t ps_mV;

    // Max channels per slave (can be safely increased, but will waste RAM if it doesn't match the slaves)
    const static size_t max_channels = 10;

    // Holds channel statuses
    channel_state_t channel_state_buffer[max_channels];
    int8_t strikes_remaining_buffer[max_channels];
    uint16_t channel_voltage_buffer[max_channels];
};

} // namespace midi_chimes
#endif // ifndef _SLAVE_STATUS_H
