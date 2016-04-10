// Configuration File Handling

// Header guard
#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_

#include <Arduino.h>
#include <stdint.h>

namespace midi_chimes
{

// Maps notes to the respective slave
struct slave_note_map
{
    // I2C address of the slave
    uint8_t slave_address;

    // Channel (port) on the slave for this note
    uint8_t channel;
};

class config_file
{
public:
    config_file();
    ~config_file();

    // Parse a string ("61" or "C4#") into the MIDI note number, -1 for invalid.
    static int8_t parse_note(const String& note);

private:
    // Checks if the provided value is a number
    static bool is_number(const String& str);
    static bool is_number(const char& c);
};

} // namespace midi_chimes
#endif // ifndef _CONFIG_FILE_H_
