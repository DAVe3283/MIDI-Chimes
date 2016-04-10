// Configuration File Handling

// Header guard
#ifndef _CONFIG_FILE_H_
#define _CONFIG_FILE_H_

#include <stdint.h>
#include <IniFile.h>

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
    config_file(const char* filename);
    ~config_file();

    // Open the INI file
    bool open();

    // Validate the INI file
    bool validate();

    // Get the number of slave configs in the INI file
    bool get_slave_count(int8_t& slave_count);

    // Get the notes mapped to the given slave
    bool get_slave_notes(const int8_t& slave_no, int8_t notes[10]);

    // Print the INI error message from the errNo provided
    static void print_ini_error_message(Print& display, const uint8_t& errNo, bool eol = true);

    // Print the error message for the current error number
    void print_ini_error_message(Print& display, bool eol = true) const;

    // Parse a string ("61" or "C4#") into the MIDI note number, -1 for invalid.
    static int8_t parse_note(const char* note);

private:
    // Holds the INI file used to store config settings
    IniFile ini;

    // Buffer for reading INI file
    const static size_t ini_buffer_len = 84;
    char ini_buffer[ini_buffer_len];

    // Maximum number of slaves
    const static int8_t max_slaves = 13;
    // MIDI allows 128 notes, and each slave can handle 10, so we need up to 13

    // Checks if the provided value is a number
    static bool is_number(const char* str);
    static bool is_number(const char& c);
};

} // namespace midi_chimes
#endif // ifndef _CONFIG_FILE_H_
