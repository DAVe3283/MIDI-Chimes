#include "config_file.h"

namespace midi_chimes
{

// Setting names
const char note_section[] = "Notes";
const char calibration_section[] = "Calibration";
const char calibration_min_name[] = "Minimum_PWM";

// Local file globals
const char split_tokens[] = " ,";
char slave_key_buffer[] = "Slave13";

// Note name to number map
const char* note_names[128] =
{
    "C-1","C#-1","D-1","D#-1","E-1","F-1","F#-1","G-1","G#-1","A-1","A#-1","B-1",
    "C0", "C#0", "D0", "D#0", "E0", "F0", "F#0", "G0", "G#0", "A0", "A#0", "B0",
    "C1", "C#1", "D1", "D#1", "E1", "F1", "F#1", "G1", "G#1", "A1", "A#1", "B1",
    "C2", "C#2", "D2", "D#2", "E2", "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2",
    "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3",
    "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
    "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
    "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
    "C7", "C#7", "D7", "D#7", "E7", "F7", "F#7", "G7", "G#7", "A7", "A#7", "B7",
    "C8", "C#8", "D8", "D#8", "E8", "F8", "F#8", "G8", "G#8", "A8", "A#8", "B8",
    "C9", "C#9", "D9", "D#9", "E9", "F9", "F#9", "G9",
};

config_file::config_file(const char* filename)
    : ini(filename)
    , calibration_min_default(0.0f)
    , ini_buffer {}
{}

config_file::~config_file()
{}

bool config_file::open()
{
    return ini.open();
}

bool config_file::validate()
{
    return ini.validate(ini_buffer, ini_buffer_len);
}

bool config_file::load_globals()
{
    // Load the minimum calibration PWM %
    if (ini.getValue(calibration_section, calibration_min_name, ini_buffer, ini_buffer_len))
    {
        return parse_percentage(ini_buffer, calibration_min_default);
    }
    else
    {
        return false;
    }
}

bool config_file::get_slave_count(int8_t& slave_count)
{
    slave_count = 0;
    for (int slave = 0; slave < max_slaves; ++slave)
    {
        sprintf(slave_key_buffer + 5, "%d", slave + 1);
        if (ini.getValue(note_section, slave_key_buffer, ini_buffer, ini_buffer_len))
        {
            slave_count++;
        }
        else
        {
            break;
        }
    }

    return slave_count > 0;
}

bool config_file::get_slave_notes(const int8_t& slave_no, int8_t notes[10])
{
    // Get INI setting name
    sprintf(slave_key_buffer + 5, "%d", slave_no + 1);

    // Get INI setting value
    if (!ini.getValue(note_section, slave_key_buffer, ini_buffer, ini_buffer_len))
    {
        return false;
    }
    // ini_buffer should now contain something like "D#5 E5 F5 F#5 G5 NC NC NC NC NC"

    // Parse notes
    for (int note(0); note < 10; ++note)
    {
        // Get next token
        const char* current_note_string = strtok(
            note == 0 ? ini_buffer : NULL,
            split_tokens);

        // Did we get a token?
        if (!current_note_string)
        {
            return false;
        }

        // Process note
        notes[note] = parse_note(current_note_string);
    }

    // Success!
    return true;
}

bool config_file::get_note_calibration(const int8_t& note, float& min, float& max)
{
    const bool got_cal_min(get_note_calibration_min(note, min));
    const bool got_cal_max(get_note_calibration_min(note, min));
    return got_cal_min || got_cal_max;
}
bool config_file::get_note_calibration_min(const int8_t& note, float& calibration)
{
    // Try and lookup by note number
    char buffer[9]; // Worst case: "C#-1_Min"
    sprintf(buffer, "%d_Min", note);
    if (ini.getValue(calibration_section, buffer, ini_buffer, ini_buffer_len))
    {
        // Parse the value
        return parse_percentage(ini_buffer, calibration);
    }

    // Try and lookup by note name
    const char* note_name(lookup_note_name(note));
    if (!note_name)
    {
        return false;
    }
    sprintf(buffer, "%s_Min", note_name);
    if (ini.getValue(calibration_section, buffer, ini_buffer, ini_buffer_len))
    {
        // Parse the value
        return parse_percentage(ini_buffer, calibration);
    }

    // Couldn't find a calibration
    return false;
}
bool config_file::get_note_calibration_max(const int8_t& note, float& calibration)
{
    // Try and lookup by note number
    char buffer[9]; // Worst case: "C#-1_Max"
    sprintf(buffer, "%d_Max", note);
    if (ini.getValue(calibration_section, buffer, ini_buffer, ini_buffer_len))
    {
        // Parse the value
        return parse_percentage(ini_buffer, calibration);
    }

    // Try and lookup by note name
    const char* note_name(lookup_note_name(note));
    if (!note_name)
    {
        return false;
    }
    sprintf(buffer, "%s_Max", note_name);
    if (ini.getValue(calibration_section, buffer, ini_buffer, ini_buffer_len))
    {
        // Parse the value
        return parse_percentage(ini_buffer, calibration);
    }

    // Couldn't find a calibration
    return false;
}

void config_file::print_ini_error_message(Print& display, const uint8_t& errNo, bool eol)
{
    switch (errNo)
    {
    case IniFile::errorNoError:
        display.print("no error");
        break;
    case IniFile::errorFileNotFound:
        display.print("file not found");
        break;
    case IniFile::errorFileNotOpen:
        display.print("file not open");
        break;
    case IniFile::errorBufferTooSmall:
        display.print("buffer too small");
        break;
    case IniFile::errorSeekError:
        display.print("seek error");
        break;
    case IniFile::errorSectionNotFound:
        display.print("section not found");
        break;
    case IniFile::errorKeyNotFound:
        display.print("key not found");
        break;
    case IniFile::errorEndOfFile:
        display.print("end of file");
        break;
    case IniFile::errorUnknownError:
        display.print("unknown error");
        break;
    default:
        display.print("unknown error value");
        break;
    }
    if (eol)
    {
        display.println();
    }
}

void config_file::print_ini_error_message(Print& display, bool eol) const
{
    print_ini_error_message(display, ini.getError(), eol);
}

const char* config_file::lookup_note_name(const int8_t& note)
{
    if (note < 0)
    {
        return nullptr;
    }
    return note_names[note];
}

int8_t config_file::parse_note(const char* note)
{
    int noteNum(-1);

    // Try and parse as a number first
    if (is_number(note))
    {
        noteNum = atoi(note);
    }
    // Try and decode a fancy note description
    else
    {
        // It must be of the form <note><sharp?><octave> where octave is -1 to 9
        const char n(note[0]);
        const int sharp((note[1] == '#') ? 1 : 0);
        const char* octave_str(note + 1 + sharp);
        if (!is_number(octave_str))
        {
            return -1;
        }
        const int octave(atoi(octave_str) + 1);

        // Get base note number
        switch (n)
        {
            case 'A':
            case 'a':
                noteNum = 9 + sharp;
                break;

            case 'B':
            case 'b':
                noteNum = 11;
                if (sharp != 0) { return -1; }
                break;

            case 'C':
            case 'c':
                noteNum = 0 + sharp;
                break;

            case 'D':
            case 'd':
                noteNum = 2 + sharp;
                break;

            case 'E':
            case 'e':
                noteNum = 4;
                if (sharp != 0) { return -1; }
                break;

            case 'F':
            case 'f':
                noteNum = 5 + sharp;
                break;

            case 'G':
            case 'g':
                noteNum = 7 + sharp;
                break;

            default:
                return -1;
        }

        // Add octave
        noteNum += octave * 12;
    }

    // MIDI notes are only 7 bits
    if ((noteNum < 0) || (noteNum >= (1 << 7)))
    {
        return -1;
    }
    else
    {
        return static_cast<int8_t>(noteNum);
    }
}

bool config_file::parse_percentage(char* str, float& percentage)
{
    // Validate string
    size_t len(0);
    while (str[len] != 0)
    {
        // Remove % symbol
        if (str[len] == '%')
        {
            str[len++] = 0;
            break;
        }

        // Fail on non-number characters
        if (!is_number(str[len]) &&
            (str[len] != '.'))
        {
            return false;
        }
        len++;
    }

    // Parse string
    percentage = atof(str) / 100.0f;
    return true;
}

bool config_file::is_number(const char* str)
{
    if ((str[0] == '+') ||
        (str[0] == '-'))
    {
        str++;
    }
    const size_t len(strlen(str));
    for (size_t i(0); i < len; ++i)
    {
        if (!is_number(str[i]))
        {
            return false;
        }
    }
    return true;
}
bool config_file::is_number(const char& c)
{
    return ((c >= '0') && (c <= '9'));
}

} // namespace midi_chimes