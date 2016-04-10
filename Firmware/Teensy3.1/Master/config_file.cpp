#include "config_file.h"

namespace midi_chimes
{

// Local file globals
const char split_tokens[] = " ,";
char slave_key_buffer[] = "Slave12";

config_file::config_file(const char* filename)
    : ini(filename)
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

bool config_file::get_slave_count(int8_t& slave_count)
{
    slave_count = 0;
    for (int slave = 0; slave < max_slaves; ++slave)
    {
        sprintf(slave_key_buffer + 5, "%d", slave);
        if (ini.getValue("Slaves", slave_key_buffer, ini_buffer, ini_buffer_len))
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
    sprintf(slave_key_buffer + 5, "%d", slave_no);

    // Get INI setting value
    if (!ini.getValue("Slaves", slave_key_buffer, ini_buffer, ini_buffer_len))
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

bool config_file::is_number(const char* str)
{
    if ((str[0] == '+') ||
        (str[0] == '-'))
    {
        str++;
    }
    size_t len(strlen(str));
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