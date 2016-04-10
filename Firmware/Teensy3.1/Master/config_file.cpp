#include "config_file.h"

namespace midi_chimes
{

int8_t config_file::parse_note(const String& note)
{
    long noteNum(-1);

    // Try and parse as a number first
    if (is_number(note))
    {
        noteNum = note.toInt();
    }
    // Try and decode a fancy note description
    else
    {
        // It must be of the form <note><sharp?><octave> where octave is -1 to 9
        const char n(note.charAt(0));
        const int sharp((note.charAt(1) == '#') ? 1 : 0);
        const long octave(note.substring(sharp ? 2 : 1).toInt() + 1);

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

bool config_file::is_number(const String& str)
{
    for (size_t i(0); i < str.length(); ++i)
    {
        if (!is_number(str.charAt(i)))
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