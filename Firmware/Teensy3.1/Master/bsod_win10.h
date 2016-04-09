// Draws a Windows 10 BSOD

// Header guard
#ifndef _BSOD_WIN10_H_
#define _BSOD_WIN10_H_

#include <ILI9341_t3.h>

// Actually draw the BSOD on the specified display
void draw_BSOD(ILI9341_t3& display);

// Halt the system
void halt_system();

#endif // ifndef _BSOD_WIN10_H_
