#pragma once


#define ITF_CONSOLE 0
#define ITF_DCSD 1
#define ITF_JTAG 2

void clear_screen();
void serprint(const char* format, ...);