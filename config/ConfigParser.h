#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "SystemConfig.h"
// KEY=VALUE parser. Unknown keys and invalid values reject the entire load.
// Coordinates use driver pulses; only MICROSTEP_RATIO=1 is accepted.
static const int CONFIG_MAX_LINE_LENGTH=128;
int ConfigParser_Load(const char* filepath,SystemConfig& cfg); // 0 success, -1 failure
void ConfigParser_Print(const SystemConfig& cfg);
