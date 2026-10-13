#pragma once

#include <dolphin/types.h>

void OSPanic(const char* file, int line, const char* msg, ...);

#define MTX_ASSERT(cond) ((cond) ? (void)0 : OSPanic(__FILE__, __LINE__, "Failed assertion: %s", #cond))
