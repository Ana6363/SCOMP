#ifndef DRONE_H
#define DRONE_H

#include <stdbool.h>
#include <sys/types.h>
#include "shared_memory.h"

void drone_process(int drone_index, const char* script_file);

#endif
