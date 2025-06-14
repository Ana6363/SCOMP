#include <stdio.h>
#include <math.h>
#include "utils.h"

int count_lines(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return -1;
    }

    int count = 0;
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        count++;
    }

    fclose(file);
    return count;
}
