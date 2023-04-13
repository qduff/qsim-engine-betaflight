#pragma once

#include <cstdint>

#define OSD_WIDTH 30
#define OSD_HEIGHT 16


struct memory_s {

    //  Memory interface version. Quit if major is different.
    //  Minor and patch must NOT break usage
    //      [MAJOR, MINOR, PATCH]
    char schemaVersion[3];

    //  Software version of parent and child
    char parentVersion[3];
    char childVersion[3];

    uint64_t micros_passed;
    uint64_t nanos_cycle;


    //  Capabilties, currently unused.
    char capabilities;

    // Osd Array.
    unsigned char osd[OSD_HEIGHT*OSD_WIDTH];

    // RC data
    float rc[16];

    // may be worth using glm data types idk, although this will do for now to ensure portability.
    float position[3];
    float rotation[3];

    // Initialize (only in host application!)
    memory_s(): schemaVersion{0,2,1} , osd{0}, rc{0.0} {}

} ;

typedef struct memory_s memory_s;