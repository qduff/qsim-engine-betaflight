/*
Copyright (C) 2023 Quentin Duff <hidden>
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <thread>
#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>  // usleep
#elif defined _WIN32 || defined __CYGWIN__
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#endif

#include <chrono>

#include "interface.hpp"
#include <iostream>
#include "memdef.h"
#include "../lib/qsim-physics/physics.h"

#define USLEEP_DURATION 50

template <typename R, typename P>
auto to_ns(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
}

template <typename R, typename P>
auto to_us(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
}

template <typename R, typename P>
auto to_ms(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
}

int main(int argc, char **argv) {
    memory_s *shmem = NULL;
#ifdef EMPTY_ALLOC
    shmem = (memory_s *)malloc(sizeof(memory_s));
#else
    if (argc <= 1) {
        puts("This process cannot be run standalone!");
        return -1;
    } else {
#ifdef __linux__
        int fd = shm_open(argv[1], O_RDWR, S_IRUSR | S_IWUSR);
        if (fd == -1) {
            perror("shm_open_err!");
            fprintf(stderr, "Cannot open %s\n", argv[1]);
            return EXIT_FAILURE;
        }
        shmem = (memory_s *)mmap(
          NULL, sizeof(memory_s), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (shmem == MAP_FAILED) {
            perror("ps_map_err!");
            return EXIT_FAILURE;
        }
#elif defined _WIN32 || defined __CYGWIN__
        HANDLE hMapFile;
        hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, argv[1]);
        if (hMapFile == NULL) {
            printf("Could not open file mapping object (%d).\n",
                   GetLastError());
            return 1;
        }
        shmem = (memory_s *)MapViewOfFile(
          hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(memory_s));
        if (shmem == NULL) {
            printf("Could not map view of file (%d).\n", GetLastError());
            CloseHandle(hMapFile);
            return 1;
        }
#else
#error Unsupported platform!
#endif
#endif
}
shmem->childVersion[0] = 0;
shmem->childVersion[1] = 0;
shmem->childVersion[2] = 3;

Interface &bf_interface = Interface::getInstance();
Physics quaphy = Physics();  // unused rn

if (!bf_interface.init()) {
    puts("[KERNEL] Failed to initialize bf_interface!");
    return EXIT_FAILURE;
}

bf_interface.setOsdLocation(shmem->osd);

puts("[KERNEL] Successfully initialized bf_interface!");
// std::cout << "[KERNEL] quaphy ver: " << quaphy.ver << std::endl;
std::cout << "[KERNEL] bf ver: " << bf_interface.getVersion() << std::endl;

bool run = true;
auto starttime = std::chrono::system_clock::now();
auto curtime = std::chrono::system_clock::now();

while (run) {
#ifdef dyad
    bf_interface.updateDyad();
#endif
    bf_interface.run_sched();
#define fakestate
#ifdef fakestate
    bf_interface.updateStateFromParams(
      glm::quat{0, 0, 0, 0}, glm::vec3{0, 0, 0}, glm::vec3{0, 0, 0});
#endif
#ifdef debugmenu
    bf_interface.set_rc_data(std::array<float, 8>{
      0., 1., 0, -1, 0, 0, 0, 0});  // roll pirch throttle yaw FOR MENU
#endif
    bf_interface.set_rc_data_from_pointer(shmem->rc);

    bf_interface.micros_passed =
      to_us(std::chrono::system_clock::now() - starttime);

    shmem->micros_passed = bf_interface.micros_passed;
    shmem->nanos_cycle = to_ns(std::chrono::system_clock::now() - curtime);
    curtime = std::chrono::system_clock::now();

    // debug stuff
    shmem->position[0] = std::sin(bf_interface.micros_passed / 1000000.0f);
    shmem->position[1] = std::cos(bf_interface.micros_passed / 1000000.0f);
    shmem->position[2] = 0;

    shmem->rotation[0] = 0;
    shmem->rotation[1] = 0;
    shmem->rotation[2] = 0;

    auto pwms = bf_interface.get_motor_pwms();
    for (int i = 0; i<4; i++){
        // printf("motor %d, %u\n", i, pwms.at(i));
    }
    // bf_interface.debugArmFlags();

    // for (int i = 0; i < 8; ++i) {
    // printf("axis %i :  %f\n", i, *(rcpnt + i));
    // }
    #ifndef __CYGWIN__
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    #endif
    // do not eat cpu! potentially target rate
    // instead since this thing can go FAST!
    // NB: not an ideal solution!
}
puts("Process quit (hmmmmmm)");
return 0;
}
