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

// most of these are probably useless

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <thread>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>


#include <sys/mman.h>


#include "interface.h"
#include "memdef.h"
#include "iostream"
#include "quaphy.h"

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

memory_s *shmem; // moveme

int main(int argc, char **argv) {

    if (argc==1){
        puts("This process cannot be run standalone!");
        return -1;

    } else {
        int fd = shm_open(argv[1], O_RDWR, S_IRUSR | S_IWUSR);
        if (fd == -1)
        {
            perror("shm_open_err!");
            fprintf(stderr,"Cannot open %s\n", argv[1]);
            return EXIT_FAILURE;
        }
        shmem = (memory_s*)mmap(NULL, sizeof(memory_s), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        if (shmem == MAP_FAILED)
        {
            perror("ps_map_err!");
            return EXIT_FAILURE;
        }
    }
    shmem->childVersion[0] = 0;
    shmem->childVersion[1] = 0;
    shmem->childVersion[2] = 3;


    Interface& interface = Interface::getInstance();
    Quaphy quaphy = Quaphy(); // unused rn

    if (!interface.init()) {
        puts("[KERNEL] Failed to initialize interface!");
        return EXIT_FAILURE;
    }
  
    interface.setOsdLocation(shmem->osd);

    puts("[KERNEL] Successfully initialized interface!");
    std::cout << "[KERNEL] quaphy ver: " << quaphy.ver << std::endl;
    std::cout << "[KERNEL] bf ver: " << interface.getVersion() << std::endl;

    bool run = true;
    auto starttime = std::chrono::system_clock::now();
    auto curtime = std::chrono::system_clock::now();

    while (run) {
        #ifdef dyad
        interface.updateDyad();
        #endif
        interface.run_sched();
        #define fakestate
        #ifdef fakestate
        interface.updateStateFromParams(
          glm::quat{0, 0, 0, 0}, glm::vec3{0, 0, 0}, glm::vec3{0, 0, 0});
        #endif
        #ifdef debugmenu
        interface.set_rc_data(std::array<float, 8>{
          0., 1., 0, -1, 0, 0, 0, 0});  // roll pirch throttle yaw FOR MENU
        #endif

        interface.micros_passed =
          to_us(std::chrono::system_clock::now() - starttime);  

        shmem->micros_passed=interface.micros_passed;
        shmem->nanos_cycle=to_ns(std::chrono::system_clock::now() - curtime);
        curtime = std::chrono::system_clock::now();

        // interface.debugArmFlags(loops);

        // for (int i = 0; i < 8; ++i) {
            // printf("axis %i :  %f\n", i, *(rcpnt + i));
        // }

        usleep(USLEEP_DURATION); // do not eat cpu! potentially target rate instead since this thing can go FAST!
    }
    puts("Process quit (hmmmmmm)");
    return 0;
}
