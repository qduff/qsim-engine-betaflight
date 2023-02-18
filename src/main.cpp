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

#include <boost/asio.hpp>
#include <boost/interprocess/creation_tags.hpp>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <thread>

#include "interface.h"
#include "iostream"
#include "packets.hpp"
#include "quaphy.h"

using hr_clock = std::chrono::high_resolution_clock;

template <typename R, typename P>
auto to_us(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::microseconds>(t).count();
}

template <typename R, typename P>
auto to_ms(std::chrono::duration<R, P> t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(t).count();
}

typedef std::array<int16_t, 4> motor_pwms_t;

int main() {
    managed_shared_memory shm{open_only, "shrdx"};

    Interface& interface = Interface::getInstance();
    Quaphy quaphy = Quaphy();

    if (!interface.init()) {
        puts("[KERNEL] Failed to initialize interface!");
        return EXIT_FAILURE;
    }
    // uint8_t(*osdScreen)[VIDEO_LINES][CHARS_PER_LINE];
    // osdScreen =
    //   shm.find<uint8_t[VIDEO_LINES][CHARS_PER_LINE]>("osdArray").first;

    unsigned char(*osdpnt) = shm.find<uint8_t>("osdpnt").first;
    float(*rcpnt) = shm.find<float>("rcpnt").first;

    printf("%p\n", osdpnt);
    interface.setOsdLocation(osdpnt);

    puts("[KERNEL] Successfully initialized interface!");
    std::cout << "[KERNEL] quaphy ver: " << quaphy.ver << std::endl;
    std::cout << "[KERNEL] bf ver: " << interface.getVersion() << std::endl;

    bool run = true;
    auto start = std::chrono::system_clock::now();

    // unsigned long long int loops;
    while (run) {
        // interface.updateDyad();
        interface.run_sched();
        interface.updateStateFromParams(
          glm::quat{0, 0, 0, 0}, glm::vec3{0, 0, 0}, glm::vec3{0, 0, 0});
        // interface.set_rc_data(std::array<float, 8>{
        //   0., 1., 0, -1, 0, 0, 0, 0});  // roll pirch throttle yaw FOR MENU
        // interface.set_rc_data(std::array<float, 8>{
        //   0., 0., -1, 0., 0, 0, 0, 0});  // roll pirch throttle yaw
        // interface.set_rc_data(
        //   std::array<float, 8>{0., 0., 1, 0., 0, 0, 0, 0});  // throttle UP

        interface.micros_passed =
          to_us(std::chrono::system_clock::now() - start);  // stupid!
                                                            // ++loops;
        // if (loops == 10000) {
        //     return 0;
        //     puts("1mil done");
        // }
        // interface.debugArmFlags(loops);
        // printf("%i\n", loops);
        interface.set_rc_data_from_pointer(rcpnt);

        for (int i = 0; i < 8; ++i) {
            // printf("axis %i :  %f\n", i, *(rcpnt + i));
        }
    }

    std::cout << ("[KERNEL] NO TASKS, QUITTING\n");

    std::cout << ("[KERNEL] Stopped host process\n");

    return 0;
}
