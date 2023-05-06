#include "interface.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <thread>
#include <vector>


extern "C" {
#include "dyad.h"
}

namespace bf {
extern "C" {

#include "common/maths.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_fake.h"
#include "fc/init.h"
#include "fc/runtime_config.h"
#include "fc/tasks.h"
#include "flight/imu.h"
#include "io/displayport_fake.h"
#include "io/gps.h"
#include "rx/msp.h"
#include "scheduler/scheduler.h"
#include "sensors/sensors.h"
#include "src/target.h"

#undef ENABLE_STATE

// BEFAFLIGHT:src/main/fc/runtime_config.h 121
void EnableState(stateFlags_t mask) {
    stateFlags |= mask;
}
}
}



#define USE_QUAT_ORIENTATION

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

Interface& Interface::getInstance() {
    static Interface bf_interface;
    return bf_interface;
}

bool Interface::init() {
    dyad_init();
    dyad_setUpdateTimeout(0.001);

    std::cout << ("[INTERFACE] Initializing betaflight\n");
    bf::init();

    return true;
}

void Interface::setOsdLocation(uint8_t* pnt) {
    bf::osdPntr = pnt;
}

void Interface::set_rc_data_from_pointer(float* pnt) {
    // printf("rcpnt %p\n", pnt);

    uint16_t rcData[8];
    for (int i = 0; i < 8; i++) {
        rcData[i] = uint16_t(1500 + pnt[i] * 500);
        // printf("setting %d %d", i, rcData[i]);
    }
    bf::rxMspFrameReceive(&rcData[0], 8);
}

const char* Interface::getVersion() { // yeah idk
    return FC_VERSION_STRING;
    return bf::shortGitRevision;
}

void Interface::updateStateFromParams(glm::quat orientation,
                                      glm::vec3 ang_velocity,
                                      glm::vec3 lin_acceleration) {
    // appartently accelerometer is not not necessary if using setattitudequat -
    // look into
    int16_t x, y, z;
    x = bf::constrain(lin_acceleration.x * ACC_SCALE, -32767, 32767);
    y = bf::constrain(lin_acceleration.y * ACC_SCALE, -32767, 32767);
    z = bf::constrain(lin_acceleration.z * ACC_SCALE, -32767, 32767);
    bf::fakeAccSet(bf::fakeAccDev, x, y, y);

    x = bf::constrain(ang_velocity.x * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = bf::constrain(-ang_velocity.y * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = bf::constrain(ang_velocity.z * GYRO_SCALE * RAD2DEG, -32767, 32767);
    bf::fakeGyroSet(bf::fakeGyroDev, x, y, z);

#if !defined(USE_IMU_CALC)  // we have undefined it above since we wish to
                            // tell it the true attitude.
#if defined(USE_QUAT_ORIENTATION)
    bf::imuSetAttitudeQuat(
      orientation[0], orientation[1], orientation[2], orientation[3]);
#else
#error NON QUAT ORIENTATION SET??? bruh.
#endif
#else
// apparently the fakeAccset can go here..?
#endif
}

void Interface::debugArmFlags() {
    int flags = bf::getArmingDisableFlags();
    while (flags) {
        #ifdef __MINGW32__
        const int bitpos = __builtin_ffs(flags) - 1;
        #else
        const int bitpos = bf::ffs(flags) - 1;
        #endif
        flags &= ~(1 << bitpos);
        printf(" %s", bf::armingDisableFlagNames[bitpos]);
    }
    if (flags) {
        puts(" ");
    }
}

// will WAIT for state
bool Interface::checkNewInput() {
    return false;
}

void Interface::run_sched() {
    bf::scheduler();
}

void Interface::updateDyad() {
    dyad_update(); 
    // dyad is NOT necceeary if no TCP UART sock. leave for now...
}

std::array<int16_t, 4> Interface::get_motor_pwms() {
    std::array<int16_t, 4> myarray;
    std::copy(bf::motorsPwm, bf::motorsPwm + 4, myarray.begin());
    return myarray;
}

void Interface::set_rc_data(std::array<float, 8> data) {
    // ideally use shmem->rc instead....
    uint16_t rcData[8];
    for (int i = 0; i < 8; i++) {
        rcData[i] = uint16_t(1500 + data[i] * 500);
    }
    bf::rxMspFrameReceive(&rcData[0], 8);
}
void Interface::set_rotation(const glm::mat3& rotation) {
    glm::quat qrotation = glm::quat_cast(rotation);
    bf::imuSetAttitudeQuat(
      qrotation[3], -qrotation[2], qrotation[0], -qrotation[1]);
}


Interface::~Interface() {
    dyad_shutdown();
    //! other shutdown stuff. 
}

// betaflight overrides
extern "C" {
void systemInit(void) {
    int ret;

    printf("[INTERFACE:extern] [system] Init...\n");

    bf::SystemCoreClock = 500 * 1000000;  // fake 500MHz
    bf::FLASH_Unlock();
}

void systemReset(void) {
    printf("[INTERFACE:extern] [system] Reset!\n");
    exit(0);
    // execve(2) mayhaps? will likely be PITA on winapi
    // restart from parent may not be ideal as context on exit needed (exit code?)
}

void systemResetToBootloader(void) {
    printf("[INTERFACE:extern] [system] ResetToBootloader!\n");
    exit(1);
}

uint32_t micros(void) {
    // polled by scheduler!
    return Interface::getInstance().micros_passed & 0xFFFFFFFF; 
}

uint32_t millis(void) {
    return (Interface::getInstance().micros_passed / 1000) & 0xFFFFFFFF;
}

void microsleep(uint32_t usec) {
    // _should_ work...?
    //    printf("[INTERFACE] uS:%i ", usec);
    // Simulator::getInstance().sleep_timer = usec;
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

void delayMicroseconds(uint32_t usec) {
    microsleep(usec);
}

void delay(uint32_t ms) {
    microsleep(ms * 1000);
}
}