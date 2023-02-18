#include "interface.h"

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

#include "packets.hpp"

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
}  // namespace bf

#define USE_QUAT_ORIENTATION

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

Interface& Interface::getInstance() {
    static Interface interface;
    return interface;
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
    uint16_t rcData[8];
    for (int i = 0; i < 8; i++) {
        rcData[i] = uint16_t(1500 + pnt[i] * 500);
        // printf("setting %d %d", i, rcData[i]);
    }
    bf::rxMspFrameReceive(&rcData[0], 8);
}

const char* Interface::getVersion() {
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
// apparently the fakeAccset can go here
#endif
}

// std::vector<std::vector<int>> getOsdVec() {
//     for (int y = 0; y < VIDEO_LINES; y++) {
//         for (int x = 0; x < CHARS_PER_LINE; x++) {
//             update.osd.value[y * CHARS_PER_LINE + x] = bf::osdScreen[y][x];
//         }
//     }
// }
// bool Simulator::step(float delta, bool crashed) {
//     const auto deltaMicros = int(delta * 1e6);
//     total_delta += deltaMicros;

//     ////const auto last = hr_clock::now();

//     dyad_update();

// // auto dyad_time = hr_clock::now() - last;
// // long long dyad_time_i = to_us(dyad_time);

// // update rc at 100Hz, otherwise rx loss gets reported:
//
// set rc data separwelty
// set_rc_data(state.rcData.value);

// for (auto k = 0u; total_delta - DELTA >= 0; k++) {
//     total_delta -= DELTA;
//     micros_passed += DELTA;
//     const float dt = DELTA / 1e6f;

//     set_accelerometer(acceleration);

//     if (sleep_timer > 0) {
//         sleep_timer -= DELTA;
//         sleep_timer = std::max(int64_t(0), sleep_timer);
//     } else {
//         bf::scheduler();
//     }

//     if (crashed) continue;

// float motorsTorque = calculate_motors(dt, state, motorsState);

// acceleration = calculate_physics(dt, state, motorsState,
// motorsTorque);
// }

// if (micros_passed - last_osd_time > OSD_UPDATE_TIME) {
//     last_osd_time = micros_passed;
//     StateOsdUpdatePacket update;
//     update.angularVelocity.value = state.angularVelocity.value;
//     update.linearVelocity.value = state.linearVelocity.value;
//     for (int y = 0; y < VIDEO_LINES; y++) {
//         for (int x = 0; x < CHARS_PER_LINE; x++) {
//             update.osd.value[y * CHARS_PER_LINE + x] =
//             bf::osdScreen[y][x];
//         }
//     }
//     send(send_socket, update);
// } else {
//     StateUpdatePacket update;
//     update.angularVelocity.value = state.angularVelocity.value;
//     update.linearVelocity.value = state.linearVelocity.value;
//     send(send_socket, update);
// }

// return true;
// }

void Interface::debugArmFlags(unsigned long long int loops) {
    int flags = bf::getArmingDisableFlags();
    while (flags) {
        const int bitpos = ffs(flags) - 1;
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
    dyad_update();  // dyad is NOT necceeary
}

std::array<int16_t, 4> Interface::get_motor_pwms() {
    std::array<int16_t, 4> myarray;
    std::copy(bf::motorsPwm, bf::motorsPwm + 4, myarray.begin());
    return myarray;
}

void Interface::set_rc_data(std::array<float, 8> data) {
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

// void Simulator::set_baro(int pressure) {
//     bf::fakeBaroSet(pkt->pressure, 2500);
//     // temperature in 0.01 C = 25 deg
// }

// void Simulator::set_gps(const glm::vec3& position, const glm::vec3& velocity)
// {
//     const auto cosLon0 = 0.63141842418f;

//     // set gps:
//     static int64_t last_gps_millis = 0;
//     int64_t millis = micros_passed / 1000;

//     if (millis - last_gps_millis > 100) {
//         bf::EnableState(bf::GPS_FIX);
//         bf::gpsSol.numSat = 10;
//         bf::gpsSol.llh.lat =
//           int32_t(-position[2] * 100 / LATLON_TO_CM) + 508445910;
//         bf::gpsSol.llh.lon =
//           int32_t(position[0] * 100 / (cosLon0 * LATLON_TO_CM)) + 43551050;
//         bf::gpsSol.llh.altCm = int32_t(position[1] * 100);
//         bf::gpsSol.groundSpeed = uint16_t(glm::length(velocity) * 100);
//         bf::GPS_update |= bf::GPS_MSP_UPDATE;

//         last_gps_millis = millis;
//     }
// }

//
//
Interface::~Interface() {
    dyad_shutdown();
}
// BF Funcs
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
    //    printf("[INTERFACE] uS:%i ", usec);
    // Simulator::getInstance().sleep_timer = usec;
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

void delayMicroseconds(uint32_t usec) {
    printf("uSleep;%i", usec);
    microsleep(usec);
}

void delay(uint32_t ms) {
    microsleep(ms * 1000);
}
}