#pragma once

#include <array>
#include <cstdint>
#include <glm/ext/vector_float3.hpp>
#include <glm/gtc/quaternion.hpp>

typedef struct stateRecievePkt {
    bool acknowledged;
    glm::vec3 position;
    glm::quat orientation;

    glm::vec3 ang_velocity;
    glm::vec3 lin_velocity;
    std::array<uint16_t, 8> rc;
} stateRecievePkt;

typedef std::array<int16_t, 4> motor_pwms_t;

struct motorSendPkt {
    bool acknowledged;
    motor_pwms_t pwms;
};
