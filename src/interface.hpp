#pragma once

#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>  // glm::vec3


typedef std::array<int16_t, 4> motor_pwms_t;

class Interface {
   private:
   public:
     // tidy this nonsense. 
    static Interface& getInstance();
    bool init();
    void setOsdLocation(uint8_t* pnt);
    void set_rc_data_from_pointer(float* pnt);

    bool step();
    void set_rc_data(std::array<float, 8> data);
    void set_gyro(const glm::vec3& ang_velocity);
    void set_accelerometer(const glm::vec3& lin_acceleration);
    void set_rotation(const glm::mat3& rotation);
    bool checkNewInput();
    void set_gps(const glm::vec3& position, const glm::vec3& velocity);
    void run_sched();
    void updateDyad();
    void updateStateFromParams(glm::quat orientation,
                               glm::vec3 ang_velocity,
                               glm::vec3 lin_acceleration);
    void debugArmFlags();
    std::array<int16_t, 4> get_motor_pwms();
    ~Interface();

    bool step(float delta, bool crashed);

    glm::vec3 acceleration = {0, 0, 0};

    unsigned long long int simsteps = 0u;

    uint64_t micros_passed = 0;  // fakeish but will suffice
    const char* getVersion();
};