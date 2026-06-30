#pragma once

#include <AP_MXECAN/AP_MXECAN_config.h>

#if AP_MXECAN_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#define AP_MXECAN_USE_EVENTS (defined(CH_CFG_USE_EVENTS) && CH_CFG_USE_EVENTS == TRUE)

#if AP_MXECAN_USE_EVENTS
#include <ch.h>
#endif

#define DEFAULT_NUM_POLES 14

#define MXECAN_MAX_NUM_ESCS 8

#define MXECAN_DLC_SIZE 8

typedef enum FaultCode {
    NORMAL              = 0,
    DRIVER_FAULT        = 1,
    OVERCURRENT         = 5,
    OVERVOLTAGE         = 6,
    UNDERVOLTAGE        = 7,
    OVERHEAT            = 8,
    MOTOR2_OVERSPEED    = 11,
    MOTOR1_OVERSPEED    = 12,
    MOTOR2_OVERLOAD     = 13,
    MOTOR1_OVERLOAD     = 14,
    MOTOR2_PHASE_LOSS   = 15,
    MOTOR1_PHASE_LOSS   = 16,
    MOTOR2_BRAKING      = 17,
    MOTOR1_BRAKING      = 18,
    MOTOR2_ENCODER      = 19,
    MOTOR1_ENCODER      = 20,
    MOTOR2_OVERHEAT     = 21,
    MOTOR1_OVERHEAT     = 22,
    MOTOR2_HALL_FAULT   = 23,
    MOTOR1_HALL_FAULT   = 24,
    MOTOR2_LOCKED_ROTOR = 25,
    MOTOR1_LOCKED_ROTOR = 26,
    UART_FAULT          = 27,
    RS485_FAULT         = 28,
    CAN_FAULT           = 29,
    JOYSTICK_FAULT      = 30,
    SWITCH_FAULT        = 31,
} FaultCode_t;

class AP_MXECAN_Driver : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    
    AP_MXECAN_Driver();

    // called from SRV_Channels
    void update(const uint8_t num_poles);

private:
    union mxecan_frame_t {
        struct PACKED {
            uint8_t fault_code;
            uint8_t driver_temperature;
            int16_t axis2_speed;
            int16_t axis1_speed;
            uint16_t power_voltage;
        };
        uint8_t data[8];
    };

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    bool send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data);
    bool send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data = nullptr, const uint8_t data_len = 0);

    void loop();

    struct {
        HAL_Semaphore sem;
        bool is_new;
        uint32_t last_new_ms;
        uint16_t pwm[NUM_SERVO_CHANNELS];
#if AP_MXECAN_USE_EVENTS
        thread_t *thread_ctx;
#endif
    } _output;

#if HAL_WITH_ESC_TELEM
    struct {
        uint8_t num_poles;
        uint32_t timer_ms;
    } _telemetry;
#endif
    
    static const uint32_t ESC_NODE_ID = 0x1801E600; // aka driver RX ID
    static const uint32_t AUTOPILOT_NODE_ID = 0x1801E001; // aka driver TX ID

    static const uint32_t TELEMETRY_INTERVAL_MS = 100;

};

class AP_MXECAN {
public:
    AP_MXECAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_MXECAN);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    static AP_MXECAN *get_singleton() { return _singleton; }

private:
    static AP_MXECAN *_singleton;

    AP_Int8 _num_poles;
    AP_MXECAN_Driver *_driver;
};
namespace AP {
    AP_MXECAN *mxecan();
};

#endif // AP_MXECAN_ENABLED
