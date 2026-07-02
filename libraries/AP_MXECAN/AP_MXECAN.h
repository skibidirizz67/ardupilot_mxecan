#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_MXECAN_ENABLED
#define AP_MXECAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_PROGRAM_SIZE_LIMIT_KB > 2048)
#endif

#if AP_MXECAN_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>

class AP_MXECAN_Driver : public CANSensor {
public:
    AP_MXECAN_Driver();

    void update();

private:
    static const uint32_t ESC_NODE_ID = 0x1801E600; // aka driver RX ID
    static const uint32_t AUTOPILOT_NODE_ID = 0x1801E001; // aka driver TX ID

    static const uint16_t loop_delay_us = 50000; // 50 ms

    void handle_frame(AP_HAL::CANFrame &frame) override;
    void loop();
};

class AP_MXECAN {
public:
    AP_MXECAN();
    CLASS_NO_COPY(AP_MXECAN);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();
    static AP_MXECAN *get_singleton() { return _singleton; }

private:
    static AP_MXECAN *_singleton;
    AP_MXECAN_Driver *_driver;
};

namespace AP {
    AP_MXECAN *mxecan();
};

#endif // AP_MXECAN_ENABLED
