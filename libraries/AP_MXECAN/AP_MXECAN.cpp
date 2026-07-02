#include "AP_MXECAN.h"

#if AP_MXECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX
#include <AP_Arming/AP_Arming.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define UNUSED(x) (void)(x)

#define AP_MXECAN_DEBUG 1

extern const AP_HAL::HAL& hal;

AP_MXECAN *AP_MXECAN::_singleton;

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_MXECAN::var_info[] = {
    AP_GROUPEND
};

AP_MXECAN::AP_MXECAN() {
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) AP_HAL::panic("AP_MXECAN must be singleton");
#endif
    _singleton = this;
}

void AP_MXECAN::init() {
    // only allow one instance
    if (_driver != nullptr) {
#if AP_MXECAN_DEBUG
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:init: another instance exists");
#endif
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::MXECAN) {
            _driver = NEW_NOTHROW AP_MXECAN_Driver();
            return;
        }
    }

#if AP_MXECAN_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:init: no driver found");
#endif
}

void AP_MXECAN::update() {
    if (_driver == nullptr) return;
    _driver->update();
}

AP_MXECAN_Driver::AP_MXECAN_Driver() : CANSensor("MXECAN") {
    register_driver(AP_CAN::Protocol::MXECAN);

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MXECAN_Driver::loop, void), "mxecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

void AP_MXECAN_Driver::update() {

}

void AP_MXECAN_Driver::handle_frame(AP_HAL::CANFrame &frame) {
    if (!frame.isExtended()) return;

    uint32_t can_id = frame.id & AP_HAL::CANFrame::MaskExtID;

    if (can_id != AUTOPILOT_NODE_ID) return;

    // all MX_CAN-SV3.03 frames should have dlc of 8
    if (frame.dlc != 8) return;

    // TODO: update telemetry data
}

void AP_MXECAN_Driver::loop() {
    while (true) {
        hal.scheduler->delay_microseconds(loop_delay_us);

        bool armed = AP::arming().is_armed(); UNUSED(armed);
        uint16_t periods[8] = {0};
        hal.rcin->read(periods, 8);
        uint8_t mode = AP::vehicle()->get_mode(); UNUSED(mode);

        uint8_t data[8] = {0};
        AP_HAL::CANFrame frame = AP_HAL::CANFrame((ESC_NODE_ID | AP_HAL::CANFrame::FlagEFF), data, 8, false);

        bool result = write_frame(frame, 1000); UNUSED(result);
    }
}

namespace AP {
AP_MXECAN *mxecan() {
    return AP_MXECAN::get_singleton();
}
};

#endif // AP_MXECAN_ENABLED

