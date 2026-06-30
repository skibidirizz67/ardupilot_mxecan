#include "AP_MXECAN.h"

#if AP_MXECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

#define AP_MXECAN_DEBUG 1

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_MXECAN::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_MXECAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_MXECAN::AP_MXECAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_MXECAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_MXECAN::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"initializing MXECAN");
    if (_driver != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"another instance already exists");
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::MXECAN) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"found MXECAN CAN interface");
            _driver = NEW_NOTHROW AP_MXECAN_Driver();
            return;
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"could not find MXECAN CAN interface");
}

void AP_MXECAN::update()
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"updating MXECAN");
    if (_driver == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"no MXECAN :noboobs:");
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_MXECAN_Driver::AP_MXECAN_Driver() : CANSensor("MXECAN")
{
    register_driver(AP_CAN::Protocol::MXECAN);

    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"registering mxecan driver and creating loop thread");

    // start thread for receiving and sending CAN frames. Tests show we use about 640 bytes of stack
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MXECAN_Driver::loop, void), "mxecan", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

// parse inbound frames
void AP_MXECAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    uint32_t can_id = frame.id & AP_HAL::CANFrame::MaskExtID;

#if AP_MXECAN_DEBUG
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"MXECAN: can id:%lu, len:%u", can_id, frame.dlc);
#endif

    if (can_id != AUTOPILOT_NODE_ID) {
        // not for us or invalid id
        return;
    }

#if AP_MXECAN_DEBUG
    // all MX_CAN-SV3.03 frames should have dlc of 8
    if (frame.dlc != MXECAN_DLC_SIZE) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"MXECAN: invalid dlc value: %u != %u", frame.dlc, MXECAN_DLC_SIZE);
        return;
    }
#endif

    mxecan_frame_t mxecan_data = {0};
    memcpy(&mxecan_data, frame.data, 8);

    // TODO: update_rpm / update_telem_data

#if AP_MXECAN_DEBUG
    // all MX_CAN-SV3.03 frames should have dlc of 8
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"MXECAN: data: %08llX", (uint64_t)frame.data[0]);

    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"fault_code=%u driver_temp=%u axis2_speed=%d axis1_speed=%d power_voltage=%u",
        mxecan_data.fault_code, mxecan_data.driver_temperature,
        mxecan_data.axis2_speed, mxecan_data.axis1_speed, mxecan_data.power_voltage);
#endif
}

void AP_MXECAN_Driver::update(const uint8_t num_poles)
{
    WITH_SEMAPHORE(_output.sem);
    for (uint8_t i = 0; i < ARRAY_SIZE(_output.pwm); i++) {
        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            _output.pwm[i] = 0;
            continue;
        }
        _output.pwm[i] = c->get_output_pwm();
    }

    _output.is_new = true;

#if AP_MXECAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif

#if AP_MXECAN_DEBUG
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms > 1000) {
        last_send_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
        0,
        (unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
        (unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
    }
#endif
}

void AP_MXECAN_Driver::loop()
{
    uint16_t pwm[ARRAY_SIZE(_output.pwm)] {};

    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"launching loop");

#if AP_MXECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    while (true) {
#if AP_MXECAN_USE_EVENTS
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500);
#endif

        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"loop");

        const uint32_t now_ms = AP_HAL::millis();

        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));

            } else if (_output.last_new_ms && now_ms - _output.last_new_ms > LAST_OUTPUT_TIMEOUT_MS) {
                // if we haven't gotten any PWM updates for a bit, zero it
                // out so we don't just keep sending the same values forever
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }

        
    } // while true
}

bool AP_MXECAN_Driver::send_packet(const uint32_t timeout_us, const uint8_t *data)
{
    //AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);
    return true;
    //return write_frame(frame, timeout_us);
}

// singleton instance
AP_MXECAN *AP_MXECAN::_singleton;

namespace AP {
AP_MXECAN *mxecan()
{
    return AP_MXECAN::get_singleton();
}
};

#endif // AP_MXECAN_ENABLED

