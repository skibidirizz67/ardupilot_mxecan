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

#define UNUSED(x) (void)(x)

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
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::MXECAN) {
            _driver = NEW_NOTHROW AP_MXECAN_Driver();
            return;
        }
    }
}

void AP_MXECAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_MXECAN_Driver::AP_MXECAN_Driver() : CANSensor("MXECAN")
{
    register_driver(AP_CAN::Protocol::MXECAN);

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
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"MXECAN: can id:%lu, len:%u", can_id, frame.dlc);
#endif

    if (can_id != AUTOPILOT_NODE_ID) {
        // not for us or invalid id
        return;
    }

#if AP_MXECAN_DEBUG
    // all MX_CAN-SV3.03 frames should have dlc of 8
    if (frame.dlc != MXECAN_DLC_SIZE) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"MXECAN: invalid dlc value: %u != %u", frame.dlc, MXECAN_DLC_SIZE);
        return;
    }
#endif

    uint8_t fault_code = frame.data[0];
    uint8_t driver_temperature = frame.data[1];
    int16_t axis2_speed = (int16_t)(((uint16_t)frame.data[2] << 8) | frame.data[3]);
    int16_t axis1_speed = (int16_t)(((uint16_t)frame.data[4] << 8) | frame.data[5]);
    uint16_t power_voltage = ((uint16_t)frame.data[6] << 8) | frame.data[7];

    // TODO: update_rpm / update_telem_data

#if AP_MXECAN_DEBUG
    // all MX_CAN-SV3.03 frames should have dlc of 8
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"MXECAN: data: %08llX", (uint64_t)frame.data[0]);

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"fault_code=%u driver_temp=%u axis2_speed=%d axis1_speed=%d power_voltage=%u",
        fault_code, driver_temperature, axis2_speed, axis1_speed, power_voltage);
#endif
}

void AP_MXECAN_Driver::update(const uint8_t num_poles)
{
#if HAL_WITH_ESC_TELEM
    _telemetry.num_poles = num_poles;
#endif
    
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
        0,
        (unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
        (unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
    }
#endif
}

void AP_MXECAN_Driver::loop()
{
    uint16_t pwm[ARRAY_SIZE(_output.pwm)] {};

#if AP_MXECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    while (true) {
#if AP_MXECAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 400Hz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 400Hz
#endif

        const uint32_t now_ms = AP_HAL::millis();

        // This should run at 400Hz
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));

            } else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000) {
                // if we haven't gotten any PWM updates for a bit, zero it
                // out so we don't just keep sending the same values forever
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }

#if HAL_WITH_ESC_TELEM
        // TODO: discover what's this for
#endif // HAL_WITH_ESC_TELEM
    } // while true
}

bool AP_MXECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data)
{
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_us, (uint8_t*)&data_be16, 2);
}

bool AP_MXECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data, const uint8_t data_len)
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

