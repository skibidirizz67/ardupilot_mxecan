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
    if (_driver != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:init: another instance exists");
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::MXECAN) {
            _driver = NEW_NOTHROW AP_MXECAN_Driver();
            return;
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:init: no driver found");
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
        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:handle: frame not extended");
        return;
    }

    uint32_t can_id = frame.id & AP_HAL::CANFrame::MaskExtID;

#if AP_MXECAN_DEBUG
    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:handle: can_id=%lu len=%u", can_id, frame.dlc);
#endif

    if (can_id != AUTOPILOT_NODE_ID) {
        // not for us or invalid id
        return;
    }

#if AP_MXECAN_DEBUG
    // all MX_CAN-SV3.03 frames should have dlc of 8
    if (frame.dlc != MXECAN_DLC_SIZE) {
        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"MXECAN:handle: invalid dlc=%u", frame.dlc);
        return;
    }
#endif

    mxecan_frame_t mxecan_data = {0};
    memcpy(&mxecan_data, frame.data, 8);

    // TODO: update_rpm / update_telem_data
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
    if (now_ms - last_send_ms > 1000 && 0) {
        last_send_ms = now_ms;
        //GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"%u: %u, %u, %u, %u, %u, %u, %u, %u",
        //0,
        //(unsigned)_output.pwm[0], (unsigned)_output.pwm[1], (unsigned)_output.pwm[2], (unsigned)_output.pwm[3],
        //(unsigned)_output.pwm[4], (unsigned)_output.pwm[5], (unsigned)_output.pwm[6], (unsigned)_output.pwm[7]);
    }
#endif
}

float AP_MXECAN_Driver::apply_expo(float val, float expo)
{
    if (val < 0.0001f || val > -0.0001) return 0.0f;
    const float sign = (val > 0.0f) ? 1.0f : -1.0f;
    return sign * powf(fabsf(val), expo);
}

void AP_MXECAN_Driver::loop()
{
#if AP_MXECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    uint32_t last_tick_ms = 0;
    uint32_t last_print_ms = 0;

    while (true) {
#if AP_MXECAN_USE_EVENTS
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(50000));
 #else
        hal.scheduler->delay_microseconds(50000);
#endif

        const uint32_t now_ms = AP_HAL::millis();
        if (last_tick_ms && now_ms - last_tick_ms < 50) continue;
        last_tick_ms = now_ms;

        const bool armed = AP::arming().is_armed();

        uint16_t ch1_pwm = 0;
        uint16_t ch2_pwm = 0;
        uint16_t ch3_pwm = 0;
        uint16_t ch6_pwm = 0;

        ch1_pwm = hal.rcin->read(1); 
        ch2_pwm = hal.rcin->read(2); 
        ch3_pwm = hal.rcin->read(3); 
        ch6_pwm = hal.rcin->read(6); 

        const AP_Vehicle *vehicle = AP::vehicle();
        const uint8_t mode = vehicle->get_mode();

        bool brake_active = false;
        if (!armed) brake_active = true;
        else if (ch6_pwm > 1500) brake_active = true;
        else if (_driver_has_fault) brake_active = true;
        brake_active = false;

        float target_rpm1 = 0.0f;
        float target_rpm2 = 0.0f;
        uint8_t control_byte = 0xC3;

        float steer = 0.0f;
        float throttle = 0.0f;

        float diff2;
        if (1) {
            float diff1 = (float)ch1_pwm - 1500.0f;
            if (fabsf(diff1) > 30.0f) steer = diff1 / 500.0f;

            diff2 = ch2_pwm;
            if (fabsf(diff2) > 30.0f) throttle = diff2 / 500.0f;

            steer = apply_expo(steer, _EXPO_STEER);
            //throttle = apply_expo(throttle, _EXPO_THROTTLE);
        }
        else {
            const float steering_raw = SRV_Channels::get_output_scaled(SRV_Channel::k_steering);
            const float throttle_raw = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);


            steer = steering_raw / 4500.0f;
            throttle = throttle_raw / 100.0f;
        }

        steer = constrain_float(steer, -1.0f, 1.0f);
        throttle = constrain_float(throttle, -1.0f, 1.0f);

        float speed_multiplier = 1.0f;
        if (ch3_pwm) {
            speed_multiplier = (float)(ch3_pwm - 1000) / 1000.0f;
            speed_multiplier = constrain_float(speed_multiplier, 0.0f, 1.0f);
        }
        const float current_max_rpm = _MAX_RPM * speed_multiplier;

        float left_out  = throttle - steer;
        float right_out = -(throttle + steer);

        left_out = constrain_float(left_out, -1.0f, 1.0f);
        right_out = constrain_float(right_out, -1.0f, 1.0f);

        target_rpm1 = floorf(left_out * current_max_rpm);
        target_rpm2 = floorf(right_out * current_max_rpm);

        float d1 = target_rpm1 - _current_rpm1;
        if (d1 > _MAX_RPM_STEP) _current_rpm1 += _MAX_RPM_STEP;
        else if (d1 < -_MAX_RPM_STEP) _current_rpm1 -= _MAX_RPM_STEP;
        else _current_rpm1 = target_rpm1;

        float d2 = target_rpm2 - _current_rpm2;
        if (d2 > _MAX_RPM_STEP) _current_rpm2 += _MAX_RPM_STEP;
        else if (d2 < -_MAX_RPM_STEP) _current_rpm2 -= _MAX_RPM_STEP;
        else _current_rpm2 = target_rpm2;

        int32_t u_rpm1 = (int32_t)floorf(_current_rpm1);
        int32_t u_rpm2 = (int32_t)floorf(_current_rpm2);

        if (u_rpm1 < 0) u_rpm1 = 65536 + u_rpm1;
        if (u_rpm2 < 0) u_rpm2 = 65536 + u_rpm2;

        AP_HAL::CANFrame frame = AP_HAL::CANFrame(
            (ESC_NODE_ID | AP_HAL::CANFrame::FlagEFF),
            nullptr, 8, false
        );

        uint8_t data[8] = {0};
        data[0] = control_byte;
        data[1] = (uint8_t)((u_rpm2 >> 8) & 0xFF);
        data[2] = (uint8_t)(u_rpm2 & 0xFF);
        data[3] = 0x64;
        data[4] = (uint8_t)((u_rpm1 >> 8) & 0xFF);
        data[5] = (uint8_t)(u_rpm1 & 0xFF);
        data[6] = 0x64;
        data[7] = 0x00;

        frame = AP_HAL::CANFrame((ESC_NODE_ID | AP_HAL::CANFrame::FlagEFF), data, 8, false);

        bool result = write_frame(frame, 1000);

#if AP_MXECAN_DEBUG
        if (now_ms - last_print_ms > 1500) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "UMTR: Out1=%d lout=%f thr=%f dif=%f Out2=%d Brk=%d Arm=%d Mode=%d ch3=%d res=%d ch1=%d ch2=%d ch6=%d",
                    (int)_current_rpm1, left_out, throttle, diff2, (int)_current_rpm2,
                    brake_active,
                    armed,
                    (int)mode, (int)ch3_pwm,
                    result,
                    ch1_pwm, ch2_pwm, ch6_pwm);
            last_print_ms = now_ms;
        }
#endif

    } // while true
}

bool AP_MXECAN_Driver::send_packet(const uint32_t timeout_us, const uint8_t *data)
{
    AP_HAL::CANFrame frame = AP_HAL::CANFrame((ESC_NODE_ID | AP_HAL::CANFrame::FlagEFF), data, 8, false);
    return true;
    return write_frame(frame, timeout_us);
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

