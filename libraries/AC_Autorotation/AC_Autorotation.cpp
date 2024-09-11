#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

// Autorotation controller defaults
// Head Speed (HS) controller specific default definitions
#define HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ          2.0     // low-pass filter on accel error (unit: hz)
#define HS_CONTROLLER_HEADSPEED_P                     0.7     // Default P gain for head speed controller (unit: -)
#define HEAD_SPEED_TARGET_RATIO                       1.0    // Normalised target main rotor head speed

// Speed Height controller specific default definitions for autorotation use
#define AP_FW_VEL_P                       9.0


const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head speed controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation.  Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: TARG_SP
    // @DisplayName: Target Glide Ground Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: m/s
    // @Range: 8 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("TARG_SP", 4, AC_Autorotation, _param_target_speed, 11),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the entry phase.  Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, 0.7),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter.  For the glide phase.  Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, 0.1),

    // @Param: FW_ACC_MAX
    // @DisplayName: Forward Acceleration Limit
    // @Description: Maximum forward acceleration to apply in speed controller.
    // @Units: m/s/s
    // @Range: 0.3 2.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FW_ACC_MAX", 7, AC_Autorotation, _param_accel_max, 1.4),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor 
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.  RPM1 = 0.  RPM2 = 1.
    // @Units: s
    // @Range: 0.5 3
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("HS_SENSOR", 8, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: FW_V_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Determines the proportion of the target acceleration based on the velocity error.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Standard
    AP_SUBGROUPINFO(_p_fw_vel, "FW_V_", 9, AC_Autorotation, AC_P),

    // @Param: FW_V_FF
    // @DisplayName: Velocity (horizontal) feed forward
    // @Description: Velocity (horizontal) input filter.  Corrects the target acceleration proportionally to the desired velocity.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("FW_V_FF", 10, AC_Autorotation, _param_fwd_k_ff, 1.5),

    // @Param: FW_JERK_MAX
    // @DisplayName: Forward jerk limit
    // @Description: Maximum forward jerk to apply in speed controller.
    // @Units: m/s/s/s
    // @Range: 0.1 4.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("FW_JERK_MAX", 11, AC_Autorotation, _param_jerk_max, 0.46),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_AHRS& ahrs, AP_MotorsHeli*& motors, AC_PosControl*& pos_ctrl, AC_AttitudeControl*& att_crtl) :
    _ahrs(ahrs),
    _motors_heli(motors),
    _pos_control(pos_ctrl),
    _attitude_control(att_crtl),
    _p_hs(HS_CONTROLLER_HEADSPEED_P),
    _p_fw_vel(AP_FW_VEL_P)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _desired_heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
    }

void AC_Autorotation::init(void) {
    // Initialisation of head speed controller
    // Set initial collective position to be the current collective position for smooth init
    _collective_out = _motors_heli->get_throttle_out();

    // Reset feed forward filter
    col_trim_lpf.reset(_collective_out);

    // Protect against divide by zero
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point, 500.0));

    // Set limits and initialise xy pos controller
    _pos_control->set_max_speed_accel_xy(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_correction_speed_accel_xy(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_pos_error_max_xy_cm(1000);
    _pos_control->init_xy_controller();

    // Init to current heading
    _yaw_rate = _ahrs.get_yaw_rate_earth();
}

// Functions and config that are only to be done once at the beginning of the entry
void AC_Autorotation::init_entry(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");

    // Set desired forward speed target
    _fwd_spd_desired = _param_target_speed.get();

    // Target head speed is set to rpm at initiation to prevent steps in controller
    _target_head_speed = get_norm_head_speed();

    // The decay rate to reduce the head speed from the current to the target
    _hs_decay = (_target_head_speed - HEAD_SPEED_TARGET_RATIO) / (float(entry_time_ms)*1e-3);

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq.get());

    // set collective 
    _motors_heli->set_throttle_filter_cutoff(HS_CONTROLLER_COLLECTIVE_CUTOFF_FREQ);
}

// The entry controller just a special case of the glide controller with head speed target slewing
void AC_Autorotation::run_entry(float pilot_norm_accel)
{
    // Slowly change the target head speed until the target head speed matches the parameter defined value
    float norm_rpm = get_norm_head_speed();
    if (norm_rpm > HEAD_SPEED_TARGET_RATIO*1.05f  ||  norm_rpm < HEAD_SPEED_TARGET_RATIO*0.95f) {
        // Outside of 5% of target head speed so we slew target towards the set point
        _target_head_speed -= _hs_decay * _dt;
    } else {
        _target_head_speed = HEAD_SPEED_TARGET_RATIO;
    }

    run_glide(pilot_norm_accel);
}

// Functions and config that are only to be done once at the beginning of the glide
void AC_Autorotation::init_glide(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Glide Phase");

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_glide_cutoff_freq.get());

    // Ensure target headspeed is set to setpoint, in case it didn't reach the target during entry
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // Ensure desired forward speed target is set to param value
    _fwd_spd_desired = _param_target_speed.get();
}

// Maintain head speed and forward speed as we glide to the ground
void AC_Autorotation::run_glide(float pilot_norm_accel)
{
    update_headspeed_controller();

    // Set body frame velocity targets
    desired_velocity_bf.x = _fwd_spd_desired;
    desired_velocity_bf.y = 0.0; // Always want zero side slip

    // Set body frame accel targets. Pilot requests lateral accel.
    desired_accel_bf.y = desired_accel_bf.y * 0.975 + pilot_norm_accel * _param_accel_max.get() * 0.025; // approx 10 hz low-pass filter
    desired_accel_bf.x = 0.0; // Do not use forward accel feed-forward

    // Based on the requested lateral accel, calc the necessary yaw rate to achieve a coordinated turn
    const float curr_fwd_speed = get_speed_forward();
    const float delta_v = MIN((_fwd_spd_desired - curr_fwd_speed), (_param_accel_max.get() * _dt));
    const float projected_vel = curr_fwd_speed + delta_v; // predicted velocity in the next time step

    // Calc yaw rate from desired body-frame accels
    // this seems suspiciously simple, but it is correct
    // accel = r * w^2
    // radius can be calculated as the distance traveled in the time it take to do 360 deg
    // One rotation takes: (2*pi)/w  seconds
    // Distance traveled in that time: (s*2*pi)/w
    // radius for that distance: ((s*2*pi)/w) / (2*pi)
    // r = s / w
    // accel = (s / w) * w^2
    // accel = s * w
    // w = accel / s
    const float yaw_rate = desired_accel_bf.y / projected_vel;

    // Smoothly update desired yaw rate
    _yaw_rate = _yaw_rate * 0.975 + yaw_rate * 0.025; // approx 10 hz low-pass filter
    _desired_heading.yaw_rate_cds = degrees(_yaw_rate)*100.0;

    // Update the position controller
    update_xy_speed_controller();
}

void AC_Autorotation::update_headspeed_controller(void)
{
    // Get current rpm and update healthy signal counters
    const float head_speed_norm = get_norm_head_speed();

    if (head_speed_norm > 0.0) {
        // Calculate the head speed error.
        _head_speed_error = head_speed_norm - _target_head_speed;

        _p_term_hs = _p_hs.get_p(_head_speed_error);

        // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
        _ff_term_hs = col_trim_lpf.apply(_collective_out, _dt);

        // Calculate collective position to be set
        _collective_out = constrain_value((_p_term_hs + _ff_term_hs), 0.0f, 1.0f) ;

    } else {
         // RPM sensor is bad, set collective to angle of -2 deg and hope for the best
         _collective_out = _motors_heli->calc_coll_from_ang(-2.0);
    }

    // Send collective to setting to motors output library
    _motors_heli->set_throttle(_collective_out);
}


// Helper to get measured head speed that has been normalised by head speed set point
float AC_Autorotation::get_norm_head_speed(void) const
{
    // assuming zero rpm is safer as it will drive collective in the direction of increasing head speed
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // checking to ensure no nullptr, we do have a pre-arm check for this so it will be very bad if RPM has gone away
    if (rpm == nullptr) {
        return 0.0;
    }

    // Check RPM sensor is returning a healthy status
    if (!rpm->get_rpm(_param_rpm_instance.get(), current_rpm)) {
        return 0.0;
    }

#endif

    // Protect against div by zeros later in the code
    float head_speed_set_point = MAX(1.0, _param_head_speed_set_point.get());

    // Normalize the RPM by the setpoint
    return current_rpm/head_speed_set_point;
}

// Update speed controller
// Vehicle is trying to achieve and maintain the desired speed in the body-frame forward direction.
// During the entry and glide phases the pilot can navigate via a yaw rate input and coordinated roll is calculated.
void AC_Autorotation::update_xy_speed_controller(void)
{
    // Convert from body-frame to earth-frame
    desired_velocity_ef = _ahrs.body_to_earth2D(desired_velocity_bf) * 100.0;
    desired_accel_ef = _ahrs.body_to_earth2D(desired_accel_bf) * 100.0;

    // Update the position controller
    _pos_control->input_vel_accel_xy(desired_velocity_ef, desired_accel_ef, true);
    _pos_control->update_xy_controller();

    // Output to the attitude controller
    _attitude_control->input_thrust_vector_heading(_pos_control->get_thrust_vector(), _desired_heading);
}

// smoothly zero velocity and accel
void AC_Autorotation::run_landed(void)
{
    desired_velocity_bf.x *= 0.95;
    desired_velocity_bf.y *= 0.95;
    desired_accel_bf.x *= 0.95;
    desired_accel_bf.y *= 0.95;
    update_xy_speed_controller();
    _pos_control->soften_for_landing_xy();
}

// Determine the forward ground speed component from measured components
float AC_Autorotation::get_speed_forward(void) const
{
    Vector2f groundspeed_vector = _ahrs.groundspeed_vector();
    return groundspeed_vector.x*_ahrs.cos_yaw() + groundspeed_vector.y*_ahrs.sin_yaw(); // (m/s)
}

Vector3f AC_Autorotation::get_bf_accel(void) const
{
    Vector3f ef_accel = _ahrs.get_accel_ef();
    return _ahrs.earth_to_body(ef_accel);
}

Vector3f AC_Autorotation::get_bf_vel(void) const
{
    Vector3f ef_vel;
    if (_ahrs.get_velocity_NED(ef_vel)) {
        return _ahrs.earth_to_body(ef_vel);
    }
    // TODO: probably need to think of some smart handing case for returning false on this method
    return ef_vel;
}


#if HAL_LOGGING_ENABLED
void AC_Autorotation::log_write_autorotation(void) const
{
// @LoggerMessage: AROT
// @Vehicles: Copter
// @Description: Helicopter Autorotation information
// @Field: TimeUS: Time since system startup
// @Field: P: P-term for headspeed controller response
// @Field: hserr: head speed error; difference between current and desired head speed
// @Field: ColOut: Collective Out
// @Field: FFCol: FF-term for headspeed controller response
// @Field: SpdF: current forward speed
// @Field: DFS: desired forward speed
// @Field: p: p-term of velocity response
// @Field: ff: ff-term of velocity response
// @Field: AccT: forward acceleration target
// @Field: LR: Landed Reason state flags
// @FieldBitmaskEnum: LR: Copter::ModeAutorotate

    //Write to data flash log
    AP::logger().WriteStreaming("AROT",
                       "TimeUS,P,hserr,ColOut,FFCol,SpdF,CmdV,p,ff,AccT,LR",
                        "QfffffffffB",
                        AP_HAL::micros64(),
                        _p_term_hs,
                        _head_speed_error,
                        _collective_out,
                        _ff_term_hs,
                        get_speed_forward(),
                        _fwd_spd_desired,
                        _vel_p,
                        _vel_ff,
                        _accel_target,
                        _landed_reason);


    // @LoggerMessage: ARO2
    // @Vehicles: Copter
    // @Description: Helicopter Autorotation information 2
    // @Field: TimeUS: Time since system startup
    // @Field: VXB: Desired velocity X in body frame
    // @Field: VYB: Desired velocity Y in body frame
    // @Field: AXB: Desired Acceleration X in body frame
    // @Field: AYB: Desired Acceleration Y in body frame
    // @Field: VXE: Desired velocity X in earth frame
    // @Field: VYE: Desired velocity Y in earth frame
    // @Field: AXE: Desired Acceleration X in earth frame
    // @Field: AYE: Desired Acceleration Y in earth frame

    //Write to data flash log
    AP::logger().WriteStreaming("ARO2",
                       "TimeUS,VXB,VYB,AXB,AYB,VXE,VYE,AXE,AYE,dt",
                        "Qfffffffff",
                        AP_HAL::micros64(),
                        desired_velocity_bf.x,
                        desired_velocity_bf.y,
                        desired_accel_bf.x,
                        desired_accel_bf.y,
                        desired_velocity_ef.x,
                        desired_velocity_ef.y,
                        desired_accel_ef.x,
                        desired_accel_ef.y,
                        _dt);
}
#endif  // HAL_LOGGING_ENABLED

// Arming checks for autorotation, mostly checking for miss-configurations
bool AC_Autorotation::arming_checks(size_t buflen, char *buffer) const
{
    if (!enabled()) {
        // Don't run arming checks if not enabled
        return true;
    }

    const AP_HAL::HAL& hal = AP_HAL::get_HAL();

    // Check for correct RPM sensor config
#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm == nullptr) {
        hal.util->snprintf(buffer, buflen, "Can't access RPM");
        return false;
    }

    // Sanity check that the designated rpm sensor instance is there
    if ((_param_rpm_instance.get() < 0)) {
        hal.util->snprintf(buffer, buflen, "RPM instance <0");
        return false;
    }

    if (!rpm->enabled(_param_rpm_instance.get())) {
        hal.util->snprintf(buffer, buflen, "RPM%i not enabled", _param_rpm_instance.get()+1);
        return false;
    }
#endif

    // Check that heli motors is configured for autorotation
    if (!_motors_heli->rsc_autorotation_enabled()) {
        hal.util->snprintf(buffer, buflen, "H_RSC_AROT_* not configured");
        return false;
    }

    return true;
}

