# HOSM Wind Estimator for Multirotors - Implementation Plan

## Overview
Implement a standalone Higher-Order Sliding Mode (HOSM) wind estimator module for PX4 multirotors using **drag-based wind estimation** (no airspeed sensor required). The estimator will use IMU body accelerations, GPS velocity, and a drag model to estimate wind components.

## User Requirements
- **Type**: Standalone module (not integrated with EKF2)
- **Sensor inputs**: IMU accelerations + GPS velocity + Attitude (NO airspeed sensor)
- **Target vehicle**: Multirotor
- **Update rate**: 50-100 Hz (high-frequency)
- **Purpose**: Complementary to EKF2 wind estimation
- **Approach**: Drag-based estimation (bluff body + rotor momentum drag)

## 1. Wind Estimation Approach: Drag-Based Model

### Physical Model

For a multirotor, wind creates a relative velocity that induces drag forces on the vehicle. These drag forces appear as accelerations measured by the IMU:

**Drag Model** (from EKF2 drag_fusion.cpp:128):
```
a_drag = -0.5 * (1/bcoef) * rho * v_rel * |v_rel| - v_rel * mcoef
```

Where:
- `a_drag`: Drag acceleration in body frame (m/s²)
- `bcoef`: Bluff body drag coefficient (inverse) [m²/kg]
- `rho`: Air density (kg/m³)
- `v_rel`: Relative velocity in body frame = R_body_ned^-1 * (v_ground - wind)
- `mcoef`: Rotor momentum drag coefficient [1/s]
- `v_ground`: Ground velocity in NED frame (from GPS)
- `wind`: Wind velocity in NED frame [w_n, w_e] (to be estimated)

**Measurement Model**:
```
a_measured = a_imu - a_bias  (body frame, from IMU)
a_predicted = drag_model(v_ground, wind, attitude, rho, drag_params)
innovation = a_measured - a_predicted
```

The HOSM observer estimates `wind = [w_n, w_e]` to drive the innovation to zero.

### HOSM Observer Formulation

**State**: `x = [w_n, w_e]^T` (wind in NED frame, North and East components)

**Sliding Surface**: `σ = [σ_x, σ_y]^T` where:
```
σ_x = a_measured_x - a_predicted_x(w_n, w_e)
σ_y = a_measured_y - a_predicted_y(w_n, w_e)
```

**Super-Twisting HOSM Observer**:
```
State dynamics:
ẇ_n = v₁_n
ẇ_e = v₁_e

Observer equations:
ŵ̇_n = -λ₁ |σ_n|^(1/2) sign(σ_n) + v₁_n
ŵ̇_e = -λ₁ |σ_e|^(1/2) sign(σ_e) + v₁_e

Auxiliary state:
v̇₁_n = -λ₂ sign(σ_n)
v̇₁_e = -λ₂ sign(σ_e)

Where σ_n, σ_e are computed from body X, Y drag innovations
```

**Key difference from airspeed-based**: The innovation comes from IMU acceleration residuals rather than airspeed measurement residuals.

## 2. Module Structure

### Directory Layout
```
src/modules/wind_estimator_hosm/
├── CMakeLists.txt
├── Kconfig
├── WindEstimatorHosm.hpp
├── WindEstimatorHosm.cpp
├── hosm_wind_observer.hpp
├── hosm_wind_observer.cpp
├── wind_estimator_hosm_params.c
└── (optional) hosm_wind_observer_test.cpp
```

### Class Architecture

**WindEstimatorHosm.hpp** (Main module):
```cpp
class WindEstimatorHosm : public ModuleBase<WindEstimatorHosm>,
                          public ModuleParams,
                          public px4::WorkItem
{
public:
    WindEstimatorHosm();
    ~WindEstimatorHosm() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    void Run() override;
    void updateParams() override;
    void reset();
    void publishWindEstimate(const hrt_abstime &timestamp_sample);

    // HOSM observer instance
    HosmWindObserver _hosm_observer{};

    // Publications
    uORB::PublicationMulti<wind_s> _wind_pub{ORB_ID(wind)};

    // Primary callback subscription (IMU at ~250 Hz, downsample to 100 Hz)
    uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};

    // Regular subscriptions
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
    uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // State tracking
    hrt_abstime _timestamp_last{0};
    uint8_t _imu_sample_count{0};
    Vector2f _imu_accel_accum{0.f, 0.f};
    bool _armed{false};
    bool _in_air{false};
    bool _valid{false};

    systemlib::Hysteresis _valid_hysteresis{false};
    perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::HOSM_WIND_GAIN_L>) _param_hosm_wind_gain_l,
        (ParamFloat<px4::params::HOSM_WIND_GAIN_K>) _param_hosm_wind_gain_k,
        (ParamFloat<px4::params::HOSM_WIND_ALPHA>) _param_hosm_wind_alpha,
        (ParamFloat<px4::params::HOSM_WIND_BETA>) _param_hosm_wind_beta,
        (ParamFloat<px4::params::HOSM_MIN_GPS_VEL>) _param_hosm_min_gps_vel,
        (ParamFloat<px4::params::HOSM_BCOEF_X>) _param_hosm_bcoef_x,
        (ParamFloat<px4::params::HOSM_BCOEF_Y>) _param_hosm_bcoef_y,
        (ParamFloat<px4::params::HOSM_MCOEF>) _param_hosm_mcoef,
        (ParamFloat<px4::params::HOSM_DRAG_NOISE>) _param_hosm_drag_noise,
        (ParamInt<px4::params::HOSM_WIND_ENABLE>) _param_hosm_wind_enable
    )
};
```

**hosm_wind_observer.hpp** (Algorithm library):
```cpp
class HosmWindObserver
{
public:
    HosmWindObserver() = default;
    ~HosmWindObserver() = default;

    // Core observer update with drag measurements
    void update(const matrix::Vector2f &accel_body_xy,
                const matrix::Vector3f &velocity_ground_ned,
                const matrix::Quatf &attitude,
                float air_density,
                float dt);

    // State access
    matrix::Vector2f getWindEstimate() const { return _wind_state; }
    matrix::Vector2f getWindVariance() const { return _wind_variance; }
    matrix::Vector2f getSlidingVariable() const { return _sliding_surface; }

    // Diagnostic access
    float getConvergenceMetric() const { return _convergence_metric; }
    bool isConverged() const { return _is_converged; }

    // Configuration
    void setGains(float lambda1, float lambda2);
    void setDragCoefficients(float bcoef_x, float bcoef_y, float mcoef);
    void setSmoothingParameter(float epsilon);
    void setMinGroundSpeed(float min_gs) { _min_ground_speed = min_gs; }
    void reset();

private:
    // State
    matrix::Vector2f _wind_state{0.f, 0.f};           // [w_n, w_e]
    matrix::Vector2f _aux_state{0.f, 0.f};            // [v1_n, v1_e]
    matrix::Vector2f _sliding_surface{0.f, 0.f};      // [σ_x, σ_y]
    matrix::Vector2f _wind_variance{100.f, 100.f};

    // Observer gains
    float _lambda1{2.0f};
    float _lambda2{1.5f};
    float _epsilon_smooth{0.1f};

    // Drag model parameters
    float _bcoef_x{25.0f};   // Bluff body drag coefficient X (inverse)
    float _bcoef_y{25.0f};   // Bluff body drag coefficient Y (inverse)
    float _mcoef{0.1f};      // Rotor momentum drag coefficient

    // Convergence tracking
    float _convergence_metric{1000.f};
    bool _is_converged{false};
    float _min_ground_speed{2.0f};

    // Helpers
    float smoothSign(float x, float epsilon) const;
    matrix::Vector2f computePredictedDragAccel(
        const matrix::Vector3f &v_ground_ned,
        const matrix::Quatf &q_att,
        float rho) const;
    void updateVarianceEstimate(const matrix::Vector2f &innovation, float dt);
    void updateConvergenceMetric(float dt);
};
```

## 3. Algorithm Implementation

### Core HOSM Update Method

```cpp
void HosmWindObserver::update(const Vector2f &accel_body_xy,
                               const Vector3f &v_ground_ned,
                               const Quatf &q_att,
                               float rho,
                               float dt)
{
    // Validate inputs
    if (v_ground_ned.xy().norm() < _min_ground_speed) {
        return;  // Insufficient excitation for wind observability
    }

    // 1. Compute relative velocity in body frame
    Vector3f wind_ned(_wind_state(0), _wind_state(1), 0.f);
    Vector3f rel_vel_ned = v_ground_ned - wind_ned;
    Vector3f rel_vel_body = q_att.rotateVectorInverse(rel_vel_ned);
    float rel_vel_norm = rel_vel_body.norm();

    // 2. Predict drag acceleration in body frame
    float bcoef_inv_x = 1.0f / _bcoef_x;
    float bcoef_inv_y = 1.0f / _bcoef_y;

    // Drag model (EKF2 formulation)
    Vector2f a_drag_pred;
    a_drag_pred(0) = -0.5f * bcoef_inv_x * rho * rel_vel_body(0) * rel_vel_norm
                     - rel_vel_body(0) * _mcoef;
    a_drag_pred(1) = -0.5f * bcoef_inv_y * rho * rel_vel_body(1) * rel_vel_norm
                     - rel_vel_body(1) * _mcoef;

    // 3. Compute sliding surface (innovation)
    _sliding_surface = accel_body_xy - a_drag_pred;

    // 4. Super-Twisting observer dynamics
    Vector2f sigma_sqrt_sign;
    sigma_sqrt_sign(0) = sqrtf(fabsf(_sliding_surface(0))) *
                         smoothSign(_sliding_surface(0), _epsilon_smooth);
    sigma_sqrt_sign(1) = sqrtf(fabsf(_sliding_surface(1))) *
                         smoothSign(_sliding_surface(1), _epsilon_smooth);

    Vector2f sigma_sign;
    sigma_sign(0) = smoothSign(_sliding_surface(0), _epsilon_smooth);
    sigma_sign(1) = smoothSign(_sliding_surface(1), _epsilon_smooth);

    // 5. State update (Euler integration)
    Vector2f wind_dot = -_lambda1 * sigma_sqrt_sign + _aux_state;
    Vector2f aux_dot = -_lambda2 * sigma_sign;

    _wind_state += wind_dot * dt;
    _aux_state += aux_dot * dt;

    // 6. Constrain wind estimate
    _wind_state(0) = math::constrain(_wind_state(0), -30.f, 30.f);
    _wind_state(1) = math::constrain(_wind_state(1), -30.f, 30.f);

    // 7. Update variance and convergence
    updateVarianceEstimate(_sliding_surface, dt);
    updateConvergenceMetric(dt);
}
```

### Main Module Run Loop

```cpp
void WindEstimatorHosm::Run()
{
    perf_begin(_cycle_perf);

    // Exit check
    if (should_exit()) {
        _vehicle_imu_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    // Update parameters
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // Get IMU data (primary trigger at ~250 Hz)
    vehicle_imu_s imu{};
    if (!_vehicle_imu_sub.update(&imu)) {
        return;
    }

    // Downsample to ~100 Hz by accumulating
    const uint8_t imu_downsample_ratio = 2;  // 250 Hz / 2 = 125 Hz

    // Convert delta_velocity to acceleration
    float dt_imu = imu.delta_velocity_dt * 1e-6f;  // Convert to seconds
    Vector2f accel_xy(imu.delta_velocity[0] / dt_imu,
                      imu.delta_velocity[1] / dt_imu);

    _imu_accel_accum += accel_xy;
    _imu_sample_count++;

    if (_imu_sample_count < imu_downsample_ratio) {
        perf_end(_cycle_perf);
        return;
    }

    // Average accelerations
    Vector2f accel_avg = _imu_accel_accum / (float)_imu_sample_count;
    _imu_accel_accum.zero();
    _imu_sample_count = 0;

    // Compute dt for observer
    const float dt = (imu.timestamp - _timestamp_last) * 1e-6f;
    _timestamp_last = imu.timestamp;

    if (dt < 0.005f || dt > 0.1f) {
        perf_end(_cycle_perf);
        return;
    }

    // Check flight state
    vehicle_status_s vehicle_status{};
    _vehicle_status_sub.update(&vehicle_status);
    _armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

    vehicle_land_detected_s land_detected{};
    _vehicle_land_detected_sub.update(&land_detected);
    _in_air = !land_detected.landed;

    if (!_armed || !_in_air) {
        _valid_hysteresis.set_state_and_update(false, imu.timestamp);
        _valid = false;
        perf_end(_cycle_perf);
        return;
    }

    // Get GPS velocity (NED frame)
    vehicle_local_position_s local_pos{};
    bool gps_valid = false;
    Vector3f v_ground_ned(0.f, 0.f, 0.f);

    if (_vehicle_local_position_sub.update(&local_pos)) {
        if (local_pos.v_xy_valid && local_pos.v_z_valid) {
            v_ground_ned(0) = local_pos.vx;
            v_ground_ned(1) = local_pos.vy;
            v_ground_ned(2) = local_pos.vz;
            gps_valid = (imu.timestamp - local_pos.timestamp) < 200_ms;
        }
    }

    // Get attitude
    vehicle_attitude_s attitude{};
    bool attitude_valid = false;
    if (_vehicle_attitude_sub.update(&attitude)) {
        attitude_valid = (imu.timestamp - attitude.timestamp) < 20_ms;
    }

    // Get air density
    vehicle_air_data_s air_data{};
    float rho = 1.225f;  // Sea level default
    if (_vehicle_air_data_sub.update(&air_data)) {
        rho = air_data.rho;
    }

    // Run HOSM observer
    if (gps_valid && attitude_valid) {
        Quatf q_att(attitude.q);
        _hosm_observer.update(accel_avg, v_ground_ned, q_att, rho, dt);

        bool converged = _hosm_observer.isConverged();
        _valid_hysteresis.set_state_and_update(converged, imu.timestamp);
        _valid = _valid_hysteresis.get_state();

        publishWindEstimate(imu.timestamp);
    }

    perf_end(_cycle_perf);
}
```

## 4. Parameter Definitions

**wind_estimator_hosm_params.c**:
```c
/**
 * Enable HOSM Wind Estimator
 * @boolean
 * @reboot_required true
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_INT32(HOSM_WIND_ENABLE, 0);

/**
 * HOSM observer gain lambda1
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_WIND_GAIN_L, 2.5);

/**
 * HOSM observer gain lambda2
 * @min 0.5
 * @max 15.0
 * @decimal 2
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_WIND_GAIN_K, 1.8);

/**
 * HOSM alpha parameter for auto-tuning
 * @min 1.0
 * @max 5.0
 * @decimal 2
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_WIND_ALPHA, 2.0);

/**
 * HOSM beta parameter for auto-tuning
 * @min 1.0
 * @max 5.0
 * @decimal 2
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_WIND_BETA, 1.5);

/**
 * Minimum GPS velocity for HOSM update
 * @min 0.5
 * @max 5.0
 * @unit m/s
 * @decimal 1
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_MIN_GPS_VEL, 2.0);

/**
 * Bluff body drag coefficient X (inverse)
 * Should match EKF2_BCOEF_X for consistency
 * @min 5.0
 * @max 100.0
 * @unit m^2/kg
 * @decimal 1
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_BCOEF_X, 25.0);

/**
 * Bluff body drag coefficient Y (inverse)
 * Should match EKF2_BCOEF_Y for consistency
 * @min 5.0
 * @max 100.0
 * @unit m^2/kg
 * @decimal 1
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_BCOEF_Y, 25.0);

/**
 * Rotor momentum drag coefficient
 * Should match EKF2_MCOEF for consistency
 * @min 0.0
 * @max 1.0
 * @unit 1/s
 * @decimal 3
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_MCOEF, 0.1);

/**
 * Drag measurement noise variance
 * @min 0.1
 * @max 10.0
 * @unit (m/s^2)^2
 * @decimal 2
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_DRAG_NOISE, 1.0);

/**
 * Sign function smoothing parameter
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_SIGN_SMOOTH, 0.1);

/**
 * Lipschitz constant estimate for auto-tuning
 * @min 1.0
 * @max 20.0
 * @unit m/s^2
 * @decimal 1
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_FLOAT(HOSM_LIPSCHITZ, 8.0);

/**
 * Enable automatic gain tuning
 * @boolean
 * @group Wind Estimator HOSM
 */
PARAM_DEFINE_INT32(HOSM_AUTO_TUNE, 1);
```

## 5. Output and Integration

### Message Format
Reuse existing `Wind.msg` with multi-instance publication:

```cpp
void WindEstimatorHosm::publishWindEstimate(const hrt_abstime &timestamp_sample)
{
    wind_s wind{};

    wind.timestamp_sample = timestamp_sample;
    wind.timestamp = hrt_absolute_time();

    Vector2f wind_est = _hosm_observer.getWindEstimate();
    wind.windspeed_north = wind_est(0);
    wind.windspeed_east = wind_est(1);

    Vector2f wind_var = _hosm_observer.getWindVariance();
    wind.variance_north = wind_var(0);
    wind.variance_east = wind_var(1);

    // Drag-based method doesn't use airspeed/sideslip
    wind.tas_innov = NAN;
    wind.tas_innov_var = NAN;
    wind.beta_innov = NAN;
    wind.beta_innov_var = NAN;

    _wind_pub.publish(wind);
}
```

### Coexistence with EKF2
- EKF2 publishes to `wind` (instance 0)
- HOSM publishes to `wind_0` or `wind_1` (instance 1)
- Both run independently in parallel

### Startup Integration
Add to `/ROMFS/px4fmu_common/init.d/rc.mc_apps`:
```bash
if param compare -s HOSM_WIND_ENABLE 1
then
    wind_estimator_hosm start
fi
```

## 6. Build Configuration

**CMakeLists.txt**:
```cmake
px4_add_library(hosm_wind_observer
    hosm_wind_observer.cpp
    hosm_wind_observer.hpp
)

target_link_libraries(hosm_wind_observer
    PUBLIC
        mathlib
        matrix
)

px4_add_module(
    MODULE modules__wind_estimator_hosm
    MAIN wind_estimator_hosm
    COMPILE_FLAGS
        ${MAX_CUSTOM_OPT_LEVEL}
    SRCS
        WindEstimatorHosm.cpp
        WindEstimatorHosm.hpp
    DEPENDS
        hysteresis
        mathlib
        matrix
        px4_work_queue
        hosm_wind_observer
)

# Unit tests
px4_add_unit_gtest(
    SRC hosm_wind_observer_test.cpp
    LINKLIBS hosm_wind_observer
)
```

**Kconfig**:
```kconfig
menuconfig MODULES_WIND_ESTIMATOR_HOSM
    bool "wind_estimator_hosm"
    default n
    ---help---
        Enable HOSM-based drag wind estimation for multirotors
```

## 7. Testing and Validation

### Unit Tests
Test the HOSM observer with simulated drag measurements:
```cpp
TEST(HosmWindObserver, ConvergenceWithConstantWind)
{
    HosmWindObserver obs;
    obs.setGains(2.5f, 1.8f);
    obs.setDragCoefficients(25.0f, 25.0f, 0.1f);

    // Simulate 5 m/s north wind
    float wind_true_n = 5.0f;
    float wind_true_e = 0.0f;

    // Vehicle moving north at 10 m/s ground speed
    Vector3f v_ground_ned(10.0f, 0.0f, 0.0f);
    Quatf q_att(1.f, 0.f, 0.f, 0.f);  // Level, north heading
    float rho = 1.225f;

    // Relative velocity: v_rel = v_ground - wind = 5 m/s north
    // Compute expected drag acceleration...

    for (int i = 0; i < 2000; i++) {
        // Simulate drag acceleration measurement
        Vector3f v_rel_ned(5.0f, 0.f, 0.f);
        Vector3f v_rel_body = q_att.rotateVectorInverse(v_rel_ned);
        float rel_norm = v_rel_body.norm();

        Vector2f accel_drag;
        accel_drag(0) = -0.5f * (1.f/25.f) * rho * v_rel_body(0) * rel_norm
                        - v_rel_body(0) * 0.1f;
        accel_drag(1) = -0.5f * (1.f/25.f) * rho * v_rel_body(1) * rel_norm
                        - v_rel_body(1) * 0.1f;

        obs.update(accel_drag, v_ground_ned, q_att, rho, 0.01f);
    }

    Vector2f wind = obs.getWindEstimate();
    EXPECT_NEAR(wind(0), wind_true_n, 1.0f);
    EXPECT_NEAR(wind(1), wind_true_e, 0.5f);
}
```

### SITL Testing
1. Enable Gazebo wind plugin
2. Fly multirotor with various speeds and headings
3. Compare HOSM vs EKF2 wind estimates
4. Log: `wind`, `wind_0`, `vehicle_imu`, `vehicle_local_position`

### Tuning Guidelines
**Initial setup**:
```
HOSM_WIND_ENABLE = 1
HOSM_BCOEF_X = 25.0    # Match EKF2_BCOEF_X
HOSM_BCOEF_Y = 25.0    # Match EKF2_BCOEF_Y
HOSM_MCOEF = 0.1       # Match EKF2_MCOEF
HOSM_AUTO_TUNE = 1
HOSM_LIPSCHITZ = 8.0
```

**If convergence is slow**: Increase `HOSM_WIND_ALPHA`, `HOSM_LIPSCHITZ`
**If estimate oscillates**: Increase `HOSM_SIGN_SMOOTH`, decrease gains
**If biased**: Check drag coefficient calibration (match EKF2 params)

## 8. Critical Files Summary

Implementation will primarily reference:
- `/home/riccardo/px4/my-px4/src/modules/ekf2/EKF/aid_sources/drag/drag_fusion.cpp` - Drag model equations
- `/home/riccardo/px4/my-px4/src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp` - Module structure pattern
- `/home/riccardo/px4/my-px4/msg/Wind.msg` - Output message format
- `/home/riccardo/px4/my-px4/msg/VehicleImu.msg` - IMU input data

## 9. Key Advantages of Drag-Based HOSM

1. **No airspeed sensor required** - suitable for all multirotors
2. **High update rate** (50-100 Hz from IMU) - faster than GPS-based methods
3. **Finite-time convergence** - HOSM guarantees faster settling than asymptotic observers
4. **Robust to modeling errors** - sliding mode inherently robust to bounded disturbances
5. **Complementary to EKF2** - different sensor fusion approach provides independent estimate

## 10. Verification Checklist

End-to-end testing steps:
- [ ] Build module successfully
- [ ] Module starts without errors
- [ ] IMU data received at ~100 Hz
- [ ] GPS velocity valid and recent
- [ ] Drag parameters match vehicle configuration
- [ ] Wind estimate converges in 10-20 seconds
- [ ] Steady-state error < 2 m/s compared to EKF2
- [ ] CPU load < 1%
- [ ] No crashes or instability during flight
