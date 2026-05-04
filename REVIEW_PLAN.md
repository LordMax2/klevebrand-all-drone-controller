# Naming & Formatting Review Plan

All items complete.

## Naming Conventions

- [x] **#1 — `start_micros_timestamp` uses abbreviation `micros`**
  Renamed to `start_microseconds_timestamp` in `base_drone.h` and `base_drone.cpp`.
  Local variables `current_micros_timestamp`, `expected_loop_duration_micros`, `elapsed_micros` also renamed accordingly.

---

## Formatting — Missing blank lines between function bodies

- [x] **#2 — `src/base_drone.cpp`**
  Added blank lines between logical groups in the constructor body.
  Normalised brace style on `getFlightMode`, `timestampMilliseconds`, `timestampMicroseconds`, `getFlightModeType`, `getFeedbackLoopHz`.

- [x] **#3 — `src/base_drone_gyro.cpp`** *(already compliant)*

- [x] **#4 — `src/base_drone_motor.cpp`**
  Expanded inline `{}` to proper multi-line body.

- [x] **#5 — `src/base_drone_position.cpp`** *(already compliant)*

- [x] **#6 — `src/base_hardware_processor.cpp`** *(already compliant)*

- [x] **#7 — `src/base_pid_repository.cpp`** *(already compliant)*

- [x] **#8 — `src/gyro_pid.cpp`** *(already compliant)*

- [x] **#9 — `src/pid.cpp`** *(already compliant)*

- [x] **#10 — `src/pid_optimizer.cpp`** *(already compliant)*

- [x] **#11 — `src/pid_yaw_compass.cpp`** *(already compliant)*

- [x] **#12 — `include/template_drone.ipp`** *(already compliant)*

- [x] **#13 — Inline functions in headers**
  `include/pid_optimizer.h` — added blank lines between all inline one-liner getters and setters.
  All other headers (`base_flight_mode.h`, `flight_mode_acro.h`, `flight_mode_auto_level.h`, `gyro_pid.h`) were already compliant.
