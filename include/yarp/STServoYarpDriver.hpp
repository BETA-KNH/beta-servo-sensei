#ifndef ST_SERVO_YARP_DRIVER_HPP
#define ST_SERVO_YARP_DRIVER_HPP

/**
 * @file STServoYarpDriver.hpp
 * @brief YARP device plugin that exposes an ST bus servo chain as a
 *        standard YARP motor-control device.
 *
 * Supported YARP interfaces
 * --------------------------------------------------------------------------
 * | Interface              | Servo feature                                 |
 * |------------------------|-----------------------------------------------|
 * | IPositionControl       | TARGET_LOCATION + OPERATION_SPEED             |
 * | IVelocityControl       | OPERATION_MODE=1 + OPERATION_SPEED            |
 * | ITorqueControl         | TORQUE_LIMIT (clamping — no torque sensor)    |
 * | IEncodersTimed         | CURRENT_LOCATION / CURRENT_SPEED (read-back)  |
 * | IMotorEncoders         | Same as IEncoders (direct-drive, no gearbox)  |
 * | IControlMode           | OPERATION_MODE register                       |
 * | IAmplifierControl       | TORQUE_SWITCH / CURRENT_CURRENT / TORQUE_LIMIT|
 * | IPidControl            | P/I/D_COEFF + SPEED_P_COEFF / VELOCITY_I_COEFF|
 *
 * Unit conventions (public-facing)
 * ---------------------------------
 * - Position  : degrees  (internal conversion: step = deg × 4095 / 360)
 * - Speed     : deg/s
 * - Acceleration: deg/s²
 * - Current   : Amperes  (register unit: ×6.5 mA)
 * - Voltage   : Volts    (register unit: ×0.1 V)
 * - Torque    : % of rated torque (no Nm calibration available)
 *
 * Configuration keys (yarp::os::Searchable / yarpdev CLI)
 * --------------------------------------------------------
 * | Key         | Type    | Required | Description                        |
 * |-------------|---------|----------|------------------------------------|
 * | port        | string  | yes      | Serial device path (/dev/ttyUSB0)  |
 * | baud        | int     | yes      | Bus baud rate (e.g. 1000000)       |
 * | servo_ids   | list    | yes      | Ordered servo IDs, e.g "(1 2 3 4)" |
 * | timeout_ms  | int     | no       | Read timeout, default 50 ms        |
 * | auto_response_level | bool | no  | Set RESPONSE_LEVEL=1 on servos     |
 * |                    |      |         | that don't reply to READ (factory  |
 * |                    |      |         | default is ping-only). Default off |
 *
 * @note ITorqueControl is approximated: setRefTorque writes TORQUE_LIMIT
 *       (a maximum clamping value, not a closed-loop demand). getTorque
 *       returns the drive duty-cycle (CURRENT_LOAD) as a torque proxy.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include "serial/SerialPortDriver.hpp"
#include "serial/SerialPortConfig.hpp"
#include "servo/STServoRegisters.hpp"
#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"

// ---------------------------------------------------------------------------

class STServoYarpDriver
    : public yarp::dev::DeviceDriver
    , public yarp::dev::IPositionControl
    , public yarp::dev::IVelocityControl
    , public yarp::dev::ITorqueControl
    , public yarp::dev::IEncodersTimed
    , public yarp::dev::IMotorEncoders
    , public yarp::dev::IControlMode
    , public yarp::dev::IAmplifierControl
    , public yarp::dev::IPidControl
    , public yarp::dev::IInteractionMode
{
public:
    STServoYarpDriver()          = default;
    ~STServoYarpDriver() override = default;

    // -----------------------------------------------------------------------
    // DeviceDriver
    // -----------------------------------------------------------------------
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // -----------------------------------------------------------------------
    // IPositionControl
    // -----------------------------------------------------------------------
    bool getAxes(int* ax) override;

    bool positionMove(int j, double ref) override;
    bool positionMove(const double* refs) override;
    bool positionMove(int n_joint, const int* joints, const double* refs) override;

    bool relativeMove(int j, double delta) override;
    bool relativeMove(const double* deltas) override;
    bool relativeMove(int n_joint, const int* joints, const double* deltas) override;

    bool checkMotionDone(int j, bool* flag) override;
    bool checkMotionDone(bool* flag) override;
    bool checkMotionDone(int n_joint, const int* joints, bool* flag) override;

    bool setRefSpeed(int j, double sp) override;
    bool setRefSpeeds(const double* spds) override;
    bool setRefSpeeds(int n_joint, const int* joints, const double* spds) override;

    bool setRefAcceleration(int j, double acc) override;
    bool setRefAccelerations(const double* accs) override;
    bool setRefAccelerations(int n_joint, const int* joints, const double* accs) override;

    bool getRefSpeed(int j, double* ref) override;
    bool getRefSpeeds(double* spds) override;
    bool getRefSpeeds(int n_joint, const int* joints, double* spds) override;

    bool getRefAcceleration(int j, double* acc) override;
    bool getRefAccelerations(double* accs) override;
    bool getRefAccelerations(int n_joint, const int* joints, double* accs) override;

    bool stop(int j) override;
    bool stop() override;
    bool stop(int n_joint, const int* joints) override;

    // -----------------------------------------------------------------------
    // IVelocityControl  (getAxes / setRefAcceleration* / getRefAcceleration*
    //                    / stop overloads already declared above)
    // -----------------------------------------------------------------------
    bool velocityMove(int j, double sp) override;
    bool velocityMove(const double* sp) override;
    bool velocityMove(int n_joint, const int* joints, const double* spds) override;

    // -----------------------------------------------------------------------
    // ITorqueControl  (getAxes already declared above)
    // -----------------------------------------------------------------------
    bool getRefTorques(double* t) override;
    bool getRefTorque(int j, double* t) override;
    bool setRefTorques(const double* t) override;
    bool setRefTorque(int j, double t) override;
    bool getTorque(int j, double* t) override;
    bool getTorques(double* t) override;
    bool getTorqueRange(int j, double* min, double* max) override;
    bool getTorqueRanges(double* min, double* max) override;

    // -----------------------------------------------------------------------
    // IEncodersTimed  (IEncoders + timestamps, getAxes already declared above)
    // -----------------------------------------------------------------------
    bool resetEncoder(int j) override;
    bool resetEncoders() override;
    bool setEncoder(int j, double val) override;
    bool setEncoders(const double* vals) override;
    bool getEncoder(int j, double* v) override;
    bool getEncoders(double* encs) override;
    bool getEncoderSpeed(int j, double* sp) override;
    bool getEncoderSpeeds(double* spds) override;
    bool getEncoderAcceleration(int j, double* acc) override;
    bool getEncoderAccelerations(double* accs) override;
    /// @brief IEncodersTimed — returns current position and timestamp.
    bool getEncoderTimed(int j, double* v, double* t) override;
    /// @brief IEncodersTimed — returns all positions and per-joint timestamps.
    bool getEncodersTimed(double* encs, double* ts) override;

    // -----------------------------------------------------------------------
    // IMotorEncoders
    // -----------------------------------------------------------------------
    bool getNumberOfMotorEncoders(int* num) override;
    bool resetMotorEncoder(int m) override;
    bool resetMotorEncoders() override;
    bool setMotorEncoderCountsPerRevolution(int m, const double cpr) override;
    bool getMotorEncoderCountsPerRevolution(int m, double* cpr) override;
    bool setMotorEncoder(int m, double val) override;
    bool setMotorEncoders(const double* vals) override;
    bool getMotorEncoder(int m, double* v) override;
    bool getMotorEncoders(double* encs) override;
    bool getMotorEncodersTimed(double* encs, double* times) override;
    bool getMotorEncoderTimed(int m, double* enc, double* time) override;
    bool getMotorEncoderSpeed(int m, double* sp) override;
    bool getMotorEncoderSpeeds(double* spds) override;
    bool getMotorEncoderAcceleration(int m, double* acc) override;
    bool getMotorEncoderAccelerations(double* accs) override;

    // -----------------------------------------------------------------------
    // IControlMode
    // -----------------------------------------------------------------------
    bool getControlMode(int j, int* mode) override;
    bool getControlModes(int* modes) override;
    bool getControlModes(int n_joint, const int* joints, int* modes) override;
    bool setControlMode(const int j, const int mode) override;
    bool setControlModes(int n_joint, const int* joints, int* modes) override;
    bool setControlModes(int* modes) override;

    // -----------------------------------------------------------------------
    // IInteractionMode  (all servo joints are stiff-only; set is a no-op)
    // -----------------------------------------------------------------------
    bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    bool getInteractionModes(int n_joints, int* joints,
                             yarp::dev::InteractionModeEnum* modes) override;
    bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    bool setInteractionModes(int n_joints, int* joints,
                             yarp::dev::InteractionModeEnum* modes) override;
    bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

    // -----------------------------------------------------------------------
    // IPositionControl — extra overrides for yarpmotorgui
    // -----------------------------------------------------------------------
    bool getTargetPosition(const int joint, double* ref) override;
    bool getTargetPositions(double* refs) override;
    bool getTargetPositions(const int n_joint, const int* joints, double* refs) override;

    // -----------------------------------------------------------------------
    // IVelocityControl — extra overrides for yarpmotorgui
    // -----------------------------------------------------------------------
    bool getRefVelocity(const int joint, double* vel) override;
    bool getRefVelocities(double* vels) override;
    bool getRefVelocities(const int n_joint, const int* joints, double* vels) override;

    // -----------------------------------------------------------------------
    // IAmplifierControl
    // -----------------------------------------------------------------------
    bool enableAmp(int j) override;
    bool disableAmp(int j) override;
    bool getAmpStatus(int* st) override;
    bool getAmpStatus(int j, int* v) override;
    bool getCurrents(double* vals) override;
    bool getCurrent(int j, double* val) override;
    bool getMaxCurrent(int j, double* v) override;
    bool setMaxCurrent(int j, double v) override;
    bool getPowerSupplyVoltage(int j, double* v) override;

    // -----------------------------------------------------------------------
    // IPidControl
    // -----------------------------------------------------------------------
    bool setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j,
                const yarp::dev::Pid& pid) override;
    bool setPids(const yarp::dev::PidControlTypeEnum& pidtype,
                 const yarp::dev::Pid* pids) override;
    bool setPidReference(const yarp::dev::PidControlTypeEnum& pidtype,
                         int j, double ref) override;
    bool setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype,
                          const double* refs) override;
    bool setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype,
                          int j, double limit) override;
    bool setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype,
                           const double* limits) override;
    bool getPidError(const yarp::dev::PidControlTypeEnum& pidtype,
                     int j, double* err) override;
    bool getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype,
                      double* errs) override;
    bool getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype,
                      int j, double* out) override;
    bool getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype,
                       double* outs) override;
    bool getPid(const yarp::dev::PidControlTypeEnum& pidtype,
                int j, yarp::dev::Pid* pid) override;
    bool getPids(const yarp::dev::PidControlTypeEnum& pidtype,
                 yarp::dev::Pid* pids) override;
    bool getPidReference(const yarp::dev::PidControlTypeEnum& pidtype,
                         int j, double* ref) override;
    bool getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype,
                          double* refs) override;
    bool getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype,
                          int j, double* limit) override;
    bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype,
                           double* limits) override;
    bool resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;
    bool disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;
    bool enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j) override;
    bool setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype,
                      int j, double v) override;
    bool isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype,
                      int j, bool* enabled) override;

private:
    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    std::unique_ptr<SerialPortDriver> serial_;
    mutable std::mutex                mutex_;  ///< Serialises all half-duplex UART transactions.

    std::vector<uint8_t> servoIds_;            ///< Servo IDs in joint-index order.
    int                  nJoints_{0};

    // Per-joint cached references (in YARP units: deg, deg/s, deg/s²)
    std::vector<double> refSpeeds_;         ///< Default: 60 deg/s per joint.
    std::vector<double> refAccelerations_;  ///< Default: 0 (servo max).
    std::vector<double> refTorques_;        ///< % of rated torque.
    std::vector<int>    controlModes_;      ///< Current YARP VOCAB_CM_* per joint.
    std::vector<double> encoderOffsets_;    ///< Software zero offset in degrees.
    std::vector<bool>   pidEnabled_;        ///< PID enabled state per joint.
    std::vector<double> refPositions_;      ///< Last commanded target position (deg) per joint.
    std::vector<yarp::dev::InteractionModeEnum> interactionModes_; ///< Always VOCAB_IM_STIFF.

    // Velocity estimation (for getEncoderAcceleration)
    std::vector<double> prevSpeedDegS_;
    std::vector<double> prevSpeedTime_;

    // -----------------------------------------------------------------------
    // Unit conversion constants
    // -----------------------------------------------------------------------
    static constexpr double STEPS_PER_DEG  = 4095.0 / 360.0;
    static constexpr double DEG_PER_STEP   = 360.0  / 4095.0;
    static constexpr double MOTOR_CPR      = 4096.0;   ///< Counts per revolution.
    static constexpr double CURRENT_SCALE  = 0.0065;   ///< A per count (6.5 mA).
    static constexpr double VOLTAGE_SCALE  = 0.1;      ///< V per count.
    static constexpr double LOAD_SCALE     = 0.1;      ///< % per count.
    static constexpr double ACC_UNIT       = 100.0;    ///< step/s² per register unit.

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// @brief Clamp and round degrees to a signed 16-bit step count.
    static int16_t degToSteps(double deg)
    {
        return static_cast<int16_t>(
            std::clamp(std::round(deg * STEPS_PER_DEG),
                       static_cast<double>(INT16_MIN),
                       static_cast<double>(INT16_MAX)));
    }

    /// @brief Convert a signed step count to degrees.
    static double stepsToDeg(int16_t s) { return s * DEG_PER_STEP; }

    /// @brief Convert deg/s to raw step/s (uint16, clamped to 0-65535).
    static uint16_t degSToStepS(double dps)
    {
        return static_cast<uint16_t>(
            std::clamp(std::round(std::abs(dps) * STEPS_PER_DEG), 0.0, 65535.0));
    }

    /// @brief Convert raw step/s to deg/s.
    static double stepSToDegS(uint16_t s) { return s * DEG_PER_STEP; }

    /// @brief True if joint index is valid.
    bool validJoint(int j) const { return j >= 0 && j < nJoints_; }

    /// @brief Send a packet and receive one response of the expected size.
    ///        Caller must NOT already hold mutex_.
    STServo::ServoResponse transact(const std::vector<uint8_t>& packet,
                                    std::size_t expectedBytes) const;

    /// @brief Read a 1-byte register from a single servo.
    std::optional<uint8_t>  readByte  (uint8_t id, const STServo::Reg& reg) const;

    /// @brief Read a 2-byte (LE) register from a single servo.
    std::optional<uint16_t> readUint16(uint8_t id, const STServo::Reg& reg) const;

    /// @brief Write a 1-byte value to a single servo register.
    bool writeByte  (uint8_t id, const STServo::Reg& reg, uint8_t value);

    /// @brief Write a 2-byte LE value to a single servo register.
    bool writeUint16(uint8_t id, const STServo::Reg& reg, uint16_t value);

    /// @brief Write OPERATION_MODE (EEPROM) with the required LOCK_FLAG cycle.
    bool writeOperationMode(uint8_t id, uint8_t mode);

    /// @brief Command a position+speed move to one joint (does not lock mutex).
    ///        Called with mutex_ already held when used inside a SYNC path.
    bool doMoveOne(int j, int16_t steps, uint16_t speedSteps);

    /// @brief SYNC_WRITE a move to a subset of joints.
    bool doSyncMove(const std::vector<int>& jointIdxs,
                    const std::vector<int16_t>& stepsVec,
                    const std::vector<uint16_t>& speedVec);

    /// @brief SYNC_READ a 2-byte register from all servos into a vector.
    bool syncReadUint16(const STServo::Reg& reg, std::vector<uint16_t>& out) const;

    /// @brief SYNC_READ a 1-byte register from all servos into a vector.
    bool syncReadByte(const STServo::Reg& reg, std::vector<uint8_t>& out) const;

    /// @brief Build and encode a 6-byte move payload
    ///        [TARGET_LOCATION(2) | OPERATION_TIME(2)=0 | OPERATION_SPEED(2)].
    static std::vector<uint8_t> encodeMovePayload(int16_t steps, uint16_t speedSteps)
    {
        return {
            static_cast<uint8_t>(steps      & 0xFF),
            static_cast<uint8_t>((steps >> 8) & 0xFF),
            0x00, 0x00,   // OPERATION_TIME — always 0 for ST3215
            static_cast<uint8_t>(speedSteps      & 0xFF),
            static_cast<uint8_t>((speedSteps >> 8) & 0xFF),
        };
    }
};

#endif // ST_SERVO_YARP_DRIVER_HPP
