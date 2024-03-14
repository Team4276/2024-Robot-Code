package frc.team4276.frc2024.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.drivers.FourBarFeedForward;
import frc.team4276.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.team4276.lib.revlib.VIKCANSparkServoMotor;
import frc.team4276.lib.revlib.VIKCANSparkServoMotor.Direction;
import frc.team4276.frc2024.Logging.PrintLogger;

import frc.team254.lib.util.Util;

//TODO: abstract this
public class SimpleFourbarSubsystem extends Subsystem {
    private VIKCANSparkServoMotor mMaster;

    private AbsoluteEncoder mAbsoluteEncoder;

    private PeriodicIO mPeriodicIO;

    private ControlState mControlState = ControlState.IDLE;

    public enum ControlState {
        IDLE,
        VOLTAGE,
        SMART_MOTION,
        CALIBRATING

    }

    private State mStateSetpoint = new State(SuperstructureConstants.kFourbarStowState, 0.0);
    private double mProfileStartTime = 0.0;

    private FourBarFeedForward mFourbarFF;
    private TrapezoidProfile mTrapezoidProfile;
    private ProfiledPIDController mProfiledPIDController;

    private SparkPIDController mSparkPIDController;

    private double kMaxPosition = Double.POSITIVE_INFINITY;
    private double kMinPosition = Double.NEGATIVE_INFINITY;

    private boolean isMaintain = false;

    private static SimpleFourbarSubsystem mInstance;

    public static SimpleFourbarSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SimpleFourbarSubsystem(SuperstructureConstants.kFourBarConstants);
        }
        return mInstance;
    }

    private SimpleFourbarSubsystem(ServoMotorSubsystemConstants constants) {
        mMaster = new VIKCANSparkServoMotor(constants.kMasterConstants.id, MotorType.kBrushless);
        mMaster.restoreFactoryDefaults();
        mMaster.enableVoltageCompensation(constants.kVoltageCompensation);
        mMaster.setSmartCurrentLimit(constants.kSmartCurrentLimit);
        mMaster.setIdleMode(constants.kIdleMode);
        // mMaster.setCANTimeout(10);
        // mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        mMaster.setInverted(constants.kMasterConstants.isInverted);
        // mMaster.enableForwardLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen, true);
        // mMaster.enableReverseLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen, false);
        mMaster.setEncoderLimit(kMinPosition, Direction.NEGATIVE, false, 0.1);
        mMaster.setEncoderLimit(kMaxPosition, Direction.POSITIVE, true, 0.1);

        mAbsoluteEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);
        mAbsoluteEncoder.setPositionConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setInverted(constants.kIsInverted);
        mAbsoluteEncoder.setZeroOffset(constants.kOffset);
        mMaster.setEncoderForLimits(mAbsoluteEncoder::getPosition);

        mSparkPIDController = mMaster.getPIDController();
        mSparkPIDController.setFeedbackDevice(mAbsoluteEncoder);
        mSparkPIDController.setP(constants.kP, 0);
        mSparkPIDController.setI(constants.kI, 0);
        mSparkPIDController.setD(constants.kD, 0);
        mSparkPIDController.setFF(constants.kFF, 0);
        mSparkPIDController.setDFilter(constants.kDFilter, 0);
        mSparkPIDController.setIZone(constants.kIZone, 0);
        mSparkPIDController.setIMaxAccum(constants.kIMaxAccum, 0);
        mSparkPIDController.setOutputRange(-constants.kPIDOutputRange, constants.kPIDOutputRange, 0);
        if (constants.kIsCircular) {
            mSparkPIDController.setPositionPIDWrappingEnabled(constants.kIsCircular);
            mSparkPIDController.setPositionPIDWrappingMaxInput(constants.kMaxPosition);
            mSparkPIDController.setPositionPIDWrappingMinInput(constants.kMaxPosition);
        }

        mFourbarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
        mTrapezoidProfile = new TrapezoidProfile(new Constraints(constants.kMaxSpeed, constants.kMaxAccel));
        mProfiledPIDController = new ProfiledPIDController(0.0, 0.0, 0.0,
                new Constraints(constants.kMaxSpeed, constants.kMaxAccel), Constants.kLooperDt);
        mProfiledPIDController.setTolerance(constants.kPosTol, constants.kVelTol);

        this.kMaxPosition = constants.kMaxPosition;
        this.kMinPosition = constants.kMinPosition;

        mMaster.burnFlash();

        mPeriodicIO = new PeriodicIO();

        SmartDashboard.putNumber("Calibration Velocity Test", 0.0);
        SmartDashboard.putNumber("Calibration Voltage Test", 0.0);
        SmartDashboard.putNumber("Calibration Efficiency Test", 0.0);
        SmartDashboard.putNumber("Calibration Static Test", 0.0);
        SmartDashboard.putNumber("Calibration Accel Test", 0.0);
    }

    public ControlState getControlState() {
        return mControlState == null ? ControlState.IDLE : mControlState;
    }

    public void idle() {
        if (mControlState != ControlState.IDLE) {
            mControlState = ControlState.IDLE;
        }

    }

    public void setVoltage(double volts) {
        if (mControlState != ControlState.VOLTAGE) {
            mControlState = ControlState.VOLTAGE;
        }

        mPeriodicIO.demand = volts;
    }

    public void setSmartMotionSetpoint(double position_radians) {
        if (mControlState != ControlState.SMART_MOTION) {
            mControlState = ControlState.SMART_MOTION;
        }

        if (mStateSetpoint.position == position_radians)
            return;

        mProfileStartTime = mPeriodicIO.timestamp;

        mStateSetpoint = new State(Util.limit(position_radians, kMinPosition, kMaxPosition), 0.0);
        isMaintain = false;
        mProfiledPIDController.reset(mPeriodicIO.meas_state);
    }

    public void setCalibrating() {
        if (mControlState != ControlState.CALIBRATING) {
            mControlState = ControlState.CALIBRATING;
        }
    }

    public void setIdleMode(IdleMode idleMode) {
        if (idleMode == mMaster.getIdleMode())
            return;

        mMaster.setIdleMode(idleMode);
    }

    public IdleMode getIdleMode() {
        return mMaster.getIdleMode();
    }

    private class PeriodicIO {
        // Inputs
        double timestamp;
        double meas_position_units;
        double meas_velocity_units;
        double meas_acceleration_units;
        State meas_state;
        double meas_voltage;

        double velocity_test;
        double voltage_test;
        double efficiency_test;
        double static_test;
        double accel_test;

        // Outputs
        double feed_forward;
        double demand;
    }

    private double prev_timestamp;
    private double prev_meas_velocity_units = 0.0;

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.meas_position_units = mAbsoluteEncoder.getPosition();
        mPeriodicIO.meas_velocity_units = mAbsoluteEncoder.getVelocity();
        mPeriodicIO.meas_state = new State(mPeriodicIO.meas_position_units, mPeriodicIO.meas_velocity_units);
        mPeriodicIO.meas_voltage = mMaster.getAppliedOutput() * 12.0;
        mPeriodicIO.meas_acceleration_units = (mPeriodicIO.meas_velocity_units - prev_meas_velocity_units)
                / (mPeriodicIO.timestamp - prev_timestamp);

        prev_timestamp = mPeriodicIO.timestamp;
        prev_meas_velocity_units = mPeriodicIO.meas_velocity_units;

    }

    private State prev_des_state = new State();
    private double prev_des_accel = 0.0;

    @Override
    public void writePeriodicOutputs() {
        switch (mControlState) {
            case IDLE:
                mMaster.setVoltage(0.0);

                break;

            case VOLTAGE:
                mMaster.updateEncoderForLimits(mPeriodicIO.meas_position_units);
                mMaster.setVoltage(mPeriodicIO.demand);

                break;

            case SMART_MOTION:
                SmartDashboard.putNumber("Time Since Start", mPeriodicIO.timestamp - mProfileStartTime);
                SmartDashboard.putNumber("Input Measured State Position", mPeriodicIO.meas_state.position);
                SmartDashboard.putNumber("Input Measured State Velocity", mPeriodicIO.meas_state.velocity);
                SmartDashboard.putNumber("Input Setpoint State Position", mStateSetpoint.position);
                SmartDashboard.putNumber("Input Setpoint State Velocity", mStateSetpoint.velocity);

                mPeriodicIO.accel_test = SmartDashboard.getNumber("Calibration Accel Test", 0.0);

                if (!Util.epsilonEquals(prev_des_state.position, mPeriodicIO.meas_position_units, Math.PI / 40)
                        || !Util.epsilonEquals(prev_des_state.velocity, mPeriodicIO.meas_velocity_units,
                                Math.PI / 40)) {
                    mProfileStartTime = mPeriodicIO.timestamp;
                }

                State state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp - mProfileStartTime,
                        mPeriodicIO.meas_state,
                        mStateSetpoint);

                SmartDashboard.putNumber("Fourbar Trapezoid State Position", state.position);
                SmartDashboard.putNumber("Fourbar Trapezoid State Velocity", state.velocity);

                mPeriodicIO.feed_forward = mFourbarFF.calculate(state.position, state.velocity);

                mPeriodicIO.feed_forward += (mPeriodicIO.accel_test * (state.velocity - mPeriodicIO.meas_velocity_units)
                        / Constants.kLooperDt);

                SmartDashboard.putNumber("Accel Error", mPeriodicIO.meas_acceleration_units - prev_des_accel);
                prev_des_accel = (state.velocity - mPeriodicIO.meas_velocity_units) / Constants.kLooperDt;

                SmartDashboard.putNumber("Velocity Error", mPeriodicIO.meas_velocity_units - prev_des_state.velocity);
                prev_des_state = state;

                SmartDashboard.putNumber("Fourbar Feedforward Voltage", mPeriodicIO.feed_forward);

                SmartDashboard.putBoolean("Is Fourbar FInished",
                        mTrapezoidProfile.isFinished(mPeriodicIO.timestamp - mProfileStartTime));

                // if (Math.abs(mPeriodicIO.meas_position_units - mStateSetpoint.position) < (Math.PI / 50)
                //         || isMaintain) {
                //     isMaintain = true;
                //     mSparkPIDController.setReference(mStateSetpoint.position, ControlType.kPosition, 0,
                //             mFourbarFF.calculate(mStateSetpoint.position, 0.0), ArbFFUnits.kVoltage);
                //     return;

                // }

                
                mMaster.updateEncoderForLimits(mPeriodicIO.meas_position_units);
                mMaster.setVoltage(mPeriodicIO.feed_forward);

                break;

            case CALIBRATING:
                SmartDashboard.putNumber("Time Since Start", mPeriodicIO.timestamp - mProfileStartTime);
                SmartDashboard.putNumber("Input Measured State Position", mPeriodicIO.meas_state.position);
                SmartDashboard.putNumber("Input Measured State Velocity", mPeriodicIO.meas_state.velocity);
                SmartDashboard.putNumber("Input Setpoint State Position", mStateSetpoint.position);
                SmartDashboard.putNumber("Input Setpoint State Velocity", mStateSetpoint.velocity);

                mPeriodicIO.velocity_test = SmartDashboard.getNumber("Calibration Velocity Test", 0.0);
                mPeriodicIO.voltage_test = SmartDashboard.getNumber("Calibration Voltage Test", 0.0);
                mPeriodicIO.efficiency_test = SmartDashboard.getNumber("Calibration Efficiency Test", 0.0);
                mPeriodicIO.static_test = SmartDashboard.getNumber("Calibration Static Test", 0.0);
                mPeriodicIO.accel_test = SmartDashboard.getNumber("Calibration Accel Test", 0.0);

                mFourbarFF.setEfficiency(mPeriodicIO.efficiency_test);
                mFourbarFF.setkS(mPeriodicIO.static_test);

                mPeriodicIO.feed_forward = mPeriodicIO.voltage_test == 0.0
                        ? mFourbarFF.calculate(mPeriodicIO.meas_position_units,
                                mPeriodicIO.velocity_test)
                        : mPeriodicIO.voltage_test;

                SmartDashboard.putNumber("Velocity Error", mPeriodicIO.meas_velocity_units - prev_des_state.velocity);
                prev_des_state.velocity = mPeriodicIO.velocity_test;
                
                mMaster.updateEncoderForLimits(mPeriodicIO.meas_position_units);
                mMaster.setVoltage(mPeriodicIO.feed_forward);
                SmartDashboard.putNumber("Fourbar Feedforward Voltage", mPeriodicIO.feed_forward);

                break;

            default:
                break;
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Fourbar Applied Voltage", mPeriodicIO.meas_voltage);
        SmartDashboard.putBoolean("Front Limit", mMaster.isReverseLimitPressed());
        SmartDashboard.putBoolean("Back Limit", mMaster.isForwardLimitPressed());
        SmartDashboard.putString("Simple Fourbar COntrolstate", mControlState.name());
        SmartDashboard.putNumber("Fourbar Position Degrees", Math.toDegrees(mPeriodicIO.meas_position_units));
        SmartDashboard.putNumber("Fourbar Acceleration", mPeriodicIO.meas_acceleration_units);

    }

}
