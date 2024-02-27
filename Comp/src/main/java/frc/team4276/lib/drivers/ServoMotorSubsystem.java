package frc.team4276.lib.drivers;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
// import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team254.lib.util.Util;

import frc.team4276.lib.drivers.FourBarFeedForward.FourBarFeedForwardConstants;

// Subsystem class for NEO v1.1 Brushless motors

//TODO: add soft limits to everything;

public abstract class ServoMotorSubsystem extends Subsystem {
    private final CANSparkMax mMaster;
    private final CANSparkMax[] mFollowers;

    private final AbsoluteEncoder mAbsoluteEncoder;
    // private final RelativeEncoder mRelativeEncoder;

    private PeriodicIO mPeriodicIO;

    private ControlState mControlState;

    private SparkPIDController mPIDController;

    private final FourBarFeedForward mFourBarFF;
    private TrapezoidProfile mTrapezoidProfile;

    private State mStateSetpoint;
    private double mProfileStartTime;

    private ServoMotorSubsystemConstants constants;

    public static class ServoMotorConstants {
        public int id;
        public boolean isInverted; // TODO: check if relative to the master
    }

    public static class ServoMotorSubsystemConstants {
        public ServoMotorConstants kMasterConstants = new ServoMotorConstants();
        public ServoMotorConstants[] kFollowerConstants = new ServoMotorConstants[0];

        public boolean kIsInverted = false; // Encoder
        public boolean kIsCircular = false;
        public double kUnitsPerRotation = 1.0; // Overall Rotation
        public double kGearRatio = 1.0; // Motor Rotations to Overall Rotations
        public double kOffset = 0.0; // Set in hardware client
        public double kHomePosition = 0.0;
        public double kMinPosition = Double.NEGATIVE_INFINITY;
        public double kMaxPosition = Double.POSITIVE_INFINITY;
        public int kRelativeEncoderAvgSamplingDepth = 2;

        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kFF = 0.0;
        public double kPIDOutputRange = 0.0;

        public double kMaxSpeed = 0.0; // Radians
        public double kMaxAccel = 0.0; // Radians

        public FourBarFeedForwardConstants kFourBarFFConstants;

        public IdleMode kIdleMode = IdleMode.kCoast;

        public int kSmartCurrentLimit = 40;
        public double kVoltageCompensation = 12.0;
    }

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        this.constants = constants;

        mMaster = new CANSparkMax(constants.kMasterConstants.id, MotorType.kBrushless);
        mMaster.restoreFactoryDefaults();
        mMaster.enableVoltageCompensation(constants.kVoltageCompensation);
        mMaster.setSmartCurrentLimit(constants.kSmartCurrentLimit);
        mMaster.setIdleMode(constants.kIdleMode);
        // mMaster.setCANTimeout(10);
        // mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);

        mFollowers = new CANSparkMax[constants.kFollowerConstants.length];

        for (int i = 0; i < constants.kFollowerConstants.length; i++) {
            mFollowers[i] = new CANSparkMax(constants.kFollowerConstants[i].id, MotorType.kBrushless);
            mFollowers[i].setInverted(constants.kMasterConstants.isInverted);
            mFollowers[i].enableVoltageCompensation(constants.kVoltageCompensation);
            mFollowers[i].setSmartCurrentLimit(constants.kSmartCurrentLimit);
            mFollowers[i].setIdleMode(constants.kIdleMode);
            mFollowers[i].follow(mMaster, constants.kFollowerConstants[i].isInverted);
        }

        mAbsoluteEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);
        mAbsoluteEncoder.setPositionConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setInverted(constants.kIsInverted);
        mAbsoluteEncoder.setZeroOffset(constants.kOffset);

        // mRelativeEncoder = mMaster.getEncoder();
        // mRelativeEncoder.setPositionConversionFactor(constants.kUnitsPerRotation / constants.kGearRatio);
        // mRelativeEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation / (60.0 * constants.kGearRatio));
        // mRelativeEncoder.setAverageDepth(constants.kRelativeEncoderAvgSamplingDepth);
        // mRelativeEncoder.setPosition(constants.kOffset / constants.kGearRatio);

        mFourBarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
        mTrapezoidProfile = new TrapezoidProfile(new Constraints(constants.kMaxSpeed, constants.kMaxAccel));

        mPIDController = mMaster.getPIDController();
        // mPIDController.setFeedbackDevice(mAbsoluteEncoder);
        mPIDController.setPositionPIDWrappingEnabled(constants.kIsCircular);
        mPIDController.setPositionPIDWrappingMaxInput(constants.kMaxPosition);
        mPIDController.setPositionPIDWrappingMinInput(constants.kMinPosition);
        mPIDController.setP(constants.kP);
        mPIDController.setI(constants.kI);
        mPIDController.setD(constants.kD);
        mPIDController.setFF(constants.kFF);
        // mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        // mPIDController.setSmartMotionAllowedClosedLoopError(constants.k);
        // mPIDController.setSmartMotionMaxAccel(constants.kMaxAccel *
        // constants.kGearRatio , 0);
        // mPIDController.setSmartMotionMaxVelocity(constants.kMaxSpeed *
        // constants.kGearRatio, 0);
        mPIDController.setOutputRange(-constants.kPIDOutputRange, constants.kPIDOutputRange);

        mMaster.burnFlash();

        mPeriodicIO = new PeriodicIO();
    }

    public enum ControlState {
        OPEN_LOOP,
        SPARK_PID,
        SPARK_PID_FF,
        FOUR_BAR_FF,
        PROFILED,
        TEST
    }

    public enum TrapezoidProfileConfig {
        SLOW,
        MEDIUM,
        SONIC
    }

    // TODO: properly implement brake mode
    public synchronized void setIdleMode(IdleMode idleMode) {
        if (mMaster.getIdleMode() == idleMode)
            return;

        mMaster.setIdleMode(idleMode);

        for (CANSparkMax follwer : mFollowers) {
            follwer.setIdleMode(idleMode);
        }
    }

    public IdleMode getIdleMode() {
        return mMaster.getIdleMode();
    }

    public double getMeasPosition() {
        return mPeriodicIO.meas_position_units;
    }

    public double getMeasVelocity() {
        return mPeriodicIO.meas_velocity_units;
    }

    public double getAppliedVoltage() {
        return mPeriodicIO.meas_applied_voltage;
    }

    public synchronized void setVoltage(double voltage) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = voltage;
    }

    // TODO: look into S curve
    public synchronized void setFourBarFFSetpoint(double position_rad) {
        if (mControlState != ControlState.FOUR_BAR_FF) {
            mControlState = ControlState.FOUR_BAR_FF;
        }

        if (mStateSetpoint != null) {
            if (position_rad == mStateSetpoint.position)
                return;
        }

        mStateSetpoint = new State(position_rad, 0.0);

        mProfileStartTime = mPeriodicIO.timestamp;

        SmartDashboard.putNumber("State Setopint", mStateSetpoint.position);
    }

    public synchronized void setFourBarFFSetpointTEST(double position_rad) {
        if (mControlState != ControlState.FOUR_BAR_FF) {
            mControlState = ControlState.FOUR_BAR_FF;
        }

        if (mStateSetpoint != null) {
            if (position_rad == mStateSetpoint.position)
                return;
        }

        mStateSetpoint = new State(position_rad, 0.0);

        mProfileStartTime = mPeriodicIO.timestamp;
    }

    public synchronized void setPositionPIDSetpoint(double position_rad) {
        if (mControlState != ControlState.SPARK_PID) {
            mControlState = ControlState.SPARK_PID;
        }

        mPIDController.setReference(position_rad, ControlType.kPosition, 0, constants.kFourBarFFConstants.kS,
                ArbFFUnits.kVoltage);
    }

    public synchronized void updateFourbarFFConfigs() {
    }

    private class PeriodicIO {
        // Inputs
        double timestamp;
        double meas_position_units;
        double meas_velocity_units;
        // double meas_internal_position_units;
        // double meas_internal_velocity_units;
        double meas_applied_voltage;
        State meas_state;

        // Outputs
        double demand;
        double feed_forward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.meas_position_units = mAbsoluteEncoder.getPosition();
        mPeriodicIO.meas_velocity_units = mAbsoluteEncoder.getVelocity();
        // mPeriodicIO.internal_position_units = mRelativeEncoder.getPosition();
        // mPeriodicIO.internal_velocity_units = mRelativeEncoder.getVelocity();
        mPeriodicIO.meas_applied_voltage = mMaster.getAppliedOutput() * 12.0;
        mPeriodicIO.meas_state = new State(mPeriodicIO.meas_position_units, mPeriodicIO.meas_velocity_units);

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (this) {
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMaster.setVoltage(-mPeriodicIO.demand);
        } else if (mControlState == ControlState.FOUR_BAR_FF) {
            SmartDashboard.putNumber("Time Since Start", mPeriodicIO.timestamp - mProfileStartTime);
            SmartDashboard.putNumber("Input Measured State Position", mPeriodicIO.meas_state.position);
            SmartDashboard.putNumber("Input Measured State Velocity", mPeriodicIO.meas_state.velocity);
            SmartDashboard.putNumber("Input Setpoint State Position", mStateSetpoint.position);
            SmartDashboard.putNumber("Input Setpoint State Velocity", mStateSetpoint.velocity);

            State state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp - mProfileStartTime, mPeriodicIO.meas_state,
                    mStateSetpoint);
            mPeriodicIO.feed_forward = Util.limit(mFourBarFF.calculate(state.position, state.velocity), 4.8);

            SmartDashboard.putNumber("State Position", state.position);
            SmartDashboard.putNumber("State Velocity", state.velocity);

            mMaster.setVoltage(-mPeriodicIO.feed_forward);
            SmartDashboard.putNumber("Fourbar Feedforward Voltage", -mPeriodicIO.feed_forward);
        } else if (mControlState == ControlState.TEST) {
            SmartDashboard.putNumber("Time Since Start", mPeriodicIO.timestamp - mProfileStartTime);
            SmartDashboard.putNumber("Input Measured State Position", mPeriodicIO.meas_state.position);
            SmartDashboard.putNumber("Input Measured State Velocity", mPeriodicIO.meas_state.velocity);
            SmartDashboard.putNumber("Input Setpoint State Position", mStateSetpoint.position);
            SmartDashboard.putNumber("Input Setpoint State Velocity", mStateSetpoint.velocity);

            State state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp - mProfileStartTime, mPeriodicIO.meas_state,
                    mStateSetpoint);
            mPeriodicIO.feed_forward = Util.limit(mFourBarFF.calculate(state.position, state.velocity), 4.8)
                    + SmartDashboard.getNumber("kS Calibration", 0.0);

            mMaster.setVoltage(-mPeriodicIO.feed_forward);
            SmartDashboard.putNumber("Fourbar Feedforward Voltage", mPeriodicIO.feed_forward);
        }
    }

    @Override
    public void outputTelemetry() {

    }
}
