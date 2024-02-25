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
import com.revrobotics.SparkPIDController.AccelStrategy;
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
    private final RelativeEncoder mRelativeEncoder;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private ControlState mControlState;

    private SparkPIDController mPIDController;

    private final FourBarFeedForward mFourBarFF;
    private TrapezoidProfile mTrapezoidProfile;

    private double mProfileStartTime = 0.0;
    

    public static class ServoMotorConstants {
        public int id;
        public boolean isInverted; // TODO: check if relative to the master
    }

    public static class ServoMotorSubsystemConstants {
        public ServoMotorConstants kMasterConstants = new ServoMotorConstants();
        public ServoMotorConstants[] kFollowerConstants = new ServoMotorConstants[0];

        public boolean kIsInverted = false;
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

        // TODO: add PID placeholders here

        public double kMaxSpeed = 0.0; // Radians
        public double kMaxAccel = 0.0; // Radians

        public FourBarFeedForwardConstants kFourBarFFConstants;


        public IdleMode kIdleMode = IdleMode.kCoast;

        public int kSmartCurrentLimit = 40;
        public double kVoltageCompensation = 12.0;
    }

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        mMaster = new CANSparkMax(constants.kMasterConstants.id, MotorType.kBrushless);
        mMaster.restoreFactoryDefaults();
        mMaster.setInverted(constants.kMasterConstants.isInverted);
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
        mAbsoluteEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation / 60.0);
        mAbsoluteEncoder.setInverted(constants.kIsInverted);
        mAbsoluteEncoder.setZeroOffset(constants.kOffset);

        mRelativeEncoder = mMaster.getEncoder();
        mRelativeEncoder.setPositionConversionFactor(constants.kUnitsPerRotation / constants.kGearRatio);
        mRelativeEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation / (60.0 * constants.kGearRatio));
        mRelativeEncoder.setAverageDepth(constants.kRelativeEncoderAvgSamplingDepth);
        mRelativeEncoder.setPosition(constants.kOffset / constants.kGearRatio);

        mFourBarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
        mTrapezoidProfile = new TrapezoidProfile(new Constraints(constants.kMaxSpeed, constants.kMaxAccel));

        mPIDController = mMaster.getPIDController();
        mPIDController.setFeedbackDevice(mAbsoluteEncoder);
        mPIDController.setPositionPIDWrappingEnabled(constants.kIsCircular);
        mPIDController.setPositionPIDWrappingMaxInput(constants.kMaxPosition);
        mPIDController.setPositionPIDWrappingMinInput(constants.kMinPosition);
        mPIDController.setP(mProfileStartTime);
        mPIDController.setI(mProfileStartTime);
        mPIDController.setD(mProfileStartTime);
        mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        // mPIDController.setSmartMotionAllowedClosedLoopError(constants.k);
        mPIDController.setSmartMotionMaxAccel(constants.kMaxAccel * constants.kGearRatio , 0);
        mPIDController.setSmartMotionMaxVelocity(constants.kMaxSpeed * constants.kGearRatio, 0);

        mMaster.burnFlash();

        SmartDashboard.putNumber("Fourbar Vel", 0.0);
    }

    public enum ControlState {
        OPEN_LOOP,
        SPARK_PID,
        SPARK_PID_FF,
        FOUR_BAR_FF,
        PROFILED
    }

    //TODO: properly implement brake mode
    public synchronized void setIdleMode(IdleMode idleMode) {
        if(mMaster.getIdleMode() == idleMode) return;

        mMaster.setIdleMode(idleMode);
        
        for (CANSparkMax follwer : mFollowers) {
            follwer.setIdleMode(idleMode);
        }
    }

    public IdleMode getIdleMode(){
        return mMaster.getIdleMode();
    }

    public synchronized void setVoltage(double voltage) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = voltage;
    }

    //TODO: look into S curve
    public synchronized void setFourBarFFSetpoint(double position_rad) {
        if (mControlState != ControlState.FOUR_BAR_FF) {
            mControlState = ControlState.FOUR_BAR_FF;
        }

        mPeriodicIO.position_setpoint_units = position_rad;
        mPeriodicIO.velocity_setpoint_units = 0.0;
        
        mProfileStartTime = mPeriodicIO.timestamp;
    }

    public synchronized void setFourbarProfiledSetpoint(double position_rad){
        if(mControlState != ControlState.PROFILED){
            mControlState = ControlState.PROFILED;
        }

        mPeriodicIO.position_setpoint_units = position_rad;
    }

    public synchronized void updateFourbarFFConfigs(){
    }

    private class PeriodicIO{
        // Inputs
        double timestamp;
        double position_units;
        double velocity_units;
        // double internal_position_units;
        // double internal_velocity_units;
        double applied_output;

        double position_setpoint_units;
        double velocity_setpoint_units;

        // Outputs
        double demand;
        double feed_forward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.position_units = mAbsoluteEncoder.getPosition();
        mPeriodicIO.velocity_units = mAbsoluteEncoder.getVelocity();
        // mPeriodicIO.internal_position_units = mRelativeEncoder.getPosition();
        // mPeriodicIO.internal_velocity_units = mRelativeEncoder.getVelocity();
        mPeriodicIO.applied_output = mMaster.getAppliedOutput();

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
                    switch (mControlState) {
                        case OPEN_LOOP:
                            break;
                        case SPARK_PID:
                            break;
                        case SPARK_PID_FF:
                            break;
                        case FOUR_BAR_FF:
                            updateFourBarSetpoint();
                            break;
                        default:
                            break;
                    }
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
            mMaster.setVoltage(mPeriodicIO.demand);
            SmartDashboard.putNumber("Fourbar Open Loop Voltage", mPeriodicIO.demand);
        } else if (mControlState == ControlState.SPARK_PID || mControlState == ControlState.SPARK_PID_FF) {
            // WEEEEEEEEEEEEEEEE
        } else if (mControlState == ControlState.FOUR_BAR_FF) {
            mMaster.setVoltage(Util.limit(mPeriodicIO.feed_forward, 4.8));
            SmartDashboard.putNumber("Fourbar Feedforward Voltage", mPeriodicIO.feed_forward);
        } else if (mControlState == ControlState.PROFILED) {
            mPIDController.setReference(mPeriodicIO.position_setpoint_units, ControlType.kSmartMotion, 0, 
                0.15, ArbFFUnits.kVoltage);
        }
    }
    
    private void updateFourBarSetpoint(){
        double t = mPeriodicIO.timestamp - mProfileStartTime;
        State currState = new State(mPeriodicIO.position_units, mPeriodicIO.velocity_units);
        State desState = new State(mPeriodicIO.position_setpoint_units, mPeriodicIO.velocity_setpoint_units);
        State state = mTrapezoidProfile.calculate(t, currState, desState);

        SmartDashboard.putNumber("Fourbar Des State Position", state.position);
        SmartDashboard.putNumber("Fourbar Des State Velocity", state.velocity);

        mPeriodicIO.feed_forward = mFourBarFF.calculate(state.position, state.velocity);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Fourbar Applied output", mPeriodicIO.applied_output);
        SmartDashboard.putNumber("Encoder Position", mPeriodicIO.position_units);
        SmartDashboard.putNumber("Encoder Position Degrees", Math.toDegrees(mPeriodicIO.position_units));
        SmartDashboard.putNumber("Calced FourbarFF output", mFourBarFF.calculate(mPeriodicIO.position_setpoint_units, SmartDashboard.getNumber("Fourbar Vel", 0.0)));
        SmartDashboard.putNumber("Fourbar Setpoint Position", mPeriodicIO.position_setpoint_units);
    }
}
