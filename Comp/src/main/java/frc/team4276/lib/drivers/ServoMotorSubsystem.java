package frc.team4276.lib.drivers;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.lib.drivers.FourBarFeedForward.FourBarFeedForwardConstants;

// Subsystem class for NEO v1.1 Brushless motors

//TODO: add soft limits to everything;

public abstract class ServoMotorSubsystem extends Subsystem {
    private final CANSparkMax mMaster;
    private final CANSparkMax[] mFollowers;

    private final AbsoluteEncoder mEncoder;

    private ControlState mControlState;
    private final FourBarFeedForward mFourBarFF;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private TrapezoidProfile mTrapezoidProfile;

    public class ServoMotorConstants {
        public int id;
        public boolean isInverted; // TODO: check if relative to the master
    }

    public class ServoMotorSubsystemConstants {// TODO: look into avg depth sampling
        public ServoMotorConstants kMasterConstants = new ServoMotorConstants();
        public ServoMotorConstants[] kFollowerConstants = new ServoMotorConstants[0];

        public boolean kIsInverted = false;
        public boolean kIsCircular = false;
        public double kUnitsPerRotation = 1.0;
        public double kOffset = 0; // Set in hardware client
        public double kHomePosition = 0.0;
        public double kMinPosition = Double.NEGATIVE_INFINITY;
        public double kMaxPosition = Double.POSITIVE_INFINITY;

        public ControlState kControlState = ControlState.SPARK_PID;

        // TODO: add PID placeholders here

        public FourBarFeedForwardConstants kFourBarFFConstants;
        public Constraints kFourBarProfileContraints;

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

        mFollowers = new CANSparkMax[constants.kFollowerConstants.length];

        for (int i = 0; i < constants.kFollowerConstants.length; i++) {
            mFollowers[i] = new CANSparkMax(constants.kFollowerConstants[i].id, MotorType.kBrushless);
            mFollowers[i].setInverted(constants.kMasterConstants.isInverted);
            mFollowers[i].enableVoltageCompensation(constants.kVoltageCompensation);
            mFollowers[i].setSmartCurrentLimit(constants.kSmartCurrentLimit);
            mFollowers[i].setIdleMode(constants.kIdleMode);
            mFollowers[i].follow(mMaster, constants.kFollowerConstants[i].isInverted);
        }

        mEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);
        mEncoder.setInverted(constants.kIsInverted);
        mEncoder.setPositionConversionFactor(constants.kUnitsPerRotation);
        mEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation / 60.0);
        mEncoder.setZeroOffset(constants.kOffset);

        mControlState = constants.kControlState;

        mFourBarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
        mTrapezoidProfile = new TrapezoidProfile(constants.kFourBarProfileContraints);

        mMaster.burnFlash();
    }

    public enum ControlState {
        OPEN_LOOP,
        SPARK_PID,
        SPARK_PID_FF,
        FOUR_BAR_FF
    }

    public void setOpenLoop(double volts) {
        if (mControlState != ControlState.OPEN_LOOP) {
            mControlState = ControlState.OPEN_LOOP;
        }
    }

    //TODO: look into S curve
    public void setFourBarFFSetpoint(double position_rad) {
        mPeriodicIO.fourbar_setpoint = new State(position_rad, 0);

        if (mControlState != ControlState.FOUR_BAR_FF) {
            mControlState = ControlState.FOUR_BAR_FF;
        }
    }

    private class PeriodicIO{
        // Inputs
        double timestamp;
        double position_units;
        double velocity_units;
        State fourbar_setpoint;

        // Outputs
        double demand;
        double feed_forward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.position_units = mEncoder.getPosition();
        mPeriodicIO.velocity_units = mEncoder.getVelocity();
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
        } else if (mControlState == ControlState.SPARK_PID || mControlState == ControlState.SPARK_PID_FF) {

        } else if (mControlState == ControlState.FOUR_BAR_FF) {
            mMaster.setVoltage(mPeriodicIO.feed_forward);
        }
    }

    private void updateFourBarSetpoint(){
        State state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp, new State(mPeriodicIO.position_units, mPeriodicIO.velocity_units), mPeriodicIO.fourbar_setpoint);

        mPeriodicIO.feed_forward = mFourBarFF.calculate(state.position, state.velocity);
    }

    

    
}
