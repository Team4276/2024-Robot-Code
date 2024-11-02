package frc.team4276.lib.drivers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import frc.team4276.frc2024.Constants;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.VIKCANSparkMaxServo;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team254.lib.util.Util;

public abstract class ServoMotorSubsystem extends Subsystem {
    public static class ServoMotorConfig {
        public int id;
        public boolean isInverted;
    }

    public static class ServoMotorSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public ServoMotorConfig kMasterConstants = new ServoMotorConfig();
        public ServoMotorConfig[] kFollowerConstants = new ServoMotorConfig[0];

        public int kSmartCurrentLimit = 40;
        public CANSparkBase.IdleMode kIdleMode = CANSparkBase.IdleMode.kCoast;
        public boolean kIsCircular = false; // Units expect to be 
        public double kMinPosition = Double.NEGATIVE_INFINITY;
        public double kMaxPosition = Double.POSITIVE_INFINITY;
        public double kMaxVel = 0.0;
        public double kMaxAccel = 0.0;
        public double kS = 0.0;
        public double kTol = 0.0;
        public SparkLimitSwitch.Type kForwardLimitPolarity = null; // Null for disable
        public SparkLimitSwitch.Type kReverseLimitPolarity = null; // Null for disable

        public int kSlotIdSmartMotionCruise = 0;
        public int kSlotIdSmartMotionMaintain = 1;
        public int kSlotIdFuseMotion = 2;

        public CANSparkMaxFactory.CANSparkMaxPIDFConfig[] kPidfConfigs = new CANSparkMaxFactory.CANSparkMaxPIDFConfig[0];

        public VIKCANSparkMaxServo.FuseMotionConfig kFuseMotionConfig = new VIKCANSparkMaxServo.FuseMotionConfig();
    }

    protected final ServoMotorSubsystemConstants mConstants;
    protected final VIKCANSparkMaxServo mMaster;
    protected final VIKCANSparkMax[] mFollowers;

    protected SparkLimitSwitch mForwardLimitSwitch;
    protected SparkLimitSwitch mReverseLimitSwitch;

    /**
     * Encoder Init needed
     * 
     * @param constants
     */
    public ServoMotorSubsystem(ServoMotorSubsystemConstants constants) {
        mConstants = constants;
        mMaster = CANSparkMaxFactory.createDefaultServo(mConstants.kMasterConstants.id);
        mFollowers = new VIKCANSparkMax[mConstants.kFollowerConstants.length];

        if (mConstants.kForwardLimitPolarity != null) {
            mForwardLimitSwitch = mMaster.getForwardLimitSwitch(mConstants.kForwardLimitPolarity);
            mForwardLimitSwitch.enableLimitSwitch(true);
        }

        if (mConstants.kReverseLimitPolarity != null) {
            mReverseLimitSwitch = mMaster.getReverseLimitSwitch(mConstants.kReverseLimitPolarity);
            mReverseLimitSwitch.enableLimitSwitch(true);
        }

        SparkPIDController pidfContoller = mMaster.getPIDController();

        for (int i = 0; i < mConstants.kPidfConfigs.length; i++) {
            CANSparkMaxFactory.configPIDF(mMaster, mConstants.kPidfConfigs[i]);
            pidfContoller.setSmartMotionMaxVelocity(mConstants.kMaxVel, i);
            pidfContoller.setSmartMotionMaxAccel(mConstants.kMaxAccel, i);
            pidfContoller.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, i);
            pidfContoller.setSmartMotionAllowedClosedLoopError(mConstants.kTol, i);
        }

        if (mConstants.kIsCircular) {
            pidfContoller.setPositionPIDWrappingEnabled(mConstants.kIsCircular);
            pidfContoller.setPositionPIDWrappingMaxInput(mConstants.kMaxPosition);
            pidfContoller.setPositionPIDWrappingMinInput(mConstants.kMinPosition);
        }

        mMaster.setInverted(mConstants.kMasterConstants.isInverted);
        mMaster.setSmartCurrentLimit(mConstants.kSmartCurrentLimit);
        mMaster.setIdleMode(mConstants.kIdleMode);

        for (int i = 0; i < mFollowers.length; i++) {
            mFollowers[i] = CANSparkMaxFactory.createDefaultFollower(
                mConstants.kFollowerConstants[i].id, mMaster, 
                mConstants.kFollowerConstants[i].isInverted);

            VIKCANSparkMax follower = mFollowers[i];

            follower.setSmartCurrentLimit(mConstants.kSmartCurrentLimit);
            follower.setIdleMode(mConstants.kIdleMode);

        }

        mPeriodicIO = new PeriodicIO();
    }

    // Call once on init in children
    protected void burnFlash() {
        mMaster.burnFlash();
        for (VIKCANSparkMax follower : mFollowers) {
            follower.burnFlash();
        }
    }

    public PeriodicIO mPeriodicIO;

    protected ControlState mControlState = ControlState.VOLTAGE;

    protected boolean mIsMaintain = false;

    protected boolean mIsDisabled = false;

    protected enum ControlState {
        VOLTAGE,
        SMART_MOTION, // Onboard PIDF
        FUSE_MOTION // RIO FF w/ Onboard PID

    }

    public synchronized void setDisabled(boolean isDisabled){
        mIsDisabled = isDisabled;
    }

    public synchronized void setAbsoluteEncoderZero(double currentPosition) {
        mMaster.zeroAbsoluteEncoder(currentPosition);
    }

    public synchronized void getAbsoluteEncoderZeroOffset() {
        mMaster.getAbsoluteEncoder().getZeroOffset();
    }

    public synchronized void setVoltage(double voltage) {
        if (mControlState != ControlState.VOLTAGE) {
            mControlState = ControlState.VOLTAGE;
        }

        mPeriodicIO.demand = voltage;
    }

    public synchronized void setSmartMotionSetpoint(double position) {
        if (mControlState != ControlState.SMART_MOTION) {
            mControlState = ControlState.SMART_MOTION;
        } else if (mPeriodicIO.demand == position) {
            return;
        }

        mPeriodicIO.demand = constrain(position);
    }

    public synchronized void setFuseMotionSetpoint(double position) {
        if (mControlState != ControlState.FUSE_MOTION) {
            mControlState = ControlState.FUSE_MOTION;
        } else if (Util.epsilonEquals(mPeriodicIO.demand, position, 1.0)) {
            return;
        }

        mPeriodicIO.demand = constrain(position);

    }

    protected double constrain(double pos) {
        return Math.max(Math.min(pos, mConstants.kMaxPosition), mConstants.kMinPosition);
    }

    

    public void setWantBrakeMode(boolean brake) {
        mMaster.setWantBrakeMode(brake);
        for (VIKCANSparkMax follower : mFollowers) {
            follower.setWantBrakeMode(brake);
        }
    }

    public double getPosition(){
        return mMaster.getEncoder().getPosition();
    }

    public double getVelocity(){
        return mMaster.getEncoder().getVelocity();
    }

    public boolean isBrakeMode() {
        return mMaster.getIdleMode() == CANSparkBase.IdleMode.kBrake;
    }

    /**
     * False for voltage mode
     * @return
     */
    public boolean atSetpoint() {
        return atSetpoint(mConstants.kTol);
    }

    public boolean atSetpoint(double tol) {
        if(mControlState == ControlState.VOLTAGE) return false;

        return Math.abs(mPeriodicIO.meas_position - mPeriodicIO.demand) < tol;
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    public class PeriodicIO {
        // Inputs
        public double meas_master_voltage;
        public double meas_position;
        public double meas_velocity;

        // Outputs
        public double demand;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.meas_master_voltage = mMaster.getAppliedVoltage();
        mPeriodicIO.meas_position = getPosition();
        mPeriodicIO.meas_velocity = getVelocity();

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(mIsDisabled) {
            mMaster.setVoltage(0.0);
            return;
        }

        switch (mControlState) {
            case VOLTAGE:
                mMaster.setVoltage(mPeriodicIO.demand);

                break;

            case SMART_MOTION:
                mIsMaintain = Math.abs(mPeriodicIO.meas_position - mPeriodicIO.demand) > mConstants.kTol;

                if (mIsMaintain) {
                    mMaster.setReference(mPeriodicIO.demand, CANSparkBase.ControlType.kSmartMotion,
                            mConstants.kSlotIdSmartMotionCruise,
                            mConstants.kS, SparkPIDController.ArbFFUnits.kVoltage);

                } else {
                    mMaster.setReference(mPeriodicIO.demand, CANSparkBase.ControlType.kPosition,
                            mConstants.kSlotIdSmartMotionMaintain,
                            mConstants.kS, SparkPIDController.ArbFFUnits.kVoltage);

                }

                break;

            case FUSE_MOTION:
                mMaster.setFuseMotionSetpoint(mPeriodicIO.demand);

                break;

            default:
                break;
        }
    }

    @Override
    public synchronized void outputTelemetry() {
        if(Constants.disableExtraTelemetry) return;

        SmartDashboard.putNumber("Debug/" + mConstants.kName + " Measured Voltage", mPeriodicIO.meas_master_voltage);
        SmartDashboard.putNumber("Debug/" + mConstants.kName + " Measured Velocity", mPeriodicIO.meas_velocity);
        SmartDashboard.putString("Debug/" + mConstants.kName + " Controlstate", mControlState.name());
        SmartDashboard.putBoolean("Debug/" + mConstants.kName + " is Maintain", mIsMaintain);

    }

}
