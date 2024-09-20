package frc.team4276.lib.rev;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.lib.characterizations.IFeedForward;
import frc.team4276.lib.motion.TrapezoidProfile;

/**
 * Experimental class; mimic motion magic
 */
public class VIKCANSparkMaxServo extends VIKCANSparkMax {
    public VIKCANSparkMaxServo(int deviceId) {
        super(deviceId);
    }

    public static class FuseMotionConfig {
        public int kProfileSlot = 2;
        public IFeedForward kFeedForward;
        public double kLooperDt = Constants.kLooperDt; // check frame updates
        public double kMaxVel = 0.0; // Copy Subsystem
        public double kMaxAccel = 0.0; // Copy Subsystem
        public EncoderMode kEncoderMode = EncoderMode.INTERNAL;
    }

    private int kProfileSlotFuse;
    private IFeedForward kFuseMotionFF;
    private double kLooperDt;
    private TrapezoidProfile kProfileFuse;

    /**
     * Only use on init
     */
    public void configFuseMotion(FuseMotionConfig config) {
        this.kFuseMotionFF = config.kFeedForward;
        this.kLooperDt = config.kLooperDt;
        this.kProfileFuse = new TrapezoidProfile(config.kMaxVel, config.kMaxAccel);
        this.kProfileSlotFuse = config.kProfileSlot;
        setEncoderMode(config.kEncoderMode);
        fuseMotionLooper = new Notifier(updateFuse);
    }

    private double[] setpoint_fuse = { Double.NaN, 0.0 };
    private double profile_timestamp_fuse;
    private double[] profile_start_fuse = { Double.NaN, Double.NaN };

    private Notifier fuseMotionLooper;

    private boolean isFuseMotion = false;

    /**
     * @return true if successful
     */
    public synchronized boolean setFuseMotionSetpoint(double setpoint) {
        if (kFuseMotionFF == null)
            return false;

        if (isFuseMotion && setpoint_fuse[0] == setpoint)
            return true;

        this.setpoint_fuse[0] = setpoint;
        profile_timestamp_fuse = Timer.getFPGATimestamp();
        profile_start_fuse[0] = getPosition();
        profile_start_fuse[1] = getVelocity();

        if (!isFuseMotion) {
            isFuseMotion = true;

            fuseMotionLooper.startPeriodic(kLooperDt);
        }

        return true;
    }

    double prevTime = 0.0;

    private Runnable updateFuse = new Runnable() {
        public void run() {
            if (!isFuseMotion) {
                fuseMotionLooper.stop();
                return;
            }

            double currTime = Timer.getFPGATimestamp();

            System.out.println(currTime - prevTime);
            
            prevTime = currTime;

            double[] state = kProfileFuse.calculate(Timer.getFPGATimestamp() - profile_timestamp_fuse,
                    profile_start_fuse, setpoint_fuse);

            if (kFuseMotionFF.isLinear()) {
                getPIDController().setReference(state[0], ControlType.kPosition, kProfileSlotFuse,
                        kFuseMotionFF.calculate(state[0], state[1], 0.0), ArbFFUnits.kVoltage);

            } else { // Asume setpoint given in degrees
                double ff = kFuseMotionFF.calculate(Math.toRadians(state[0]), Math.toRadians(state[1]), 0.0);

                SmartDashboard.putNumber("Debug/Test/FF Voltage", ff);

                if (ControlBoard.getInstance().driver.getAButton()) {
                    getPIDController().setReference(state[0], ControlType.kPosition,
                            kProfileSlotFuse,
                            ff, ArbFFUnits.kVoltage);

                } else {
                    setVoltage(0.0);
                }

            }
        }
    };

    

    @Override
    public synchronized void setVoltage(double outputVolts) {
        isFuseMotion = false;
        super.setVoltage(outputVolts);
    }

    @Override
    public synchronized void setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward,
            ArbFFUnits arbFFUnits) {
        isFuseMotion = false;
        super.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

    private EncoderMode mEncoderMode = EncoderMode.INTERNAL;

    public enum EncoderMode {
        INTERNAL,
        ABSOLUTE,
        ALTERNATE
    }

    public void setEncoderMode(EncoderMode mode) {
        mEncoderMode = mode;
    }

    /**
     * Do not use Alternate Mode
     * @return
     */
    public double getPosition(){
        switch (mEncoderMode) {
            default:
                return super.getEncoder().getPosition();
            case INTERNAL:
                return super.getEncoder().getPosition();
            
            case ABSOLUTE:
                return super.getAbsoluteEncoder().getPosition();

            case ALTERNATE:
                // DO NOT USE
                return Double.NaN;

        }
    }

    /**
     * Do not use Alternate Mode
     * @return
     */
    public double getVelocity(){
        switch (mEncoderMode) {
            default:
                return super.getEncoder().getVelocity();
            case INTERNAL:
                return super.getEncoder().getVelocity();
            
            case ABSOLUTE:
                return super.getAbsoluteEncoder().getVelocity();

            case ALTERNATE:
                // DO NOT USE
                return Double.NaN;

        }
    }

    
}
