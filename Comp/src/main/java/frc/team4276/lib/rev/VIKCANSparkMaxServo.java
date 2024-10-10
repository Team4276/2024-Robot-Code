package frc.team4276.lib.rev;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.lib.Threading.ThreadWait;
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
    }

    private int kProfileSlotFuse;
    private IFeedForward kFuseMotionFF;
    private double kLooperDt;
    private TrapezoidProfile kProfileFuse;

    private Supplier<Double> mPositionSupplier;
    private Supplier<Double> mVelocitySupplier;

    /**
     * Only use on init
     */
    public void configFuseMotion(FuseMotionConfig config, Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier) {
        this.kFuseMotionFF = config.kFeedForward;
        this.kLooperDt = config.kLooperDt;
        this.kProfileFuse = new TrapezoidProfile(config.kMaxVel, config.kMaxAccel);
        this.kProfileSlotFuse = config.kProfileSlot;
        mPositionSupplier = positionSupplier;
        mVelocitySupplier = velocitySupplier;
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
        profile_start_fuse[0] = mPositionSupplier.get();
        profile_start_fuse[1] = mVelocitySupplier.get();

        if (!isFuseMotion) {
            isFuseMotion = true;

            fuseMotionLooper.startPeriodic(kLooperDt);
        }

        return true;
    }

    private double maxTime = 0.0;
    private int counter = 0;

    double timestamp_ = 0.0;
    double dt_ = 0.0;

    private Runnable updateFuse = new Runnable() {
        @Override
        public void run() {
            if (!isFuseMotion) {
                return;
            }
            
            double now = Timer.getFPGATimestamp();

            updateFuse();

            ThreadWait.threadWait(kLooperDt, timestamp_, now);
            now = Timer.getFPGATimestamp();
            dt_ = now - timestamp_;
            timestamp_ = now;

            if (dt_ > maxTime) {
                maxTime = dt_;
                // System.out.println(maxTime);
                counter = 0;

            } else if(counter >= 100){
                maxTime = dt_;

            } else {
                counter++;
            }

            // System.out.println(dt_);
        }
    };

    private synchronized void updateFuse() {
        double[] state = kProfileFuse.calculate(Timer.getFPGATimestamp() - profile_timestamp_fuse,
                profile_start_fuse, setpoint_fuse);

        double ff;

        if (kFuseMotionFF.isLinear()) {
            ff = kFuseMotionFF.calculate(state[0], state[1], 0.0);

        } else { // Asume setpoint given in degrees
            ff = kFuseMotionFF.calculate(Math.toRadians(state[0]), Math.toRadians(state[1]), 0.0);

        }
            
        SmartDashboard.putNumber("Debug/Test/FF Voltage", ff);

        if(ControlBoard.getInstance().enableFourbarFuse()){
            getPIDController().setReference(state[0], ControlType.kPosition, kProfileSlotFuse, ff, ArbFFUnits.kVoltage);
            
        } else {
            setVoltage(0.0);

        }


    }

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
}
