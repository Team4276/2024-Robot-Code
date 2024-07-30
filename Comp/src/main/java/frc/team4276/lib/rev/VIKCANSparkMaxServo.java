package frc.team4276.lib.rev;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.team4276.frc2024.Constants;
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
    private Supplier<Double> poseSupplier;
    private Supplier<Double> velSupplier;

    public void configFuseMotion(FuseMotionConfig config, Supplier<Double> pose_supplier, Supplier<Double> vel_supplier){
        this.kFuseMotionFF = config.kFeedForward;
        this.kLooperDt = config.kLooperDt;
        this.kProfileFuse = new TrapezoidProfile(config.kMaxVel, config.kMaxAccel);
        this.kProfileSlotFuse = config.kProfileSlot;
        this.poseSupplier = pose_supplier;
        this.velSupplier = vel_supplier;
    }

    private double[] setpoint_fuse = { Double.NaN, 0.0 };
    private double profile_timestamp_fuse;
    private double[] profile_start_fuse = { Double.NaN, Double.NaN };

    private Notifier looper = new Notifier(this::updateFuse);

    /**
     * @return true if successful
     */
    public boolean setFuseMotionSetpoint(double setpoint) {
        if (kFuseMotionFF == null)
            return false;

        if (this.setpoint_fuse[0] == setpoint)
            return true;

        this.setpoint_fuse[0] = setpoint;
        profile_timestamp_fuse = Timer.getFPGATimestamp();
        profile_start_fuse[0] = poseSupplier.get();
        profile_start_fuse[1] = velSupplier.get();

        looper.startPeriodic(kLooperDt);

        return true;
    }

    private void updateFuse() {
        double[] state = kProfileFuse.calculate(Timer.getFPGATimestamp() - profile_timestamp_fuse,
                profile_start_fuse, setpoint_fuse);

        getPIDController().setReference(state[0], ControlType.kPosition, kProfileSlotFuse,
                kFuseMotionFF.calculate(state[0], state[1]), ArbFFUnits.kVoltage);

    }

}
