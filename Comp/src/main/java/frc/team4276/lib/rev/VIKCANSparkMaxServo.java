package frc.team4276.lib.rev;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

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
    }

    private int kProfileSlotFuse;
    private IFeedForward kFuseMotionFF;
    private double kLooperDt;
    private TrapezoidProfile kProfileFuse;

    private Supplier<Double> mPositionSupplier;

    private boolean isFuseConfiged = false;

    /**
     * Only use on init
     */
    public void configFuseMotion(FuseMotionConfig config, Supplier<Double> positionSupplier) {
        this.kFuseMotionFF = config.kFeedForward;
        this.kLooperDt = config.kLooperDt;
        this.kProfileFuse = new TrapezoidProfile(config.kMaxVel, config.kMaxAccel);
        this.kProfileSlotFuse = config.kProfileSlot;
        mPositionSupplier = positionSupplier;
        fuseMotionLooper = new Notifier(updateFuse);
        fuseMotionLooper.startPeriodic(kLooperDt);
        isFuseConfiged = true;
    }

    private double[] setpoint_fuse = { Double.NaN, 0.0 };
    private double[] stateSetpoint = {0.0, 0.0};

    private Notifier fuseMotionLooper;
    private boolean isFuseMotion = false;

    /**
     * @return true if successful
     */
    public synchronized boolean setFuseMotionSetpoint(double setpoint) {
        if (!isFuseConfiged)
            return false;
        
        isFuseMotion = true;

        this.setpoint_fuse[0] = setpoint;

        if (!isFuseMotion || DriverStation.isDisabled()) {
            stateSetpoint[0] = mPositionSupplier.get();

        }

        return true;
    }

    private Runnable updateFuse = new Runnable() {
        @Override
        public void run() {
            if (!isFuseMotion) return;

            updateFuse();
        }
    };

    private synchronized void updateFuse() {
        stateSetpoint = kProfileFuse.calculate(kLooperDt,
                stateSetpoint, setpoint_fuse);

        double ff;

        if (kFuseMotionFF.isLinear()) {
            ff = kFuseMotionFF.calculate(stateSetpoint[0], stateSetpoint[1], 0.0);

        } else { // Asume setpoint given in degrees
            ff = kFuseMotionFF.calculate(Math.toRadians(stateSetpoint[0]), Math.toRadians(stateSetpoint[1]), 0.0);

        }

        if(ControlBoard.getInstance().enableFourbarFuse()){
            getPIDController().setReference(stateSetpoint[0], ControlType.kPosition, kProfileSlotFuse, ff);
            
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
