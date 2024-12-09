package frc.team4276.lib.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.team4276.lib.rev.RevUtil.SparkAbsoluteEncoderConfig;

public class SparkMaxFactory {
    /** Set the periodic frame period when not using the internal encoder */
    public static VIKSparkMax createDefault(int id) {
        VIKSparkMax sparkMax = new VIKSparkMax(id);

        sparkMax.restoreFactoryDefaults();
        sparkMax.clearFaults();

        sparkMax.enableVoltageCompensation(12.0);

        return sparkMax;
    }

    public static VIKSparkMax createDefaultFollower(int id, VIKSparkMax master) {
        return createDefaultFollower(id, master, false);
    }

    /**
     * @param id
     * @param master
     * @param isInverted Relative to master
     * @return
     */
    public static VIKSparkMax createDefaultFollower(int id, VIKSparkMax master, boolean isInverted) {
        VIKSparkMax sparkMax = createDefault(id);

        sparkMax.follow(master, isInverted);

        master.setPeriodicFramePeriodSec(PeriodicFrame.kStatus0, 0.005);

        return sparkMax;
    }

    public static class CANSparkMaxPIDFConfig {
        public int kSlotId = 0;
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kFF = 0.0;
        public double kDFilter = 0.0;
        public double kIZone = 0.0;
        public double kIMaxAccum = 0.0;
        public double kPIDOutputRange = 1.0;
    }

    public static void configPIDF(CANSparkMax motor, CANSparkMaxPIDFConfig config) {
        SparkPIDController controller = motor.getPIDController();

        controller.setP(config.kP, config.kSlotId);
        controller.setI(config.kI, config.kSlotId);
        controller.setD(config.kD, config.kSlotId);
        controller.setFF(config.kFF, config.kSlotId);
        controller.setDFilter(config.kDFilter, config.kSlotId);
        controller.setIZone(config.kIZone, config.kSlotId);
        controller.setIMaxAccum(config.kIMaxAccum, config.kSlotId);
        controller.setOutputRange(
                -1.0 * config.kPIDOutputRange, config.kPIDOutputRange, config.kSlotId);
    }

    /**
     * NO USE
     */
    public static void configAnalogSensor(VIKSparkMax motor, double periodSec) {
        motor.setPeriodicFramePeriodSec(PeriodicFrame.kStatus3, periodSec);
    }

    /**
     * NO USE
     */
    public static void configAlternateEncoder(VIKSparkMax motor, double periodSec) {
        motor.setPeriodicFramePeriodSec(PeriodicFrame.kStatus4, periodSec);
    }

    /**
     * NO USE
     */
    public static AbsoluteEncoder configAbsoluteEncoder(
            VIKSparkMax motor, SparkAbsoluteEncoderConfig config) {
        AbsoluteEncoder e = motor.getAbsoluteEncoder();

        e.setInverted(config.kIsInverted);
        e.setPositionConversionFactor(config.kUnitsPerRotation);
        e.setVelocityConversionFactor(config.kUnitsPerRotation);
        if (config.kOffset != Double.NaN) {
            e.setZeroOffset(config.kOffset);
        }
        e.setAverageDepth(config.kAvgSamplingDepth);

        motor.setPeriodicFramePeriodSec(PeriodicFrame.kStatus5, config.kPeriodicFrameTime);
        motor.setPeriodicFramePeriodSec(PeriodicFrame.kStatus6, config.kPeriodicFrameTime);

        return e;
    }
}
