package frc.team4276.lib.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;

import frc.team4276.lib.rev.RevUtil.SparkAbsoluteEncoderConfig;

public class CANSparkMaxFactory {
    /**
     * Set the periodic frame period when not using the internal encoder
     */
    public static VIKCANSparkMax createDefault(int id){
        VIKCANSparkMax sparkMax = new VIKCANSparkMax(id);

        sparkMax.restoreFactoryDefaults();
        sparkMax.clearFaults();

        sparkMax.enableVoltageCompensation(12.0);

        return sparkMax;
    }

    public static VIKCANSparkMaxServo createDefaultServo(int id){
        VIKCANSparkMaxServo sparkMax = new VIKCANSparkMaxServo(id);

        sparkMax.restoreFactoryDefaults();
        sparkMax.clearFaults();

        sparkMax.enableVoltageCompensation(12.0);

        return sparkMax;
    }

    public static VIKCANSparkMax createDefaultFollower(int id, VIKCANSparkMax master){
        return createDefaultFollower(id, master, false);
    }

    /**
     * @param id
     * @param master
     * @param isInverted Relative to master
     * @return
     */
    public static VIKCANSparkMax createDefaultFollower(int id, VIKCANSparkMax master, boolean isInverted){
        VIKCANSparkMax sparkMax = createDefault(id);

        sparkMax.follow(master, isInverted);

        master.queuePeriodicFramePeriodSec(PeriodicFrame.kStatus0, 0.005);

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
        public double kPIDOutputRange = 0.0;
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
        controller.setOutputRange(-1.0 * config.kPIDOutputRange, config.kPIDOutputRange, config.kSlotId);
    }

    public static void configAnalogSensor(VIKCANSparkMax motor, double periodSec) {
        motor.queuePeriodicFramePeriodSec(PeriodicFrame.kStatus3, periodSec);

    }

    public static void configAlternateEncoder(VIKCANSparkMax motor, double periodSec) {
        motor.queuePeriodicFramePeriodSec(PeriodicFrame.kStatus4, periodSec);

    }

    public static AbsoluteEncoder configAbsoluteEncoder(VIKCANSparkMax motor, SparkAbsoluteEncoderConfig config) {
        AbsoluteEncoder e = motor.getAbsoluteEncoder();

        e.setInverted(config.kIsInverted);
        e.setPositionConversionFactor(config.kUnitsPerRotation);
        e.setVelocityConversionFactor(config.kUnitsPerRotation);
        if(config.kOffset != Double.NaN){
            e.setZeroOffset(config.kOffset);
        }
        e.setAverageDepth(config.kAvgSamplingDepth);
        
        // motor.queuePeriodicFramePeriodSec(PeriodicFrame.kStatus5, config.kPeriodicFrameTime);
        // motor.queuePeriodicFramePeriodSec(PeriodicFrame.kStatus6, config.kPeriodicFrameTime);

        return e;
    }
}
