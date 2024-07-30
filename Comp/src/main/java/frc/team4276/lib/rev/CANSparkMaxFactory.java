package frc.team4276.lib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;

public class CANSparkMaxFactory {
    
    /**
     * Set the periodic frame period when not using the internal encoder
     */
    public static VIKCANSparkMax createDefault(int id){
        VIKCANSparkMax sparkMax = new VIKCANSparkMax(id);

        sparkMax.clearFaults();
        sparkMax.restoreFactoryDefaults();

        // Clean up CAN usage
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 1000);

        sparkMax.enableVoltageCompensation(12.0);

        return sparkMax;
    }

    public static VIKCANSparkMaxServo createDefaultServo(int id){
        VIKCANSparkMaxServo sparkMax = new VIKCANSparkMaxServo(id);

        sparkMax.clearFaults();
        sparkMax.restoreFactoryDefaults();

        // Clean up CAN usage
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 1000);

        sparkMax.enableVoltageCompensation(12.0);

        return sparkMax;
    }

    public static VIKCANSparkMax createDefaultFollower(int id, CANSparkMax master){
        VIKCANSparkMax sparkMax = createDefault(id);

        sparkMax.follow(master);

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
        controller.setOutputRange(config.kPIDOutputRange, config.kPIDOutputRange, config.kSlotId);
    }
}
