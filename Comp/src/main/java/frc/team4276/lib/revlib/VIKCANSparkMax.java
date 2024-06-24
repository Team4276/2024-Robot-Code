package frc.team4276.lib.revlib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import frc.team4276.frc2024.Logging.PrintLogger;

public class VIKCANSparkMax extends CANSparkMax {
    public VIKCANSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public REVLibError restoreFactoryDefaults() {
        REVLibError status = super.restoreFactoryDefaults();
        PrintLogger.print(status.toString());
        return status;
    }

    public REVLibError enableVoltageCompensation(CANSparkMax motor, double volts) {
        REVLibError status = super.enableVoltageCompensation(volts);
        PrintLogger.print(status.toString());
        return status;
    }

    public REVLibError setSmartCurrentLimit(CANSparkMax motor, int limit) {

        REVLibError status = super.setSmartCurrentLimit(limit);
        PrintLogger.print(status.toString());
        return status;
    }

    public REVLibError setIdleMode(CANSparkMax motor, IdleMode mode) {

        REVLibError status = super.setIdleMode(mode);
        PrintLogger.print(status.toString());
        return status;
    }

}
