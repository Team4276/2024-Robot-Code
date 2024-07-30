package frc.team4276.lib.rev;

import com.revrobotics.CANSparkMax;

public class VIKCANSparkMax extends CANSparkMax {
    public VIKCANSparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    public double getAppliedVoltage() {
        return getAppliedOutput() * getBusVoltage();
    }

    public void setWantBrakeMode(boolean brake){
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;

        setIdleMode(mode);
    }

}
