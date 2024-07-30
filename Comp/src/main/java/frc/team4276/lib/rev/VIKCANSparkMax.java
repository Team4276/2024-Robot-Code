package frc.team4276.lib.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

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

    public void setReference(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward, SparkPIDController.ArbFFUnits arbFFUnits) {
        getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

    public void setPeriodicFramePeriodSec(PeriodicFrame frame, double periodSec) {
        setPeriodicFramePeriod(frame, (int)(periodSec * 1000));
    }

}
