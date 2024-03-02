package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class intake {
    private CANSparkMax m1;

    private static intake mInstance;

    public static intake getInstance(){
        if(mInstance == null){
            mInstance = new intake();
        }

        return mInstance;
    }

    public intake(){
        m1 = new CANSparkMax(19, MotorType.kBrushless);

        m1.restoreFactoryDefaults();

        m1.setIdleMode(IdleMode.kBrake);

        m1.setSmartCurrentLimit(20);

        m1.enableVoltageCompensation(12);

        m1.burnFlash();
    }

    public void runIntake(double speed){
        m1.set(speed);
    }

}
