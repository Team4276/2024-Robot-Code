package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Test2 {
    private CANSparkMax m1;
    private CANSparkMax m2;

    public Test2(){
        m1 = new CANSparkMax(13, MotorType.kBrushless);
        m2 = new CANSparkMax(8, MotorType.kBrushless);

        m1.restoreFactoryDefaults();
        m2.restoreFactoryDefaults();
    }

    public void set(double speed){
        m1.set(speed/2);
        m2.set(speed/2);
    }
}
