package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter 
{
    private CANSparkMax launcherMotor;
    private CANSparkMax shooterFeeder;

    public Shooter (){
        launcherMotor = new CANSparkMax(20, MotorType.kBrushless);
        shooterFeeder = new CANSparkMax(18, MotorType.kBrushless);
    }

}