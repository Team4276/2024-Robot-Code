package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team4276.lib.drivers.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private static IntakeSubsystem mInstance;

    private CANSparkMax mMotor;

    public static synchronized IntakeSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new IntakeSubsystem();
        }

        return mInstance;
    }

    private IntakeSubsystem(){
        mMotor = new CANSparkMax(12, MotorType.kBrushless);
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kCoast);
        mMotor.setSmartCurrentLimit(25);
        mMotor.enableVoltageCompensation(12);
        mMotor.burnFlash();
    }

    public void intake(){
        mMotor.set(1.0);
    }

    public void stop(){
        mMotor.set(0);
    }

    public void reverse(double power){
        mMotor.set(-power / 2);
    }
    
}
