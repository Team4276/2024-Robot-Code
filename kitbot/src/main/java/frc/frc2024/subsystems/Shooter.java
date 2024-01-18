package frc.frc2024.subsystems;

import static frc.frc2024.Constants.LauncherConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    private CANSparkMax mMotor;
    private SparkPIDController mMotorController;

    private double des_speed = 0;
    
    private Shooter(){
        mMotor = new CANSparkMax(LauncherConstants.kShooterID, MotorType.kBrushless);

        mMotor.setSmartCurrentLimit(LauncherConstants.kCurrentLimit);
        mMotor.setIdleMode(IdleMode.kBrake);

        // mMotorController = mMotor.getPIDController();
        // mMotorController

        mMotor.burnFlash();
    }

    private static Shooter mInstance;

    public static Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public synchronized void update(){
        mMotor.set(des_speed);
    }

    public void setSpeed(double speed){
        des_speed = speed;
    }


}
