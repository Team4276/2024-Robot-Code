package frc.frc2024.subsystems;

import static frc.frc2024.Constants.LauncherConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

public class Shooter {
    private CANSparkMax mFlywheel;
    private CANSparkMax mLauncher;
    private SparkPIDController mMotorController;

    private double des_fw_speed = 0;
    private double des_f_speed = 0;
    
    private Shooter(){
        mFlywheel = new CANSparkMax(LauncherConstants.kShooterID, MotorType.kBrushless);
        mLauncher = new CANSparkMax(LauncherConstants.kFeederID, MotorType.kBrushed);

        mFlywheel.setSmartCurrentLimit(LauncherConstants.kCurrentLimit);
        mLauncher.setSmartCurrentLimit(LauncherConstants.kCurrentLimit);

        mFlywheel.setIdleMode(IdleMode.kBrake);
        mLauncher.setIdleMode(IdleMode.kBrake);

        // mMotorController = mMotor.getPIDController();
        // mMotorController

        mFlywheel.burnFlash();
        mLauncher.burnFlash();
    }

    private static Shooter mInstance;

    public static Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }

    public synchronized void update(){
        mFlywheel.set(des_fw_speed);
        mLauncher.set(des_f_speed);
    }

    public void setSpeed(double speed){
        des_fw_speed = speed;
    }

    public void feed(){
        des_f_speed = LauncherConstants.kFeederSpeed;
    }

    public void refeed(){
        des_f_speed = LauncherConstants.kReFeederSpeed;
    }

    public void stopfeed(){
        des_f_speed = 0;
    }

    private double wait = 2;
    double start = 0;

    public void startShootSequence(){
        start = Timer.getFPGATimestamp();
    }

    public void shootSequence(){
        setSpeed(LauncherConstants.kLauncherSpeedHigh);
        if (Timer.getFPGATimestamp() >= start + wait){
            feed();
        }
    }




}
