package frc.team4276.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.team4276.frc2024.Constants.SkidConstants;
import frc.team4276.frc2024.subsystems.Subsystem;

public class MAXSkid extends Subsystem{
    private CANSparkMax lMotor;
    private CANSparkMax fMotor;

    private RelativeEncoder encoder;

    private SparkMaxPIDController pidController;

    public MAXSkid(int id1, int id2, boolean isInverted){
        lMotor = new CANSparkMax(id1, MotorType.kBrushless);        
        fMotor = new CANSparkMax(id2, MotorType.kBrushless);

        lMotor.restoreFactoryDefaults();
        fMotor.restoreFactoryDefaults();

        lMotor.setIdleMode(SkidConstants.kIdleMode);
        fMotor.setIdleMode(SkidConstants.kIdleMode);

        lMotor.setSmartCurrentLimit(SkidConstants.kCurrentLimit);
        fMotor.setSmartCurrentLimit(SkidConstants.kCurrentLimit);

        lMotor.setInverted(isInverted);
        //TODO: do i need to do this? CHECK IT AS SOON AS POSSIBLE
        fMotor.setInverted(isInverted);

        fMotor.follow(lMotor);

        encoder = lMotor.getEncoder();
        encoder.setPositionConversionFactor(SkidConstants.kPositionFactor);
        encoder.setVelocityConversionFactor(SkidConstants.kPositionFactor / 60);

        pidController = lMotor.getPIDController();
        pidController.setFeedbackDevice(encoder);
        pidController.setP(SkidConstants.kDrivingP);
        pidController.setI(SkidConstants.kDrivingI);
        pidController.setD(SkidConstants.kDrivingD);
        pidController.setFF(SkidConstants.kDrivingFF);
        pidController.setOutputRange(-1,1);

        lMotor.burnFlash();
        fMotor.burnFlash();
    }

    public void setSpeed(double speedMetersPerSecond){

    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void resetEncoder(){
        encoder.setPosition(0);
    }


}