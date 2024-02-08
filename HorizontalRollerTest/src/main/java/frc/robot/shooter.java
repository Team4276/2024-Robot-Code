package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class shooter {
    private CANSparkMax m1;
    private CANSparkMax m2;

    private SparkPIDController p1;
    private SparkPIDController p2;

    private RelativeEncoder r1;
    private RelativeEncoder r2;

    //TODO: get static value

    public shooter(){
        m1 = new CANSparkMax(20, MotorType.kBrushless);
        m2 = new CANSparkMax(18, MotorType.kBrushless);

        m1.restoreFactoryDefaults();
        m2.restoreFactoryDefaults();
        
        m1.setIdleMode(IdleMode.kBrake);
        m2.setIdleMode(IdleMode.kBrake);

        m1.setSmartCurrentLimit(60);
        m2.setSmartCurrentLimit(60);

        p1 = m1.getPIDController();
        p2 = m2.getPIDController();

        r1 = m1.getEncoder();
        r2 = m2.getEncoder();

        r1.setVelocityConversionFactor(1);
        r2.setVelocityConversionFactor(1);

        p1.setFeedbackDevice(r1);
        p2.setFeedbackDevice(r2);

        p1.setOutputRange(-1, 1);
        p2.setOutputRange(-1, 1);

        p1.setP(0.1);
        p1.setI(0.0);
        p1.setD(0.0);
        p2.setP(0.1);
        p2.setI(0.0);
        p2.setD(0.0);

        m1.burnFlash();
        m2.burnFlash();
    }

    private double last_des_rpm = 0;

    public void setReference(int rpm){
        last_des_rpm = rpm;

        p1.setReference(rpm, ControlType.kVelocity);
        p2.setReference(-rpm, ControlType.kVelocity);
    }

    public void setSpeed(double speed){
        m1.set(speed);
        m2.set(-speed);
    }

    public boolean atSetpoint(){
        return Math.abs(r1.getVelocity()) - Math.abs(r2.getVelocity()) < 50 
            && (last_des_rpm == 0 ? true : Math.abs(r1.getVelocity() - last_des_rpm) < 30);
    }

    public void outputVelocities(){
        SmartDashboard.putNumber("R1", r1.getVelocity());
        SmartDashboard.putNumber("R2", r2.getVelocity());
        SmartDashboard.putBoolean("Let it rip!!!!!", atSetpoint());
    }


}
