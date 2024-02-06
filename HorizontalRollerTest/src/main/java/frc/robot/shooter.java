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

    public shooter(){
        m1 = new CANSparkMax(20, MotorType.kBrushless);
        m2 = new CANSparkMax(18, MotorType.kBrushless);
        
        m1.setIdleMode(IdleMode.kCoast);
        m2.setIdleMode(IdleMode.kCoast);

        m1.setSmartCurrentLimit(40);
        m2.setSmartCurrentLimit(40);

        p1 = m1.getPIDController();
        p2 = m2.getPIDController();

        r1 = m1.getEncoder();
        r2 = m2.getEncoder();

        p1.setFeedbackDevice(r1);
        p2.setFeedbackDevice(r2);

        p1.setOutputRange(-1, 1);
        p2.setOutputRange(-1, 1);

        p1.setP(0.01);
        p1.setI(0.0);
        p1.setD(0.001);
        p2.setP(0.01);
        p2.setI(0.0);
        p2.setD(0.001);

        m1.burnFlash();
        m2.burnFlash();
    }

    public void setReference(int rpm){
        p1.setReference(rpm, ControlType.kSmartVelocity);
        p2.setReference(-rpm, ControlType.kSmartVelocity);
    }

    public void setSpeed(double speed){
        m1.set(speed);
        m2.set(-speed);
    }

    public boolean atSetpoint(){
        return r1.getVelocity() - r2.getVelocity() < 10;
    }

    public void outputVelocities(){
        SmartDashboard.putNumber("R1", r1.getVelocity());
        SmartDashboard.putNumber("R2", r2.getVelocity());
    }


}
