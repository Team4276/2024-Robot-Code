package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class motors {
    private CANSparkMax motor1;
    private CANSparkMax motor2;

    private static motors mInstance;

    public static motors getInstance(){
        if(mInstance == null){
            mInstance = new motors();
        }

        return mInstance;
    }

    private motors(){
        motor1 = new CANSparkMax(1, MotorType.kBrushless);
        motor2 = new CANSparkMax(3, MotorType.kBrushless);

        motor1.enableVoltageCompensation(12.0);
        motor2.enableVoltageCompensation(12.0);

        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
    
        motor1.setInverted(false);

        motor2.follow(motor1, false);
    }

    public void run(){
        motor1.setVoltage(1.0);
    }

    public void invertLeader(boolean invert){
        motor1.setInverted(invert);
    }

    public void invertFollower(boolean invert){
        motor2.follow(motor1, invert);
    }

    public void outputTelemetry(){
        SmartDashboard.putNumber("Motor 1 Voltage", motor1.getAppliedOutput() * 12.0);
        SmartDashboard.putNumber("Motor 2 Voltage", motor2.getAppliedOutput() * 12.0);
        
        SmartDashboard.putBoolean("Is Motor 1 Inverted", motor1.getInverted());
        SmartDashboard.putBoolean("Is Motor 2 Inverted", motor2.getInverted());
    }
}
