package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class shooter {
    private CANSparkMax m1;
    private CANSparkMax m2;

    private SparkPIDController p1;
    private SparkPIDController p2;

    private RelativeEncoder r1;
    private RelativeEncoder r2;

    private SimpleMotorFeedforward ff_top;
    private SimpleMotorFeedforward ff_bottom;

    private boolean isPID = false;

    public shooter(){
        m1 = new CANSparkMax(18, MotorType.kBrushless);
        m2 = new CANSparkMax(20, MotorType.kBrushless);

        m1.setInverted(true);
        m2.setInverted(true);

        p1 = m1.getPIDController();
        p2 = m2.getPIDController();

        r1 = m1.getEncoder();
        r2 = m2.getEncoder();

        m1.restoreFactoryDefaults();
        m2.restoreFactoryDefaults();
        
        m1.setIdleMode(IdleMode.kBrake);
        m2.setIdleMode(IdleMode.kBrake);

        m1.setSmartCurrentLimit(60);
        m2.setSmartCurrentLimit(60);

        m1.enableVoltageCompensation(12);
        m2.enableVoltageCompensation(12);

        r1.setAverageDepth(2);
        r2.setAverageDepth(2);

        r1.setMeasurementPeriod(10);
        r2.setMeasurementPeriod(10);

        r1.setVelocityConversionFactor(1);
        r2.setVelocityConversionFactor(1);

        p1.setFeedbackDevice(r1);
        p2.setFeedbackDevice(r2);

        p1.setP(ShooterConstants.kP);
        p1.setI(ShooterConstants.kI);
        p1.setD(ShooterConstants.kD);
        p2.setP(ShooterConstants.kP);
        p2.setI(ShooterConstants.kI);
        p2.setD(ShooterConstants.kD);

        m1.burnFlash();
        m2.burnFlash();

        ff_top = new SimpleMotorFeedforward(ShooterConstants.kS_Top,ShooterConstants.kV,ShooterConstants.kA);
        ff_bottom = new SimpleMotorFeedforward(ShooterConstants.kS_Bottom,ShooterConstants.kV,ShooterConstants.kA);
    }

    private double last_des_rpm_t = 0;
    private double last_des_rpm_b = 0;

    public void setReference(double t_rpm, double b_rpm){
        isPID = true;

        last_des_rpm_t = t_rpm;
        last_des_rpm_b = b_rpm;


        p1.setReference(t_rpm, ControlType.kVelocity, 0, -ff_top.calculate(t_rpm), ArbFFUnits.kVoltage);
        p2.setReference(b_rpm, ControlType.kVelocity, 0, -ff_bottom.calculate(b_rpm), ArbFFUnits.kVoltage);
    }

    public void what_the_flip(double t_rpm, double b_rpm){
        isPID = true;

        double top = t_rpm;
        double bottom = b_rpm;

        if (Math.abs(r1.getVelocity()) < 50){
            m1.set(0);
            p2.setReference(bottom, ControlType.kVelocity, 0, -ff_bottom.calculate(b_rpm), ArbFFUnits.kVoltage);
            return;
        }

        last_des_rpm_t = top;
        last_des_rpm_b = bottom;

        p1.setReference(top, ControlType.kVelocity, 0, -ff_top.calculate(t_rpm), ArbFFUnits.kVoltage);
        p2.setReference(bottom, ControlType.kVelocity, 0, -ff_bottom.calculate(b_rpm), ArbFFUnits.kVoltage);
    }

    public void setSpeed(double speed){
        isPID = false;

        m1.set(speed);
        m2.set(-speed);
    }

    public boolean atSetpoint(){
        double vel_diff = Math.abs(r1.getVelocity()) - Math.abs(r2.getVelocity());

        //TRYNDAMERE
        double top_diff = Math.abs(r1.getVelocity() - last_des_rpm_t);

        double bot_diff = Math.abs(r2.getVelocity() - last_des_rpm_b);
        
        return isPID ? (top_diff < 30 && bot_diff < 30) : vel_diff < 50;
    }

    public void outputVelocities(){
        SmartDashboard.putNumber("R1", r1.getVelocity());
        SmartDashboard.putNumber("R2", r2.getVelocity());
        SmartDashboard.putBoolean("Let it rip!!!!!", atSetpoint());
        SmartDashboard.putNumber("Static Calc", staticCalc);
    }

    private static double staticCalc = 0;

    public void decreaseStaticCalc(){
        staticCalc -= 0.0001;
    }

    public void increaseStaticCalc(){
        staticCalc += 0.0001;
    }

    public void staticCalc(){
        m1.set(staticCalc);
        m2.set(staticCalc);
    }

    public void feed(){
        
    }


}
