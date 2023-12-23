package frc.team4276.frc2024.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.team4276.lib.MAXSwerveModuleV2.mPeriodicIO;

public class simpleisharm extends Subsystem {
    private CANSparkMax mLeader;

    private SparkMaxAbsoluteEncoder encoder;

    private ProfiledPIDController pidController;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public static class PeriodicIO {
        // Inputs
        public double position = 0.0;

        // Outputs
        public double demand = 0.0;
    }

    public simpleisharm(){
        mLeader = new CANSparkMax(0, MotorType.kBrushless);
        mLeader.setSmartCurrentLimit(40);
        mLeader.setIdleMode(IdleMode.kBrake);
        
        encoder = mLeader.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(1);
        encoder.setVelocityConversionFactor(1);
        encoder.setZeroOffset(0);
        
        pidController.enableContinuousInput(0, 1);
        pidController.setPID(0, 0, 0);
        pidController.setIntegratorRange(0, 0);
        pidController.setTolerance(0, 0);
        pidController.setConstraints(new TrapezoidProfile.Constraints(0, 0));
    }

    private void update(){
        double des_vel = pidController.calculate(mPeriodicIO.position);

        pidController.
        
    }

    @Override
    public void writePeriodicOutputs() {
        
    }

    @Override
    public void readPeriodicInputs() {
        
    }

    @Override
    public void zeroSensors() {
        
    }

    @Override
    public void stop() {
        
    }

}
