package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;

public class FourBarSubsystem extends ServoMotorSubsystem {
    private static FourBarSubsystem mInstance;

    public static synchronized FourBarSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new FourBarSubsystem();
        }

        return mInstance;
    }

    // -1 = forward; 1 = backward (on the robot)

    private FourBarSubsystem(){
        super(SuperstructureConstants.kFourBarConstants);

        
        SmartDashboard.putNumber("kS Calibration", 0.0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Fourbar Measured Position Radians", getMeasPosition());
        SmartDashboard.putNumber("Fourbar Measured Velocity Radians", getMeasVelocity());
        SmartDashboard.putNumber("Fourbar Measured Position Degrees", Math.toDegrees(getMeasPosition()));
        SmartDashboard.putNumber("Fourbar Measured Velocity Degrees", Math.toDegrees(getMeasVelocity()));
        SmartDashboard.putString("Fourbar Idle Mode", getIdleMode().name());
        SmartDashboard.putNumber("Fourbar Applied Voltage", getAppliedVoltage());
    }

    @Override
    public synchronized void setFourBarFFSetpoint(double position_rad) {
        if(position_rad == Double.NaN){
            super.setFourBarFFSetpoint(getMeasPosition());
            return;
        }

        super.setFourBarFFSetpoint(position_rad);
    }

    @Override
    public synchronized void setFourBarFFSetpointTEST(double position_rad) {
        if(position_rad == Double.NaN){
            super.setFourBarFFSetpoint(getMeasPosition());
            return;
        }

        super.setFourBarFFSetpoint(position_rad);
    }
}
