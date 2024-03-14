package frc.team4276.lib.drivers;

import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class SparkProfiledController {
    private SparkPIDController mController;
    private ArmFeedforward mFeedforward;

    public enum FFType{
        SIMPLE,
        ARM,
        FOUR_BAR
    }
    
    public SparkProfiledController(CANSparkBase device, double kS, double kG, double kV, double kA){
        mFeedforward = new ArmFeedforward(kS, kG, kV, kA);
        mController = device.getPIDController();
        
    }

    public SparkPIDController getPID(){
        return mController;
    }

    public ArmFeedforward getFF(){
        return mFeedforward;
    }

    public void setReference(double position){
        mController.setReference(0, ControlType.kSmartMotion, 0, mFeedforward.calculate(position, 0), ArbFFUnits.kVoltage);
        
    }
}
