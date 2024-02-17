package frc.team4276.frc2024.subsystems;

import frc.team1678.lib.loops.ILooper;
import frc.team4276.lib.drivers.Subsystem;

/** 
 * Use for ServoMotorSubsystems.
 * <p>
 * This architecture isn't necessary if it only uses one ControlState (PID, FF, etc).
*/
public class Superstructure extends Subsystem {
    private final FourBarSubsystem mFourBarSubsystem = FourBarSubsystem.getInstance();

    private double mDesiredVoltage;
    private double mCommandedVoltage;
    private boolean isFourBarVoltageControl = true;

    private static Superstructure mInstance;

    public static Superstructure getInstance(){
        if (mInstance == null){
            mInstance = new Superstructure();
        }

        return mInstance;
    }
    
    public void setFourBarVoltage(double des_voltage){
        mDesiredVoltage = des_voltage;
    }

    // Only place we take inputs from controlboard (other than drive subsystem);
    @Override
    public void readPeriodicInputs() {
        mCommandedVoltage = mDesiredVoltage;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    @Override
    public void writePeriodicOutputs() {
        if (isFourBarVoltageControl){
            mFourBarSubsystem.setVoltage(mCommandedVoltage);
        }
        
    }
}
