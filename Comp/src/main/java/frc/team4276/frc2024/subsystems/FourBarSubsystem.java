package frc.team4276.frc2024.subsystems;

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

    private FourBarSubsystem(){
        super(SuperstructureConstants.kFourBarConstants);
    }
    
    

    

}
