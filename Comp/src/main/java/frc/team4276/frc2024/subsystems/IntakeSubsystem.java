package frc.team4276.frc2024.subsystems;

import frc.team4276.lib.drivers.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private static IntakeSubsystem mInstance;

    public static synchronized IntakeSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new IntakeSubsystem();
        }

        return mInstance;
    }

    private IntakeSubsystem(){}
    
}
