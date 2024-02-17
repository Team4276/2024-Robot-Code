package frc.team4276.frc2024.subsystems;

import frc.team4276.lib.drivers.Subsystem;

public class FeederSubsystem extends Subsystem {
    private static FeederSubsystem mInstance;

    public static synchronized FeederSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new FeederSubsystem();
        }

        return mInstance;
    }

    private FeederSubsystem(){}
}
