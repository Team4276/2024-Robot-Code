package frc.team4276.frc2024.subsystems;

import frc.team4276.lib.MAXSkid;

public class Test extends Subsystem {
    

    private MAXSkid test;

    private static Test mInstance;

    public static Test getInstance(){
        if (mInstance == null){
            mInstance = new Test();
        }
        return mInstance;
    }

    private Test(){
        test = new MAXSkid(12, 15, true);

    }

    public void set(double speed){
        test.set(speed/2);
    }
}
