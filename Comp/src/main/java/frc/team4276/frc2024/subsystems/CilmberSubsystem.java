package frc.team4276.frc2024.subsystems;

public class CilmberSubsystem {

    private static CilmberSubsystem mInstance;

    public static CilmberSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new CilmberSubsystem();
        }

        return mInstance;
    }


    
}
