package frc.frc2024.subsystems;

public class Drive {
    //TODO: FIX ALL THE CLASSES

    private static Drive mInstance;

    public static Drive getInstance(){
        if (mInstance == null){
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive(){

    }
    
}
