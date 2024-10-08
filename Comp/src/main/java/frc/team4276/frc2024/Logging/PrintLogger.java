package frc.team4276.frc2024.Logging;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2024.Constants.DebugConstants;

//TODO: implement

public class PrintLogger {
    //prints to output set in constants 
    @SuppressWarnings("unused")
    public static void print(String str){
        //no dead code its just because of the condition on a constant
        if(DebugConstants.printOutput == "DriverStation"){
            System.out.println(str);
        } 
        else if(DebugConstants.printOutput == "Standard Out"){
            DriverStation.reportWarning(str, false);
        } 
        else{
            DriverStation.reportWarning("Invalid log output", false);
        }

    }
}
