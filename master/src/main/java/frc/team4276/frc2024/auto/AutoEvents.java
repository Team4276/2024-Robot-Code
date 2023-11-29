package frc.team4276.frc2024.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoEvents {
    public static HashMap<String, Command> eventMap = new HashMap<>();

    private static boolean event = false;

    private static boolean isInit = false;

    public static void init(){
        if(!isInit){
            eventMap.put("event", new InstantCommand(() -> event = true));

            isInit = true;
        }
        
    }

    public static boolean getEvent(){
        if (event){
            event = false;
            return true;
        }

        return false;
    }



    
}
