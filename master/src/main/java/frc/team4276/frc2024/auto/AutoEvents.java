package frc.team4276.frc2024.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoEvents {
    public static HashMap<String, Command> eventMap = new HashMap<>();

    private static boolean event = false;

    private static boolean isInit = false;

    private AutoEvents(){}

    private static void init(){
        eventMap.put("event", new InstantCommand(() -> event = true));
    }

    public static boolean getEvent(){
        if(!isInit){
            init();
            isInit = true;
        }

        if (event){
            event = false;
            return true;
        }

        return false;
    }



    
}
