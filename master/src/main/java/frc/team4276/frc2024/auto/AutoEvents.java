package frc.team4276.frc2024.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoEvents {
    public static HashMap<String, Command> eventMap = new HashMap<>();

    public static boolean event;

    public AutoEvents(){
        eventMap.put("event", new InstantCommand(() -> event = true));

    }    

    
}
