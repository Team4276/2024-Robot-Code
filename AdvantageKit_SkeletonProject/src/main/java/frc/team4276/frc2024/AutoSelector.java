package frc.team4276.frc2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.lib.util.VirtualSubsystem;

public class AutoSelector extends VirtualSubsystem { //TODO: impl
    public AutoSelector(){

    }
    
    public Command getCommand(){
        return Commands.none();
    }

    @Override
    public void periodic() {
        
    }
}
