package frc.team4276.frc2024.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2024.auto.modes.Default;

public class AutoPicker {
    SendableChooser<Command> chooser;

    public followPathWithEvents followPathWithEvents = new followPathWithEvents();

    public AutoPicker(){
        chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("Do nothing", new Default());

        SmartDashboard.putData("Auto: ", chooser);

    }

    public Command getAutoCommand(){
        return chooser.getSelected();
    }

   


}