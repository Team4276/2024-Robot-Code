package frc.team4276.lib.drivers;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team4276.frc2024.subsystems.IntakeSubsystem;


public class FeederSequence {
    public static void feederSequence(IntakeSubsystem mIntakeSubsystem, DigitalInput input){
        while(!input.get()){
            mIntakeSubsystem.reverse(0.1);
        }
        while(input.get()){
            mIntakeSubsystem.intake();
        }
    }
}
