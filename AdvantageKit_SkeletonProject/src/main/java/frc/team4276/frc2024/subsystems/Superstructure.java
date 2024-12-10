package frc.team4276.frc2024.subsystems;

import frc.team4276.frc2024.subsystems.arm.Arm;
import frc.team4276.frc2024.subsystems.feedtake.Feedtake;
import frc.team4276.frc2024.subsystems.flywheels.Flywheels;

public class Superstructure {
    private final Arm arm;
    private final Flywheels flywheels;
    private final Feedtake feedtake;
    

    public Superstructure(Arm arm, Flywheels flywheels, Feedtake feedtake){
        this.arm = arm;
        this.flywheels = flywheels;
        this.feedtake = feedtake;

    }



}
