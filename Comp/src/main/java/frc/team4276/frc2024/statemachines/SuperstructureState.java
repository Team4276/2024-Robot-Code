package frc.team4276.frc2024.statemachines;

import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;

public class SuperstructureState {
    public double fourbar_angle;
    public IntakeState intake_state;
    public FlywheelState flywheel_state;
    public boolean isShootingState;

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState, boolean isShootingState){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
        this.isShootingState = isShootingState;
    }
}
