package frc.team4276.frc2024.statemachines;

import frc.team254.lib.util.Util;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;

public class SuperstructureState {
    private static SuperstructureState kIdentity = new SuperstructureState();

    public static SuperstructureState identity(){
        return kIdentity;
    }

    public double fourbar_angle;
    public IntakeState intake_state;
    public FlywheelState flywheel_state;
    public boolean isShootingState;

    private SuperstructureState(){}

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState, boolean isShootingState){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
        this.isShootingState = isShootingState;
    }

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
    }

    public boolean isInRange(SuperstructureState other, double fourbarDeadbandDegrees){
        return Util.epsilonEquals(this.fourbar_angle, other.fourbar_angle, fourbarDeadbandDegrees) &&
            this.intake_state == other.intake_state &&
            this.flywheel_state.isInRange(other.flywheel_state);
    }

    /**
     * For scoring
     */
    public boolean isInRange(double fourbar_angle, FlywheelState flywheel_state, double fourbarDeadbandDegrees){
        return Util.epsilonEquals(this.fourbar_angle, fourbar_angle, fourbarDeadbandDegrees) &&
            this.flywheel_state.isInRange(flywheel_state);
    }
}
