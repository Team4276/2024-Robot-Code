package frc.team4276.frc2024.statemachines;

import frc.team254.lib.util.Util;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
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

    public enum FourbarTol{
        LIBERAL(SuperstructureConstants.kLiberalFourbarPositionTolerance, SuperstructureConstants.kLiberalFourbarVelocityTolerance),
        MODERATE(SuperstructureConstants.kModerateFourbarPositionTolerance, SuperstructureConstants.kModerateFourbarVelocityTolerance),
        CONSERVATIVE(SuperstructureConstants.kConservativeFourbarPositionTolerance, SuperstructureConstants.kConservativeFourbarVelocityTolerance);

        double pos;
        double vel;

        FourbarTol(double pos, double vel){
            this.pos = pos;
            this.vel = vel;
        }
    }

    public enum FourbarSpeed{
        SLOW,
        MEDIUM,
        SONIC
    }

    public double kFourbarPositionTolerance;
    public double kFourbarVelocityTolerance;

    private SuperstructureState(){}

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState, boolean isShootingState, FourbarTol fourbarTol){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
        this.isShootingState = isShootingState;

        kFourbarPositionTolerance = fourbarTol.pos;
        kFourbarVelocityTolerance = fourbarTol.vel;
    }

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState, boolean isShootingState){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
        this.isShootingState = isShootingState;

        if(isShootingState){
            kFourbarPositionTolerance = SuperstructureConstants.kConservativeFourbarPositionTolerance;
            kFourbarVelocityTolerance = SuperstructureConstants.kConservativeFourbarVelocityTolerance;
        } else {
            kFourbarPositionTolerance = SuperstructureConstants.kModerateFourbarPositionTolerance;
            kFourbarVelocityTolerance = SuperstructureConstants.kModerateFourbarVelocityTolerance;
        }
    }

    public SuperstructureState(double fourbar_angle, IntakeState intakeState, 
        FlywheelState flywheelState){
        this.fourbar_angle = fourbar_angle;
        this.intake_state = intakeState;
        this.flywheel_state = flywheelState;
    }

    public boolean isInRange(SuperstructureState other, double fourbarDeadbandRadians){
        return Util.epsilonEquals(this.fourbar_angle, other.fourbar_angle, fourbarDeadbandRadians) &&
            this.intake_state == other.intake_state &&
            this.flywheel_state.isInRange(other.flywheel_state);
    }

    public boolean isInRange(SuperstructureState other, double fourbarVelocity, double fourbarDeadbandRadians){
        return Util.epsilonEquals(this.fourbar_angle, other.fourbar_angle, fourbarDeadbandRadians) &&
            this.intake_state == other.intake_state &&
            this.flywheel_state.isInRange(other.flywheel_state) && Math.abs(fourbarVelocity) < Math.PI / 90.0;
    }

    /**
     * For scoring
     */
    public boolean isInRange(double fourbar_angle, FlywheelState flywheel_state, double fourbarVelocity, double fourbarDeadbandRadians){
        return Util.epsilonEquals(this.fourbar_angle, fourbar_angle, fourbarDeadbandRadians) &&
            this.flywheel_state.isInRange(flywheel_state) && Math.abs(fourbarVelocity) < Math.PI / 90.0;
    }
}
