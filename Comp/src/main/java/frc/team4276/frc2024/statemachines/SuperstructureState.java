package frc.team4276.frc2024.statemachines;

import frc.team254.lib.util.Util;

import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;

public class SuperstructureState {
    public double fourbar_angle = SuperstructureConstants.kFourBarConstants.kHomePosition;
    public FlywheelState flywheel_state = new FlywheelState();
    public IntakeState intake_state = IntakeState.IDLE;

    public FourbarSpeed fourbar_speed = FourbarSpeed.SLOW;
    public FourbarTolerance fourbar_tolerance = FourbarTolerance.LIBERAL;
    public FlywheelTolerance flywheel_tolerance = FlywheelTolerance.LIBERAL;

    public Action action = Action.NEUTRAL;

    public enum Action {
        NEUTRAL,
        INTAKE,
        REV_UP,
        DYNAMIC,
        POSITION,
        SCORE,
        OVERRIDE

    }

    public enum FourbarSpeed {
        SLOW(SuperstructureConstants.kSlowFourbarSpeed),
        MEDIUM(SuperstructureConstants.kMediumFourbarSpeed),
        SONIC(SuperstructureConstants.kSonicFourbarSpeed);

        public double max_speed;

        FourbarSpeed(double max_speed) {
            this.max_speed = max_speed;
        }
    }

    public enum FourbarTolerance {
        LIBERAL(SuperstructureConstants.kLiberalFourbarTolerance),
        MODERATE(SuperstructureConstants.kModerateFourbarTolerance),
        CONSERVATIVE(SuperstructureConstants.kConservativeFourbarTolerance);

        public double tol;

        FourbarTolerance(double tol) {
            this.tol = tol;
        }
    }

    public enum FlywheelTolerance {
        LIBERAL(SuperstructureConstants.kLiberalFlywheelTolerance),
        MODERATE(SuperstructureConstants.kModerateFlywheelTolerance),
        CONSERVATIVE(SuperstructureConstants.kConservativeFlywheelTolerance);

        public double tol;

        FlywheelTolerance(double tol) {
            this.tol = tol;
        }
    }

    /**
     * Constructs a superstructure goal state
     * 
     * @param fourbar_angle      degrees
     * @param flywheel_state
     * @param intake_state
     * @param fourbar_speed
     * @param fourbar_tolerance
     * @param flywheel_tolerance
     * @param action
     */
    public SuperstructureState(double fourbar_angle, FlywheelState flywheel_state, IntakeState intake_state,
            FourbarSpeed fourbar_speed, FourbarTolerance fourbar_tolerance, FlywheelTolerance flywheel_tolerance,
            Action action) {
        this.fourbar_angle = Math.toRadians(fourbar_angle);
        this.flywheel_state = flywheel_state;
        this.intake_state = intake_state;
        this.fourbar_speed = fourbar_speed;
        this.fourbar_tolerance = fourbar_tolerance;
        this.flywheel_tolerance = flywheel_tolerance;
        this.action = action;
    }

    /**
     * Constructs a superstructure goal state
     * 
     * @param fourbar_angle      degrees
     * @param flywheel_state
     * @param intake_state
     * @param fourbar_speed
     * @param fourbar_tolerance
     * @param flywheel_tolerance
     */
    public SuperstructureState(double fourbar_angle, FlywheelState flywheel_state, IntakeState intake_state,
            FourbarSpeed fourbar_speed, FourbarTolerance fourbar_tolerance, FlywheelTolerance flywheel_tolerance) {
        this.fourbar_angle = Math.toRadians(fourbar_angle);
        this.flywheel_state = flywheel_state;
        this.intake_state = intake_state;
        this.fourbar_speed = fourbar_speed;
        this.fourbar_tolerance = fourbar_tolerance;
        this.flywheel_tolerance = flywheel_tolerance;
    }

    /**
     * Use for measured states
     * 
     * @param fourbar_angle radians
     */
    public SuperstructureState(double fourbar_angle, FlywheelState flywheel_state, IntakeState intake_state) {
        this.fourbar_angle = fourbar_angle;
        this.flywheel_state = flywheel_state;
        this.intake_state = intake_state;
    }

    public boolean isFourbarInRange(SuperstructureState other) {
        return Util.epsilonEquals(fourbar_angle, other.fourbar_angle, fourbar_tolerance.tol);
    }

}
