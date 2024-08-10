package frc.team4276.frc2024.controlboard;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.IntakeSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

import frc.team1678.lib.Util;

public class ControlBoard { // TODO: config
    public final BetterXboxController driver;
    public final BetterXboxController operator;

    private final DigitalInput climberSetting;
    private final DigitalInput fourbarSetting;

    private DriveSubsystem mDriveSubsystem;
    private Superstructure mSuperstructure;

    private static ControlBoard mInstance;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard() {
        driver = new BetterXboxController(OIConstants.kDriverControllerPort);
        operator = new BetterXboxController(OIConstants.kOpControllerPort);

        climberSetting = new DigitalInput(Ports.CLIMBER_BRAKE_SWITCH);
        fourbarSetting = new DigitalInput(Ports.FOURBAR_BRAKE_SWITCH);

        mDriveSubsystem = DriveSubsystem.getInstance();
        mSuperstructure = Superstructure.getInstance();
    }

    public void update() {
        driver.update();
        operator.update();
    }

    public void updateNominal() {
        mDriveSubsystem.overrideHeading(wantReady());

        mSuperstructure.setDynamic(wantDynamic());

        mSuperstructure.setFerry(wantFerry());

        if (wantStow()) {
            mSuperstructure.setPrep(false);
        } else if (wantPrep()) {
            mSuperstructure.setPrep(true);
        }

        if (wantIdle()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.IDLE);

        } else if (wantIntake()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.INTAKE);

        } else if (wantShoot()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.SHOOT);

        } else if (wantReady()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.READY);

        } else if (wantExhaust()) {
            mSuperstructure.setGoalState(Superstructure.GoalState.EXHAUST);

        } else {
            mSuperstructure.setGoalState(Superstructure.GoalState.STOW);

        }
    }

    public void updateManual() {
        mSuperstructure.setManualFlywheelVoltage(0.0);
        mSuperstructure.setManualFourbarVoltage(0.0);
        mSuperstructure.setManualIntakeState(IntakeSubsystem.State.IDLE);

    }

    // Driver Controls
    public Translation2d getSwerveTranslation() {
        double forwardAxis = -driver.getLeftY();
        double strafeAxis = -driver.getLeftX();

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < OIConstants.kJoystickDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, OIConstants.kJoystickDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y)
                    .scale(DriveSubsystem.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    private double kMaxTurnMagnitude = 0.75;

    public double getSwerveRotation() {
        double rotAxis = -driver.getRightX();

        if (Math.abs(rotAxis) < OIConstants.kJoystickDeadband) {
            return 0.0;
        } else {
            return kMaxTurnMagnitude * DriveSubsystem.getInstance().getKinematicLimits().kMaxAngularVelocity
                    * (rotAxis - (Math.signum(rotAxis) * OIConstants.kJoystickDeadband))
                    / (1 - OIConstants.kJoystickDeadband);
        }
    }

    public boolean wantZeroHeading() {
        return driver.getAButtonPressed();
    }

    public boolean wantXBrake() {
        return driver.getXButton();
    }

    boolean isDemo = false;
    boolean hasReleased = false;

    public boolean wantDemoLimits() {
        if (!driver.isPOVUPPressed()) {
            hasReleased = true;

        }

        if (driver.isPOVUPPressed() && hasReleased) {
            hasReleased = false;
            isDemo = !isDemo;
        }

        return isDemo;
    }

    public boolean wantIntake() {
        return driver.getRT();
    }

    public boolean wantExhaust() {
        return false;
    }

    public boolean wantSlowLowerClimber() {
        return false;
    }

    public boolean wantLowerClimber() {
        return false;
    }

    public boolean wantReady() {
        return driver.getYButton();
    }

    // Operator Controls
    public boolean wantManual() {
        return false;
    }
    
    public boolean wantStow() {
        return false;
    }

    public boolean wantPrep() {
        return false;
    }

    public boolean wantDynamic() {
        return false;
    }

    public boolean wantShoot() {
        return false;
    }

    public boolean wantFerry() {
        return false;
    }

    public boolean wantClimbMode() {
        return false;
    }

    public boolean wantRaiseClimber() {
        return false;
    }

    private boolean wasIdle = false;

    public boolean wantIdle() {
        if (operator.getBButtonPressed()) {
            wasIdle = !wasIdle;
        }

        return wasIdle;
    }

    // Robot Button Board
    public boolean wantClimberCoastMode() {
        return climberSetting.get();
    }

    public boolean wantFourbarCoastMode() {
        return fourbarSetting.get();
    }

}