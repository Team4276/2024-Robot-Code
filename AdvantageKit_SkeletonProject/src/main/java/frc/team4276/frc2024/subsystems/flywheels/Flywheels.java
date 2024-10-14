package frc.team4276.frc2024.subsystems.flywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// TODO: need to add proper logging
import frc.team4276.frc2024.RobotState;
import frc.team4276.lib.Util;

import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelsIOInputsAutoLogged mInputs = new FlywheelsIOInputsAutoLogged();
    private SimpleMotorFeedforward mTopFF = new SimpleMotorFeedforward(
            FlywheelConstants.kS_Top, FlywheelConstants.kV_Top, FlywheelConstants.kA);
    private SimpleMotorFeedforward mBottomFF = new SimpleMotorFeedforward(
            FlywheelConstants.kS_Bottom, FlywheelConstants.kV_Bottom, FlywheelConstants.kA);

    private Goal goal = Goal.IDLE;
    private boolean closedLoop;

    public enum Goal {
        IDLE(() -> 0.0, () -> 0.0),
        SPEAKER(() -> RobotState.getInstance().getSpeakerAimingParameters().flywheelRpm()),
        FERRY(() -> RobotState.getInstance().getFerryAimingParameters().flywheelRpm()),
        AMP(() -> FlywheelConstants.kAmpTopRpm, () -> FlywheelConstants.kAmpBottomRpm),
        PREP(() -> FlywheelConstants.kPrepRpm),
        BLIND_SUB(() -> FlywheelConstants.kBlindSubRpm),
        BLIND_PODIUM(() -> FlywheelConstants.kBlindPodiumRpm),
        BLIND_FERRY(() -> FlywheelConstants.kBlindFerryRpm),
        BLIND_POOP(() -> FlywheelConstants.kBlindPoopRpm),
        CHARACTERIZING(() -> 0.0, () -> 0.0);

        public DoubleSupplier topRpm, bottomRpm;

        public double getTopRpm() {
            return topRpm.getAsDouble();
        }

        public double getBottomRpm() {
            return bottomRpm.getAsDouble();
        }

        Goal(DoubleSupplier topRpmSupplier, DoubleSupplier bottomRpmSupplier) {
            this.topRpm = topRpmSupplier;
            this.bottomRpm = bottomRpmSupplier;
        }

        Goal(DoubleSupplier rpmSupplier) {
            this.topRpm = rpmSupplier;
        }
    }

    public Flywheels(FlywheelIO io) {
        this.io = io;
        setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
    }

    @Override
    public void periodic() {
        io.updateInputs(mInputs);

        if (DriverStation.isDisabled()) {
            setGoal(Goal.IDLE);
        }
        if (closedLoop) {
            io.runVelocity(mTopFF.calculate(this.goal.getTopRpm()), mBottomFF.calculate(this.goal.getBottomRpm()));
        } else if (goal == Goal.IDLE) {
            io.stop();
        }
    }

    private void setGoal(Goal goal) {
        if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
            closedLoop = false;
            this.goal = goal;
            return;
        }
        // TODO: this will run one more time then needed atm need to fix later
        closedLoop = true;
        this.goal = goal;
    }

    public void runCharacterization(double input) {
        setGoal(Goal.CHARACTERIZING);
        io.runCharacterizationTop(input);
        io.runCharacterizationBottom(input);
    }

    public boolean isTopSpunUp() {
        return Util.epsilonEquals(mInputs.topVelocityRpm, goal.getTopRpm(), FlywheelConstants.kFlywheelTolerance)
                && goal.getTopRpm() > 2000;
    }

    public boolean isBottomSpunUp() {
        return Util.epsilonEquals(mInputs.topVelocityRpm, goal.getBottomRpm(), FlywheelConstants.kFlywheelTolerance)
                && goal.getBottomRpm() > 2000;
    }

    public boolean atGoal() {
        return isTopSpunUp() && isBottomSpunUp();
    }

    public Command SpeakerShootCommand() {
        return startEnd(() -> setGoal(Goal.SPEAKER), () -> setGoal(Goal.IDLE))
                .withName("Flywheels Speaker Shoot");
    }

    public Command FerryShootCommand() {
        return startEnd(() -> setGoal(Goal.FERRY), () -> setGoal(Goal.IDLE))
                .withName("Flywheels Ferry Shoot");
    }
}
