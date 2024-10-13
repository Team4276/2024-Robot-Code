package frc.team4276.frc2024.subsystems.flywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// TODO: need to add proper logging
import frc.team4276.frc2024.RobotState;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelsIOInputsAutoLogged mInputs = new FlywheelsIOInputsAutoLogged();
  private SimpleMotorFeedforward mTopFF =
      new SimpleMotorFeedforward(
          FlywheelConstants.kS_Top, FlywheelConstants.kV_Top, FlywheelConstants.kA);
  private SimpleMotorFeedforward mBottomFF =
      new SimpleMotorFeedforward(
          FlywheelConstants.kS_Bottom, FlywheelConstants.kV_Bottom, FlywheelConstants.kA);

  private Goal goal = Goal.IDLE;
  private boolean closedLoop;

  public enum Goal {
    // maybe move to constants

    IDLE(() -> 0.0, () -> 0.0),
    FERRY(
        () -> RobotState.getInstance().getFerryAimingParameters().getFlywheelSpeeds().leftSpeed(),
        () -> RobotState.getInstance().getFerryAimingParameters().getFlywheelSpeeds().rightSpeed()),

    SPEAKER(
        () -> RobotState.getInstance().getFerryAimingParameters().getFlywheelSpeeds().leftSpeed(),
        () -> RobotState.getInstance().getFerryAimingParameters().getFlywheelSpeeds().rightSpeed()),

    CHARACTERIZING(() -> 0.0, () -> 0.0);
    public DoubleSupplier RPM_TOP, RPM_BOTTOM;

    public double getRpmTop() {
      return RPM_BOTTOM.getAsDouble();
    }

    public double getRpmBottom() {
      return RPM_BOTTOM.getAsDouble();
    }

    Goal(DoubleSupplier RPM_TOP, DoubleSupplier RPM_BOTTOM) {
      this.RPM_TOP = RPM_TOP;
      this.RPM_BOTTOM = RPM_BOTTOM;
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
      io.runVelocity(
          mTopFF.calculate(this.goal.getRpmTop()), mBottomFF.calculate(this.goal.getRpmBottom()));
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
    return (Math.abs(mInputs.topVelocityRpm - goal.getRpmTop())
            < FlywheelConstants.kFlywheelTolerance)
        && (goal.getRpmTop() > 2000);
  }

  public boolean isBottomSpunUp() {
    return Math.abs(mInputs.bottomVelocityRpm - goal.getRpmBottom())
            < FlywheelConstants.kFlywheelTolerance
        && (goal.getRpmBottom() > 2000);
  }

  private boolean atGoal() {
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
