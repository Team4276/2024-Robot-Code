package frc.team4276.frc2024.subsystems.flywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// TODO: need to add manual volatge control, proper logging, characterization

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
    // TODO: maybe move to constants

    IDLE(0.0, 0.0),
    // TODO: impl when we have a supplier for the needed rpms just put a command to
    // get top and
    // command to get bottom instead of the zeros
    SHOOT(0.0, 0.0);
    public double RPM_TOP, RPM_BOTTOM;

    Goal(double RPM_TOP, double RPM_BOTTOM) {
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
    // need to fix open loop logic just temp atm
    if (closedLoop) {
      io.runVelocity(
          mTopFF.calculate(this.goal.RPM_TOP), mBottomFF.calculate(this.goal.RPM_BOTTOM));
    } else if (goal == Goal.IDLE) {
      io.stop();
    }
  }

  private void setGoal(Goal goal) {
    closedLoop = true;
    this.goal = goal;
  }

  public boolean isTopSpunUp() {
    return (Math.abs(mInputs.topVelocityRpm - goal.RPM_TOP) < FlywheelConstants.kFlywheelTolerance)
        && (goal.RPM_TOP > 2000);
  }

  public boolean isBottomSpunUp() {
    return Math.abs(mInputs.bottomVelocityRpm - goal.RPM_BOTTOM)
            < FlywheelConstants.kFlywheelTolerance
        && (goal.RPM_BOTTOM > 2000);
  }

  private boolean atGoal() {
    return isTopSpunUp() && isBottomSpunUp();
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }
}
