package frc.team4276.frc2024.subsystems.drive.flywheel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.subsystems.drive.flywheel.Flywheel.Goal;
// TODO: need to add manual volatge control, proper logging, characterization, and atGoal()

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();
  private SimpleMotorFeedforward mTopFF =
      new SimpleMotorFeedforward(
          FlywheelConstants.kS_Top, FlywheelConstants.kV_Top, FlywheelConstants.kA);
  private SimpleMotorFeedforward mBottomFF =
      new SimpleMotorFeedforward(
          FlywheelConstants.kS_Bottom, FlywheelConstants.kV_Bottom, FlywheelConstants.kA);
  ;

  private Goal goal = Goal.IDLE;
  private boolean openLoop;

  public enum Goal {
    // TODO: maybe move to constants

    IDLE(0.0, 0.0),
    // TODO: impl when we have a supplier for the needed rpms just put a command to get top and
    // command to get bottom instead of the zeros
    SHOOT(0.0, 0.0);
    public double RPM_TOP, RPM_BOTTOM;

    Goal(double RPM_TOP, double RPM_BOTTOM) {
      this.RPM_TOP = RPM_TOP;
      this.RPM_BOTTOM = RPM_BOTTOM;
    }
  }

  public Flywheel(FlywheelIO io) {
    this.io = io;
    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }
    if (!openLoop) {
      io.runVelocity(
          mTopFF.calculate(this.goal.RPM_TOP), mBottomFF.calculate(this.goal.RPM_BOTTOM));
    } else {

    }
  }

  private void setGoal(Goal goal) {
    openLoop = false;
    this.goal = goal;
  }

  private boolean atGoal() {
    return goal == Goal.IDLE;
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }
}
