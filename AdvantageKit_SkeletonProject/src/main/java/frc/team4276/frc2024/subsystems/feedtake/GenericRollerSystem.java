// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.team4276.frc2024.subsystems.feedtake;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollerSystem<G extends GenericRollerSystem.VoltageGoal> {
  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.io = io;

    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }
}
