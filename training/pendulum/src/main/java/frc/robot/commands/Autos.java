// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmMotorSubsystem;

public final class Autos {

  public static Command exampleAuto(ArmMotorSubsystem armMotorSubsystem) {
    return null;  //Commands.sequence(armMotorSubsystem.setArmPositionCommand(0.0), new ArmSetPositionCommand(armMotorSubsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
