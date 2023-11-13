// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import edu.wpi.first.wpilibj.XboxController;
import frc.team4276.frc2024.auto.AutoPicker;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();

    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    private final AutoPicker chooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        chooser = new AutoPicker();

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -mControlBoard.driver.getLeftY(),
                                -mControlBoard.driver.getLeftX(),
                                -mControlBoard.driver.getRightX(),
                                true, false),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new Trigger(mControlBoard.driver.getController()::getAButton)
                .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

        new Trigger(mControlBoard.driver::getLT)
                .whileTrue(new RunCommand(() -> 
                m_robotDrive.snapDrive(
                                -mControlBoard.driver.getLeftY(),
                                -mControlBoard.driver.getLeftX(),
                                0,
                                true, false)));
        
        new Trigger(mControlBoard.driver::getRT)
                .whileTrue(new RunCommand(() -> 
                        m_robotDrive.snapDrive(
                                -mControlBoard.driver.getLeftY(),
                                -mControlBoard.driver.getLeftX(),
                                180,
                                true, false)));

        }

        



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getAutoCommand();

        }
}
