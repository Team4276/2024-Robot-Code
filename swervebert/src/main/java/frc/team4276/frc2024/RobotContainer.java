// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.team4276.frc2024.auto.AutoPicker;
import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.controlboard.BetterXboxController;
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

    // The driver's controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final BetterXboxController m_bDriverController = new BetterXboxController(m_driverController);

    private final XboxController m_opController = new XboxController(OIConstants.kopControllerPort);
    private final BetterXboxController m_BetterXboxController = new BetterXboxController(m_opController);

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
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kJoystickDeadband),
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
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new Trigger(m_driverController::getLeftBumper)
                .onTrue(new InstantCommand(() -> m_robotDrive.shiftSpeedDown()));

        new Trigger(m_driverController::getRightBumper)
                .onTrue(new InstantCommand(() -> m_robotDrive.shiftSpeedUp()));

        
        new Trigger(m_driverController::getAButton)
                .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

        new Trigger(m_bDriverController::getLT)
                .whileTrue(new RunCommand(() -> 
                m_robotDrive.snapDrive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
                                0,
                                true, false)));
        
        new Trigger(m_bDriverController::getRT)
                .whileTrue(new RunCommand(() -> 
                        m_robotDrive.snapDrive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kJoystickDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kJoystickDeadband),
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
