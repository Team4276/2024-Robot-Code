package frc.team4276.frc2024;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team4276.frc2024.subsystems.arm.Arm;
import frc.team4276.frc2024.subsystems.arm.ArmConstants;
import frc.team4276.frc2024.subsystems.arm.ArmIOSparkMax;
import frc.team4276.frc2024.subsystems.drive.Drive;
import frc.team4276.frc2024.subsystems.drive.DriveConstants;
import frc.team4276.frc2024.subsystems.drive.GyroIOADIS;
import frc.team4276.frc2024.subsystems.drive.ModuleIOSparkMax;
import frc.team4276.frc2024.subsystems.feedtake.Feedtake;
import frc.team4276.frc2024.subsystems.feedtake.Roller;
import frc.team4276.frc2024.subsystems.feedtake.RollerIOSparkMax;
import frc.team4276.frc2024.subsystems.feedtake.RollerSensorsIOHardware;
import frc.team4276.frc2024.subsystems.feedtake.Feedtake.GamePieceState;
import frc.team4276.frc2024.subsystems.flywheels.FlywheelIOSpark;
import frc.team4276.frc2024.subsystems.flywheels.Flywheels;
import frc.team4276.frc2024.subsystems.vision.Vision;
import frc.team4276.frc2024.subsystems.vision.VisionConstants;
import frc.team4276.frc2024.subsystems.vision.VisionIOPhoton;
import frc.team4276.lib.feedforwards.FourbarFeedForward;

//TODO: finish everything
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    private Drive drive;
    @SuppressWarnings("unused")
    private Vision vision;
    private Flywheels flywheels;
    private Arm arm;
    private Feedtake feedtake;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final DigitalInput armCoastDio = new DigitalInput(Ports.ARM_COAST_SWITCH);

    private final BooleanSupplier disableDriveAim = () -> true;

    private final AutoSelector autoSelector = new AutoSelector();

    public RobotContainer() {
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getType()) {
                case COMPBOT -> {
                    drive = new Drive(
                            new GyroIOADIS(),
                            new ModuleIOSparkMax(DriveConstants.kModuleConfigs[0]),
                            new ModuleIOSparkMax(DriveConstants.kModuleConfigs[1]),
                            new ModuleIOSparkMax(DriveConstants.kModuleConfigs[2]),
                            new ModuleIOSparkMax(DriveConstants.kModuleConfigs[3]));
                    vision = new Vision(new VisionIOPhoton(VisionConstants.kFrontCameraConstants),
                            new VisionIOPhoton(VisionConstants.kBackCameraConstants));
                    flywheels = new Flywheels(new FlywheelIOSpark());
                    feedtake = new Feedtake(new Roller(new RollerIOSparkMax()), new RollerSensorsIOHardware());
                    arm = new Arm(new ArmIOSparkMax(), new FourbarFeedForward(ArmConstants.kFeedForwardConstants));

                }
                case SIMBOT -> {

                }

            }

        }

        arm.setCoastOverride(armCoastDio::get);

        configureAutos();
        configureButtonBinds();
    }

    private void configureAutos() {
        // TODO: impl
    }

    private void bindFlywheelSysID() {
        // hold each button until the motor stops moving. Press the buttons in the order
        // they are set here
        driver.a().whileTrue(flywheels.sysIdQuasistatic(Direction.kForward));
        driver.b().whileTrue(flywheels.sysIdQuasistatic(Direction.kReverse));
        driver.rightBumper().whileTrue(flywheels.sysIdDynamic(Direction.kForward));
        driver.leftBumper().whileTrue(flywheels.sysIdDynamic(Direction.kReverse));
    }

    private void bindShooting() {

    }

    private void bindDrive() {
        drive.setDefaultCommand(drive.run(() -> drive.feedTeleopInput(
                -driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));
        
        driver.x().onTrue(drive.run(() -> RobotState.getInstance().resetHeading(Timer.getFPGATimestamp(), 
            Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Red ? 180 : 0.0))));
    }

    private void bindIntake() {
        driver.leftTrigger().whileTrue(new ParallelCommandGroup(
                Commands.startEnd(
                        () -> arm.setGoalCommand(Arm.Goal.INTAKE),
                        () -> arm.setGoalCommand(Arm.Goal.STOW)),
                feedtake.setGoalCommand(Feedtake.Goal.INTAKE)));
    }

    private void configureButtonBinds() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // ********************** Triggers for Driver Feedback **********************

        // Note Pickup
        new Trigger(() -> feedtake.getGamePieceState() == GamePieceState.STAGED
                && feedtake.getGoal() == Feedtake.Goal.INTAKE)
                .onTrue(controllerRumbleCommand().withTimeout(1.0));

        // Note Grasp
        new Trigger(() -> feedtake.getGamePieceState() == GamePieceState.GRASPED
                && feedtake.getGoal() == Feedtake.Goal.INTAKE)
                .onTrue(controllerRumbleCommand().withTimeout(0.5));

        // Controls

        // TODO: impl
        if (Constants.SysIdMode) {
            bindFlywheelSysID();
        } else {
            bindShooting();
            bindDrive();
            bindIntake();
        }

        Supplier<Command> driveAimCommand = () -> Commands.either(
                Commands.startEnd(
                        () -> drive
                                .setHeadingGoal(robotState.getSpeakerAimingParameters().getDriveHeading()::getRadians),
                        () -> drive.setHeadingControlled(false)),
                Commands.none(),
                disableDriveAim);

        Supplier<Command> speakerAimCommand = () -> Commands.either(
                Commands.startEnd(
                        () -> drive
                                .setHeadingGoal(robotState.getSpeakerAimingParameters().getDriveHeading()::getRadians),
                        () -> drive.setHeadingControlled(false)),
                Commands.none(),
                disableDriveAim);
    }

    /** Creates a controller rumble command with specified rumble and controllers */
    private Command controllerRumbleCommand() {
        return Commands.startEnd(
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                },
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                    operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                });
    }

    public Command getAutoCommand() {
        return autoSelector.getCommand();
    }
}
