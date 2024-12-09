package frc.team4276.frc2024;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private Feedtake feedtake;
    private Arm arm;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final DigitalInput armCoastDio = new DigitalInput(Ports.ARM_COAST_SWITCH);

    public RobotContainer() {
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getType()){
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

    private void bindShooting() {

    }

    private void bindDrive() {

    }

    private void bindIntake() {

    }

    private void bindFlywheelSysID() {
        // hold each button until the motor stops moving. Press the buttons in the order
        // they are set here
        driver.a().whileTrue(flywheels.sysIdQuasistatic(Direction.kForward));
        driver.b().whileTrue(flywheels.sysIdQuasistatic(Direction.kReverse));
        driver.rightBumper().whileTrue(flywheels.sysIdDynamic(Direction.kForward));
        driver.leftBumper().whileTrue(flywheels.sysIdDynamic(Direction.kReverse));
    }

    private void configureButtonBinds() {
        // TODO: impl
        if (!Constants.SysIdMode) {
            bindShooting();
            bindDrive();
            bindIntake();
        } else {
            bindFlywheelSysID();
        }

        drive.setDefaultCommand(drive.run(() -> drive.feedTeleopInput(
                -driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));

        driver.rightTrigger().whileTrue(
                Commands.startEnd(
                        () -> drive.setHeadingGoal(
                                () -> robotState.getSpeakerAimingParameters().getDriveHeading().getRadians()),
                        () -> drive.setHeadingControlled(false)));
    }
}
