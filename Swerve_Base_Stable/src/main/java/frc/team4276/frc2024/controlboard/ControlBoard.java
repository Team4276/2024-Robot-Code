package frc.team4276.frc2024.controlboard;

import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team1678.lib.Util;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    public final BetterXboxController driver;
    public final BetterXboxController operator;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard() {
        driver = new BetterXboxController(OIConstants.kDriverControllerPort);
        operator = new BetterXboxController(OIConstants.kOpControllerPort);
    }

    public Translation2d getSwerveTranslation() {
        double forwardAxis = -driver.getController().getLeftY();
        double strafeAxis = -driver.getController().getLeftX();

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < OIConstants.kJoystickDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, OIConstants.kJoystickDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y)
                    .scale(DriveSubsystem.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = -driver.getRightX();

        if (Math.abs(rotAxis) < OIConstants.kJoystickDeadband) {
            return 0.0;
        } else {
            return DriveSubsystem.getInstance().getKinematicLimits().kMaxAngularVelocity
                    * (rotAxis - (Math.signum(rotAxis) * OIConstants.kJoystickDeadband))
                    / (1 - OIConstants.kJoystickDeadband);
        }
    }

}