package frc.team4276.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDebug {
    public static void outputSmartDashboard(double vx, double vy, double omega){
        Rotation2d angle = new Rotation2d(vx, vy);
        double vel = Math.hypot(vx, vy);

        SmartDashboard.putNumber("Debug/Swerve Output Direction", angle.getDegrees());
        SmartDashboard.putNumber("Debug/Swerve Output Velocity", vel);
        SmartDashboard.putNumber("Debug/Swerve Output Omega", omega);
    }
}
