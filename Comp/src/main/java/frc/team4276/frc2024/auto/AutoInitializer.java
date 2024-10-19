package frc.team4276.frc2024.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;

public class AutoInitializer {
    private static final RobotState rs = RobotState.getInstance();

    public static void init() {
        AutoBuilder.configure(
                rs::getWPILatestFieldToVehicle, 
                DriveSubsystem.getInstance()::resetOdometryWPI, 
                DriveSubsystem.getInstance()::getWPIMeasSpeeds, 
                DriveSubsystem.getInstance()::updatePPPathFollowingSetpoint, 
                DriveConstants.kAutoDriveControllerConstants, 
                DriveConstants.kPPRobotConfig, 
                AllianceChooser.getInstance()::isAllianceRed, 
                new EmptySubsystem());
    }
    
}
