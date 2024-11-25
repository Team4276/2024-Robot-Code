package frc.team4276.frc2024.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.subsystems.drive.controllers.HeadingController;
import frc.team4276.frc2024.subsystems.drive.controllers.TeleopDriveController;
import frc.team4276.frc2024.subsystems.drive.controllers.TrajectoryController;

public class Drive extends SubsystemBase {
    public enum DriveMode {
        /** Driving with input from driver joysticks. (Default) */
        TELEOP,

        /** Driving based on a trajectory. */
        TRAJECTORY,

        /** Driving to a location on the field automatically. */
        AUTO_ALIGN
    }

    private DriveMode mMode = DriveMode.TELEOP;

    private final GyroIO mGyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] mModules = new Module[4];

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    private final TeleopDriveController mTeleopDriveController;
    private final HeadingController mHeadingController;
    private final TrajectoryController mTrajectoryController;

    private boolean isHeadingControlled = false;

    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        mGyroIO = gyroIO;
        mModules[0] = new Module(fl, 0);
        mModules[1] = new Module(fr, 1);
        mModules[2] = new Module(bl, 2);
        mModules[3] = new Module(br, 3);

        mTeleopDriveController = new TeleopDriveController();
        mHeadingController = new HeadingController();
        mTrajectoryController = new TrajectoryController();
    }

    @Override
    public void periodic() { //TODO: add odom
        mGyroIO.updateInputs(gyroInputs);

        for(Module module : mModules){
            module.updateInputs();
        }

        Pose2d latestFieldToVehicle = RobotState.getInstance().getLatestFieldToVehicle();

        switch (mMode) {
            case TELEOP:
                desiredSpeeds = mTeleopDriveController.update(latestFieldToVehicle.getRotation());

                if(isHeadingControlled){
                    isHeadingControlled = desiredSpeeds.omegaRadiansPerSecond < 0.25;

                    desiredSpeeds.omegaRadiansPerSecond = mHeadingController.update(latestFieldToVehicle.getRotation().getRadians());

                }

            var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxVel);

            desiredSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states);
                
                break;
        
            case TRAJECTORY:
                desiredSpeeds = mTrajectoryController.update(latestFieldToVehicle, Timer.getFPGATimestamp());
                
                break;
                
            case AUTO_ALIGN:
                
                break;

            default:
                break;
        }

        ChassisSpeeds wanted_speeds = ChassisSpeeds.discretize(desiredSpeeds, Constants.kLooperDt);
    
        moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(wanted_speeds);
        
        SmartDashboard.putNumber("Comp/Des X Speed: ", wanted_speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Y Speed: ", wanted_speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Rot Speed: ", wanted_speeds.omegaRadiansPerSecond);

        SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

        for (int i = 0; i < mModules.length; i++){
            optimizedDesiredStates[i] = SwerveModuleState.optimize(moduleStates[i], mModules[i].getAngle());

            mModules[i].runSetpoint(optimizedDesiredStates[i]);
        }

        Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedDesiredStates);
    }

    public void feedTeleopInput(double controllerX, double controllerY, double controllerOmega) {
        if(!DriverStation.isTeleopEnabled()) return;

        if(mMode != DriveMode.AUTO_ALIGN){
            mMode = DriveMode.TELEOP;
        }

        mTeleopDriveController.feedDriveInput(controllerX, controllerY, controllerOmega);
    }

    public void setHeadingControlled(boolean isHeadingControlled){
        this.isHeadingControlled = isHeadingControlled;
    }

    public void setHeadingGoal(DoubleSupplier headingGoal){
        mHeadingController.setTarget(headingGoal);
        isHeadingControlled = true;
    }
}
