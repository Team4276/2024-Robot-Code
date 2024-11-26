package frc.team4276.frc2024.subsystems.drive;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.subsystems.drive.controllers.*;

public class Drive extends SubsystemBase { //TODO: impl high freq odom reading
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
    private final GyroIOInputsAutoLogged mGyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] mModules = new Module[4];

    private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] mModuleStates = new SwerveModuleState[4];

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
    public void periodic() {
        mGyroIO.updateInputs(mGyroInputs);
        Logger.processInputs("Drive/Gyro", mGyroInputs);

        for(Module module : mModules){
            module.updateInputs();
        }

        SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(
            Arrays.stream(mModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new));

        RobotState.getInstance().addOdomObservations(Timer.getFPGATimestamp(), wheelPositions, mGyroInputs.yawPosition);

        Pose2d latestFieldToVehicle = RobotState.getInstance().getLatestFieldToVehicle();

        ChassisSpeeds uncheckedSpeeds = new ChassisSpeeds();

        switch (mMode) {
            case TELEOP:
                mDesiredSpeeds = mTeleopDriveController.update(latestFieldToVehicle.getRotation());

                if(isHeadingControlled){
                    isHeadingControlled = mDesiredSpeeds.omegaRadiansPerSecond < 0.25;

                    mDesiredSpeeds.omegaRadiansPerSecond = mHeadingController.update(latestFieldToVehicle.getRotation().getRadians());

                }

                uncheckedSpeeds = mDesiredSpeeds;
                
                var states = DriveConstants.kKinematics.toSwerveModuleStates(mDesiredSpeeds);

                SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxVel);

                mDesiredSpeeds = DriveConstants.kKinematics.toChassisSpeeds(states);
                
                break;
        
            case TRAJECTORY: //TODO: impl
                mDesiredSpeeds = mTrajectoryController.update(latestFieldToVehicle, Timer.getFPGATimestamp());
                
                uncheckedSpeeds = mDesiredSpeeds;

                break;
                
            case AUTO_ALIGN: //TODO: impl
                
                break;

            default:
                break;
        }

        ChassisSpeeds wantedSpeeds = ChassisSpeeds.discretize(mDesiredSpeeds, Constants.kLooperDt);
    
        mModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(wantedSpeeds);
        
        SmartDashboard.putNumber("Comp/Des X Speed: ", wantedSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Y Speed: ", wantedSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Rot Speed: ", wantedSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];

        for (int i = 0; i < mModules.length; i++){
            optimizedDesiredStates[i] = SwerveModuleState.optimize(mModuleStates[i], mModules[i].getAngle());

            mModules[i].runSetpoint(optimizedDesiredStates[i]);
        }

        Logger.recordOutput("Drive/SwerveStates/Setpoints", optimizedDesiredStates);
        Logger.recordOutput("Drive/SwerveStates/Unchecked Setpoints", DriveConstants.kKinematics.toSwerveModuleStates(uncheckedSpeeds));
        Logger.recordOutput("Drive/Desired Speeds", mDesiredSpeeds);
        Logger.recordOutput("Drive/Setpoint Speeds", wantedSpeeds);
        Logger.recordOutput("Drive/Mode", mMode);
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
