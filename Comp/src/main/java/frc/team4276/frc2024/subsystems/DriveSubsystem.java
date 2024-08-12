// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.lib.drivers.Pigeon;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.swerve.HeadingController;
import frc.team4276.lib.swerve.MAXSwerveModuleV2;
import frc.team4276.lib.swerve.MotionPlanner;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.swerve.ModuleState;
import frc.team1678.lib.swerve.SwerveDriveOdometry;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.util.Util;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Twist2d;

public class DriveSubsystem extends Subsystem {
    public MAXSwerveModuleV2[] mModules;

    private SwerveDriveOdometry mOdometry;

    // The gyro sensor
    private Pigeon mPigeon;

    private PeriodicIO mPeriodicIO;

    public static class KinematicLimits {
        public double kMaxDriveVelocity = DriveConstants.kMaxVel; // m/s
        public double kMaxAccel = Double.MAX_VALUE; // m/s^2
        public double kMaxAngularVelocity = DriveConstants.kMaxAngularVel; // rad/s
        public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
        public String kName = "Default";
    }

    private KinematicLimits mKinematicLimits = DriveConstants.kUncappedLimits;
    
    public enum DriveControlState {
        FORCE_ORIENT,
        OPEN_LOOP,
        HEADING_CONTROL,
        PATH_FOLLOWING
    }
    
    private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;

    private MotionPlanner mMotionPlanner;

    private HeadingController mHeadingController;

    private boolean mOverrideHeading = false;

    private Rotation2d mTrackingAngle = Rotation2d.identity();

    private static DriveSubsystem mInstance;

    public static DriveSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new DriveSubsystem();
        }

        return mInstance;
    }

    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {
        mModules = new MAXSwerveModuleV2[] {
                new MAXSwerveModuleV2(
                        Ports.FRONT_LEFT_DRIVE,
                        Ports.FRONT_LEFT_TURN,
                        DriveConstants.kFrontLeftChassisAngularOffset),
                new MAXSwerveModuleV2(
                        Ports.FRONT_RIGHT_DRIVE,
                        Ports.FRONT_RIGHT_TURN,
                        DriveConstants.kFrontRightChassisAngularOffset),
                new MAXSwerveModuleV2(
                        Ports.BACK_LEFT_DRIVE,
                        Ports.BACK_LEFT_TURN,
                        DriveConstants.kBackLeftChassisAngularOffset),
                new MAXSwerveModuleV2(
                        Ports.BACK_RIGHT_DRIVE,
                        Ports.BACK_RIGHT_TURN,
                        DriveConstants.kBackRightChassisAngularOffset)
        };

        mPigeon = Pigeon.getInstance();
        mPigeon.setYaw(0.0);

        mPeriodicIO = new PeriodicIO();

        mOdometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                mPeriodicIO.meas_module_states);

        mMotionPlanner = new MotionPlanner();

        mHeadingController = HeadingController.getInstance();
    }

    public synchronized void teleopDrive(ChassisSpeeds speeds) {
        if (mControlState != DriveControlState.OPEN_LOOP && mControlState != DriveControlState.HEADING_CONTROL) {
            mControlState = DriveControlState.OPEN_LOOP;
        }

        if (mControlState == DriveControlState.HEADING_CONTROL) {
            if (Math.abs(speeds.omegaRadiansPerSecond) > 1.0) {
                mControlState = DriveControlState.OPEN_LOOP;
            } else {
                mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        mHeadingController.update(mPeriodicIO.heading.getRadians(), mPeriodicIO.timestamp));
                return;
            }
        }

        mPeriodicIO.des_chassis_speeds = speeds;
    }

    public synchronized void updatePathFollowingSetpoint(ChassisSpeeds speeds) {
        if (mControlState != DriveControlState.PATH_FOLLOWING) {
            mControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.des_chassis_speeds = speeds;
    }

    public synchronized void updatePPPathFollowingSetpoint(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        updatePathFollowingSetpoint(ChassisSpeeds.fromWPI(speeds));
    }

    public synchronized void setPathFollowingPath(PathPlannerPath path) {
        if (mControlState != DriveControlState.PATH_FOLLOWING) {
            mControlState = DriveControlState.PATH_FOLLOWING;
        }

        mMotionPlanner.setTrajectory(path, RobotState.getInstance().getLatestFieldToVehicle(), mPeriodicIO.meas_chassis_speeds, mPeriodicIO.timestamp);
    }

    public synchronized void feedTrackingSetpoint(Rotation2d angle) {
        mTrackingAngle = angle;
    }

    public synchronized void setHeadingSetpoint(Rotation2d angle) {
        if (mControlState != DriveControlState.HEADING_CONTROL) {
            mControlState = DriveControlState.HEADING_CONTROL;
        }

        if (mHeadingController.getTargetRad() != angle.getRadians()) {
            mHeadingController.setTarget(angle.getRadians());
        }
    }

    public synchronized void setKinematicLimits(KinematicLimits limits) {
        this.mKinematicLimits = limits;
    }

    public synchronized void overrideHeading(boolean overrideHeading) {
        mOverrideHeading = overrideHeading;
    }

    public void stop() {
        for (MAXSwerveModuleV2 module : mModules) {
            module.stop();
        }
    }

    // Stops drive without orienting modules
    public synchronized void stopModules() {
        List<Rotation2d> orientations = new ArrayList<>();
        for (int i = 0; i < mPeriodicIO.meas_module_states.length; i++) {
            orientations.add(Rotation2d.fromWPI(mPeriodicIO.meas_module_states[i].angle));
        }
        orientModules(orientations);
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        if (mControlState != DriveControlState.FORCE_ORIENT) {
            mControlState = DriveControlState.FORCE_ORIENT;
        }
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.des_module_states[i] = ModuleState.fromSpeeds(orientations.get(i).toWPI(), 0.0);
        }
    }

    public void setX() {
        orientModules(
                List.of(
                        Rotation2d.fromDegrees(45),
                        Rotation2d.fromDegrees(-45),
                        Rotation2d.fromDegrees(-45),
                        Rotation2d.fromDegrees(45)));
    }

    public boolean isPathFinished() {
        return mMotionPlanner.isFinished();
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    public synchronized Rotation2d getPitch() {
        return mPeriodicIO.pitch;
    }

    public synchronized ChassisSpeeds getMeasSpeeds() {
        return mPeriodicIO.meas_chassis_speeds;
    }

    public edu.wpi.first.math.kinematics.ChassisSpeeds getWPIMeasSpeeds() {
        return getMeasSpeeds().toWPI();
    }

    public synchronized ModuleState[] getModuleStates() {
        return mPeriodicIO.meas_module_states;
    }

    public synchronized DriveControlState getDriveControlState() {
        return mControlState;
    }

    public synchronized KinematicLimits getKinematicLimits() {
        return mKinematicLimits;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public synchronized void resetEncoders() {
        for (MAXSwerveModuleV2 mod : mModules) {
            mod.resetEncoders();
        }
    }

    public synchronized void resetGyro(double angleDeg) {
        Rotation2d prevOffset = mPigeon.getYawOffset();
        mPigeon.setYaw(angleDeg);
        mOdometry.offsetGyro(mPigeon.getYawOffset().rotateBy(prevOffset.inverse()).toWPI());
    }

    public synchronized void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(mPeriodicIO.meas_module_states, pose.toWPI());
    }

    private class PeriodicIO {
        // Inputs/Desired States
        double timestamp = Timer.getFPGATimestamp();
        ChassisSpeeds des_chassis_speeds = new ChassisSpeeds();
        ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds();
        ModuleState[] meas_module_states = new ModuleState[] {
                new ModuleState(),
                new ModuleState(),
                new ModuleState(),
                new ModuleState()
        };
        Rotation2d heading = Rotation2d.identity();
        Rotation2d pitch = Rotation2d.identity();
        
        Translation2d path_translation_error = Translation2d.identity();
        Rotation2d path_heading_error = Rotation2d.identity();

        // Outputs
        ModuleState[] des_module_states = new ModuleState[] {
                new ModuleState(),
                new ModuleState(),
                new ModuleState(),
                new ModuleState()
        };
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        for (int i = 0; i < mPeriodicIO.meas_module_states.length; i++) {
            mPeriodicIO.meas_module_states[i] = mModules[i].getState();
        }

        mPeriodicIO.meas_chassis_speeds = DriveConstants.kDriveKinematics
                .toChassisSpeeds(mPeriodicIO.meas_module_states);
        mPeriodicIO.heading = mPigeon.getYaw();
        mPeriodicIO.pitch = mPigeon.getPitch();

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                
            }

            @Override
            public void onLoop(double timestamp) {
                try {
                    synchronized (this) {
                        switch (mControlState) {
                            case OPEN_LOOP:
                                break;
                            case FORCE_ORIENT:
                                break;
                            case HEADING_CONTROL:
                                break;
                            case PATH_FOLLOWING:
                                updatePathFollowing();
                                break;

                            default:
                                stop();
                                break;
                        }

                        updateSetpoint();

                        mOdometry.update(
                                mPigeon.getYaw().toWPI(),
                                mPeriodicIO.meas_module_states);
                        RobotState.getInstance().addOdomObservations(
                                mPeriodicIO.timestamp, Pose2d.fromWPI(mOdometry.getPoseMeters()));
                    }
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });

    }

    private void updatePathFollowing() {
        mPeriodicIO.des_chassis_speeds = mMotionPlanner.update(RobotState.getInstance().getLatestFieldToVehicle(), mPeriodicIO.timestamp);


    }

    private void updateSetpoint() {
        if (mControlState == DriveControlState.FORCE_ORIENT) {
            return;
        }

        ChassisSpeeds des_chassis_speeds = mPeriodicIO.des_chassis_speeds;

        Pose2d robot_pose_vel = new Pose2d(des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
                des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / Constants.kLooperDt);

        ChassisSpeeds wanted_speeds;
        if (mOverrideHeading) {
            setHeadingSetpoint(mTrackingAngle);
            double new_omega = mHeadingController.update(mPeriodicIO.heading.getRadians(), mPeriodicIO.timestamp);
            wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);

        } else {
            wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);

        }

        if (mControlState != DriveControlState.PATH_FOLLOWING) {
            // Limit rotational velocity
            wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
                    * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

            // Limit translational velocity
            double velocity_magnitude = Math.hypot(des_chassis_speeds.vxMetersPerSecond,
                    des_chassis_speeds.vyMetersPerSecond);
            if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
                wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude)
                        * mKinematicLimits.kMaxDriveVelocity;
                wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude)
                        * mKinematicLimits.kMaxDriveVelocity;
            }

            ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get
                                                                                    // differentials
            ChassisSpeeds prev_chassis_speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(prev_module_states);
            ModuleState[] target_module_states = DriveConstants.kDriveKinematics.toModuleStates(wanted_speeds);

            if (wanted_speeds.epsilonEquals(ChassisSpeeds.identity(), Util.kEpsilon)) {
                for (int i = 0; i < target_module_states.length; i++) {
                    target_module_states[i].speedMetersPerSecond = 0.0;
                    target_module_states[i].angle = prev_module_states[i].angle;
                }
            }

            double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
            double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
            double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

            double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
            double min_translational_scalar = 1.0;

            if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
                // Check X
                double x_norm = Math.abs(dx / max_velocity_step);
                min_translational_scalar = Math.min(min_translational_scalar, x_norm);

                // Check Y
                double y_norm = Math.abs(dy / max_velocity_step);
                min_translational_scalar = Math.min(min_translational_scalar, y_norm);

                min_translational_scalar *= max_velocity_step;
            }

            double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
            double min_omega_scalar = 1.0;

            if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
                double omega_norm = Math.abs(domega / max_omega_step);
                min_omega_scalar = Math.min(min_omega_scalar, omega_norm);

                min_omega_scalar *= max_omega_step;
            }
            
            wanted_speeds = new ChassisSpeeds(
                prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
                prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
                prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);
        }

        

        SmartDashboard.putNumber("Comp/Des X Speed: ", wanted_speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Y Speed: ", wanted_speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Rot Speed: ", wanted_speeds.omegaRadiansPerSecond);

        ModuleState[] real_module_setpoints = DriveConstants.kDriveKinematics.toModuleStates(wanted_speeds);
        mPeriodicIO.des_module_states = real_module_setpoints;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; i++) {
            if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], true);
            } else if (mControlState == DriveControlState.PATH_FOLLOWING
                    || mControlState == DriveControlState.FORCE_ORIENT) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], false);
            }
        }

    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Comp/Heading", mPeriodicIO.heading.getDegrees());
        SmartDashboard.putString("Comp/Drive Mode", mControlState.name());

        SmartDashboard.putNumber("Debug/Motion Planner/Translation Error", mPeriodicIO.path_translation_error.norm());
        SmartDashboard.putNumber("Debug/Motion Planner/Heading Error", mPeriodicIO.path_heading_error.getDegrees());
    }
}
