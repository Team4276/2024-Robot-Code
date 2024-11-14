// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.lib.drivers.ADISGyro;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.swerve.HeadingController;
import frc.team4276.lib.swerve.MAXSwerveModule;
import frc.team4276.lib.swerve.MotionPlanner;
import frc.team4276.lib.swerve.SwerveDebug;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.swerve.ModuleState;
import frc.team1678.lib.swerve.SwerveDriveOdometry;
import frc.team1678.lib.swerve.ChassisSpeeds;

import frc.team254.lib.util.Util;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Twist2d;

public class DriveSubsystem extends Subsystem {
    public MAXSwerveModule[] mModules;

    private SwerveDriveOdometry mOdometry;

    private ADISGyro mGyro;

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
        PATH_FOLLOWING,
        PATH_FOLLOWING_CHOR
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

    private DriveSubsystem() {
        mModules = new MAXSwerveModule[] {
                new MAXSwerveModule(DriveConstants.kFLConstants),
                new MAXSwerveModule(DriveConstants.kFRConstants),
                new MAXSwerveModule(DriveConstants.kBLConstants),
                new MAXSwerveModule(DriveConstants.kBRConstants)
        };

        mGyro = ADISGyro.getInstance();

        mPeriodicIO = new PeriodicIO();

        mOdometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                mPeriodicIO.meas_module_states);

        mMotionPlanner = new MotionPlanner();
        mHeadingController = HeadingController.getInstance();
        
        Shuffleboard.getTab("Path").addNumber("X Translation", RobotState.getInstance().getLatestFieldToVehicle().getTranslation()::x);
        Shuffleboard.getTab("Path").addNumber("Y Translation", RobotState.getInstance().getLatestFieldToVehicle().getTranslation()::y);
        Shuffleboard.getTab("Path").addNumber("Rotation", RobotState.getInstance().getLatestFieldToVehicle().getRotation()::getDegrees);
        
        Shuffleboard.getTab("Path").addNumber("X Error", mMotionPlanner.getTranslationError()::x);
        Shuffleboard.getTab("Path").addNumber("Y Error", mMotionPlanner.getTranslationError()::y);
        Shuffleboard.getTab("Path").addNumber("Translation Error", mMotionPlanner.getTranslationError()::norm);
        Shuffleboard.getTab("Path").addNumber("Rotation Error", mMotionPlanner.getRotationError()::getDegrees);
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

    public synchronized void setPhoreoTraj(Trajectory<?> traj) {
        if(mControlState != DriveControlState.PATH_FOLLOWING_CHOR) {
            mControlState = DriveControlState.PATH_FOLLOWING_CHOR;
        }

        mMotionPlanner.setTrajectory(traj, RobotState.getInstance().getLatestFieldToVehicle(), mPeriodicIO.meas_chassis_speeds, Timer.getFPGATimestamp());
    }

    public synchronized void setChoreoTraj(Trajectory<SwerveSample> traj) {
        if(mControlState != DriveControlState.PATH_FOLLOWING_CHOR) {
            mControlState = DriveControlState.PATH_FOLLOWING_CHOR;
        }

        // mMotionPlanner.setTrajectory(traj, RobotState.getInstance().getLatestFieldToVehicle(), mPeriodicIO.meas_chassis_speeds, Timer.getFPGATimestamp());
    }



    public synchronized void updatePathFollowingSetpoint(ChassisSpeeds speeds) {
        if (mControlState != DriveControlState.PATH_FOLLOWING) {
            mControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.des_chassis_speeds = speeds;

        // mPeriodicIO.des_chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //     speeds.vxMetersPerSecond, 
        //     speeds.vyMetersPerSecond, 
        //     speeds.omegaRadiansPerSecond, 
        //     Rotation2d.fromDegrees(0.0).toWPI(), 
        //     AllianceChooser.getInstance().isAllianceRed());
    }

    public synchronized void updatePPPathFollowingSetpoint(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        updatePathFollowingSetpoint(ChassisSpeeds.fromWPI(speeds));
    }

    public synchronized void feedTrackingSetpoint(Rotation2d angle) {
        mTrackingAngle = angle;
    }

    public synchronized void setHeadingSetpoint(Rotation2d angle) {
        if (mControlState != DriveControlState.HEADING_CONTROL) {
            mControlState = DriveControlState.HEADING_CONTROL;
        }

        if (mHeadingController.getTargetRad() != angle.getRadians()) {
            mHeadingController.setTarget(angle.getRadians()
            // , mPeriodicIO.heading.getRadians(), mPeriodicIO.meas_chassis_speeds.omegaRadiansPerSecond
            );
        }
    }

    public synchronized void setKinematicLimits(KinematicLimits limits) {
        this.mKinematicLimits = limits;
    }

    public synchronized void overrideHeading(boolean overrideHeading) {
        mOverrideHeading = overrideHeading;
    }

    public void stop() {
        for (MAXSwerveModule module : mModules) {
            module.stop();
        }
    }

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

    public synchronized boolean isTrajFinished() {
        return mMotionPlanner.isFinished();
    }

    public synchronized boolean isVirtual(){
        return true;
    }

    public synchronized void resetDriveEncoders() {
        for (MAXSwerveModule mod : mModules) {
            mod.resetEncoders();
        }
    }

    public synchronized void resetGyro(double angleDeg) {
        Rotation2d prevOffset = mGyro.getYawOffset();
        mGyro.setYaw(angleDeg);
        mOdometry.offsetGyro(mGyro.getYawOffset().rotateBy(prevOffset.inverse()).toWPI());
    }

    public synchronized void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(mPeriodicIO.meas_module_states, pose.toWPI());
    }

    public synchronized void resetOdometryWPI(edu.wpi.first.math.geometry.Pose2d pose){
        resetOdometry(Pose2d.fromWPI(pose));
    }

    private class PeriodicIO {
        // Inputs/Desired States
        double timestamp = Timer.getFPGATimestamp();
        ChassisSpeeds des_chassis_speeds = ChassisSpeeds.identity();
        ChassisSpeeds meas_chassis_speeds = ChassisSpeeds.identity();
        ModuleState[] meas_module_states = new ModuleState[] {
                ModuleState.identity(),
                ModuleState.identity(),
                ModuleState.identity(),
                ModuleState.identity()
        };
        Rotation2d heading = Rotation2d.identity();
        Rotation2d pitch = Rotation2d.identity();

        // Outputs
        ModuleState[] des_module_states = new ModuleState[] {
                ModuleState.identity(),
                ModuleState.identity(),
                ModuleState.identity(),
                ModuleState.identity()
        };
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        for (int i = 0; i < mPeriodicIO.meas_module_states.length; i++) {
            mModules[i].readPeriodicInputs();
            mPeriodicIO.meas_module_states[i] = mModules[i].getState();
        }

        mPeriodicIO.meas_chassis_speeds = DriveConstants.kDriveKinematics
                .toChassisSpeeds(mPeriodicIO.meas_module_states);
        mPeriodicIO.heading = mGyro.getYaw();
        mPeriodicIO.pitch = mGyro.getPitch();

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // mHeadingController.reset(mPeriodicIO.heading.getRadians(), 0.0);
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
                                break;
                            case PATH_FOLLOWING_CHOR:
                                mPeriodicIO.des_chassis_speeds = mMotionPlanner.update(RobotState.getInstance().getLatestFieldToVehicle(), timestamp);

                                break;
                            default:
                                stop();
                                break;
                        }

                        updateSetpoint();

                        mOdometry.update(
                                mGyro.getYaw().toWPI(),
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
            if(isVirtual()){
                mModules[i].setDesiredState(ModuleState.identity(), false);

            } else if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], true);
            } else if (mControlState == DriveControlState.PATH_FOLLOWING
                    || mControlState == DriveControlState.FORCE_ORIENT
                    || mControlState == DriveControlState.PATH_FOLLOWING_CHOR) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], false);
            }

            mModules[i].writePeriodicOutputs();
        }

        
        SwerveDebug.outputSmartDashboard(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, 
            mPeriodicIO.des_chassis_speeds.vyMetersPerSecond, 
            mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Comp/Heading", mPeriodicIO.heading.getDegrees());
        SmartDashboard.putString("Comp/Drive Mode", mControlState.name());

        SmartDashboard.putNumber("Comp/Pos X", RobotState.getInstance().getLatestFieldToVehicle().getTranslation().x());
        SmartDashboard.putNumber("Comp/Pos Y", RobotState.getInstance().getLatestFieldToVehicle().getTranslation().y());
        
        for(int i = 0; i < mModules.length; i++) {
            mModules[i].outputTelemetry();
        }

        if(Constants.disableExtraTelemetry) return;
    }
}
