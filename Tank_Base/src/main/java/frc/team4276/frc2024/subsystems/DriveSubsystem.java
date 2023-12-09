// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.Constants.SnapConstants;
import frc.team4276.frc2024.Constants.DriveConstants.KinematicLimits;
import frc.team4276.frc2024.auto.AutoEvents;
import frc.team4276.lib.drivers.Pigeon;
import frc.team4276.lib.MAXSwerveModuleV2;

import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team1678.lib.swerve.ModuleState;
import frc.team1678.lib.swerve.SwerveDriveKinematics;
import frc.team1678.lib.swerve.SwerveDriveOdometry;
import frc.team254.lib.util.Util;

public class DriveSubsystem extends Subsystem {

  public enum DriveControlState {
    OPEN_LOOP,
    PATH_FOLLOWING
  }

  // The gyro sensor
  private Pigeon mPigeon;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry mOdometry;

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private DriveControlState mControlState = DriveControlState.OPEN_LOOP;

  private KinematicLimits mKinematicLimits = DriveConstants.kUncappedLimits;

  private static DriveSubsystem mInstance;

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }

    return mInstance;
  }

  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() {

    mPigeon = Pigeon.getInstance();
    mPigeon.setYaw(0.0);

  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(new Loop() {

      @Override
      public void onStart(double timestamp) {
      }

      @Override
      public void onLoop(double timestamp) {
        synchronized (this) {
          updateSetpoint();
          mOdometry.update(
              mPigeon.getYaw(),
              getModuleStates());
        }
      }

      @Override
      public void onStop(double timestamp) {
      }
    });

  }

  // TODO: limit motor vel in class instead of subsystem method (guarantees that its limited if u forget to limit it later)
  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void readPeriodicInputs() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.meas_module_states = getModuleStates();
    mPeriodicIO.meas_chassis_speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states);
    mPeriodicIO.heading = mPigeon.getYaw();
    mPeriodicIO.pitch = mPigeon.getPitch();

  }

  @Override
  public void outputTelemetry() {
    if (Constants.disableExtraTelemetry) {
      return;
    }

    SmartDashboard.putNumber("Robot X", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Robot Y", mOdometry.getPoseMeters().getY());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(
        getModuleStates(),
        pose);
  }

  public ModuleState[] getModuleStates() {
    ModuleState[] states = new ModuleState[4];

    return states;
  }

  private void updateSetpoint() {


    Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
        mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
        Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
    Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
    ChassisSpeeds wanted_speeds = new ChassisSpeeds(
        twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);

    if (mControlState == DriveControlState.PATH_FOLLOWING) {
      ModuleState[] real_module_setpoints = DriveConstants.kDriveKinematics.toModuleStates(wanted_speeds);
      mPeriodicIO.des_module_states = real_module_setpoints;
      return;
    }

    // Limit rotational velocity
    wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
        * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

    // Limit translational velocity
    double velocity_magnitude = Math.hypot(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond,
        mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
    if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
      wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude)
          * mKinematicLimits.kMaxDriveVelocity;
      wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude)
          * mKinematicLimits.kMaxDriveVelocity;
    }

    ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
    ChassisSpeeds prev_chassis_speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(prev_module_states);
    ModuleState[] target_module_states = DriveConstants.kDriveKinematics.toModuleStates(wanted_speeds);

    if (wanted_speeds.epsilonEquals(new ChassisSpeeds(), Util.kEpsilon)) {
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

    SmartDashboard.putNumber("Accel", min_translational_scalar);

    wanted_speeds = new ChassisSpeeds(
        prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
        prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
        prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);

    SmartDashboard.putNumber("Des X Speed: ", wanted_speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Des Y Speed: ", wanted_speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Des Rot Speed: ", wanted_speeds.omegaRadiansPerSecond);

    ModuleState[] real_module_setpoints = DriveConstants.kDriveKinematics.toModuleStates(wanted_speeds);
    mPeriodicIO.des_module_states = real_module_setpoints;

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(ModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, mKinematicLimits.kMaxDriveVelocity);

  }

  public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
    for (SwerveModuleState state : desiredStates) {
      ModuleState[] convStates = new ModuleState[] {
          new ModuleState(0,
              state.angle,
              state.speedMetersPerSecond)
      };

      setModuleStates(convStates);
    }
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mPigeon.setYaw(0);
  }

  public void zeroHeading(double reset) {
    mPigeon.setYaw(reset);
  }

  public Rotation2d getHeading() {
    return mPigeon.getYaw();
  }

  public Rotation2d getPitch() {
    return mPigeon.getPitch();
  }

  public static class PeriodicIO {
    // Inputs/Desired States
    double timestamp;
    ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    ModuleState[] meas_module_states = new ModuleState[] {
        new ModuleState(),
        new ModuleState(),
        new ModuleState(),
        new ModuleState()
    };
    Rotation2d heading = new Rotation2d();
    Rotation2d pitch = new Rotation2d();

    // Outputs
    ModuleState[] des_module_states = new ModuleState[] {
        new ModuleState(),
        new ModuleState(),
        new ModuleState(),
        new ModuleState()
    };
    Pose2d path_setpoint = new Pose2d();
    Rotation2d heading_setpoint = new Rotation2d();
  }

  public Command followPathCommand(PathPlannerTrajectory path) {
    return new FollowPathWithEvents(new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.resetOdometry(path.getInitialHolonomicPose());
        }),
        new PPSwerveControllerCommand(
            path,
            mOdometry::getPoseMeters,
            DriveConstants.kDriveKinematics.toSwerveDriveKinematics(),
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            this::setSwerveModuleStates,
            false,
            new EmptySubsystem()),
        new InstantCommand(() -> {
          stop();
        })), path.getMarkers(), AutoEvents.eventMap);
  }

  public void teleopDrive(double xSpeed, double ySpeed) {
    if (mControlState != DriveControlState.OPEN_LOOP) {
      mControlState = DriveControlState.OPEN_LOOP;
    }

    //mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(xSpeed, ySpeed, desiredRotDeg);
  }

  public void stop() {

  }

}
