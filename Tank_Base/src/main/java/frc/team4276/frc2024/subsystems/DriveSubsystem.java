// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.auto.AutoEvents;
import frc.team4276.lib.MAXSkid;
import frc.team4276.lib.drivers.Pigeon;

import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.loops.ILooper;

public class DriveSubsystem extends Subsystem {
  public enum DriveControlState {
    OPEN_LOOP,
    PATH_FOLLOWING
  }

  private MAXSkid lSkid;
  private MAXSkid rSkid;

  // The gyro sensor
  private Pigeon mPigeon;

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry mOdometry;

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private DriveControlState mControlState = DriveControlState.OPEN_LOOP;

  private static DriveSubsystem mInstance;

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }

    return mInstance;
  }

  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() {
    lSkid = new MAXSkid(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kRearLeftDrivingCanId, true);
    rSkid = new MAXSkid(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kRearRightDrivingCanId, false);

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
          mOdometry.update(
              mPigeon.getYaw(),
              lSkid.getPosition(),
              rSkid.getPosition());
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
        mPigeon.getYaw(),
        lSkid.getPosition(),
        rSkid.getPosition(),
        pose);
  }

  public void setSkidSpeeds(double lSpeed, double rSpeed){
    mPeriodicIO.des_right_speed = rSpeed;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    lSkid.resetEncoder();
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
    double des_right_speed;
    double des_left_speed;
    Rotation2d heading = new Rotation2d();
    Rotation2d pitch = new Rotation2d();

    // Outputs
    Pose2d path_setpoint = new Pose2d();
  }

  public Command followPathCommand(PathPlannerTrajectory path) {
    return new FollowPathWithEvents(new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.resetOdometry(path.getInitialPose());
        }),
        new PPRamseteCommand(
          path, 
          mOdometry::getPoseMeters,
          new RamseteController(),
          new DifferentialDriveKinematics(DriveConstants.kTrackWidth),
          this::setSkidSpeeds, 
          new EmptySubsystem()),
        new InstantCommand(() -> {
          stop();
        })), path.getMarkers(), AutoEvents.eventMap);
  }

  public void teleopDrive(double lSpeed, double rSpeed) {
    if (mControlState != DriveControlState.OPEN_LOOP) {
      mControlState = DriveControlState.OPEN_LOOP;
    }

    mPeriodicIO.des_left_speed = lSpeed;
    mPeriodicIO.des_right_speed = rSpeed;
  }

  public void stop() {

  }

}
