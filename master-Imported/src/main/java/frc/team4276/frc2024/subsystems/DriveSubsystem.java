// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.Constants.SnapConstants;
import frc.team4276.frc2024.auto.AutoEvents;
import frc.team4276.lib.MAXSwerveModule;
import frc.team4276.lib.drivers.Pigeon;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private MAXSwerveModule m_frontLeft;
  private MAXSwerveModule m_frontRight;
  private MAXSwerveModule m_rearLeft;
  private MAXSwerveModule m_rearRight;

  // The gyro sensor
  private Pigeon mPigeon;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry mOdometry;

  private double maxSpeed = DriveConstants.kMaxVel;
  private double maxAttainableSpeed = DriveConstants.kMaxAttainableVel;

  private PIDController snapController;

  private static DriveSubsystem mInstance;

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }

    return mInstance;
  }

  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() {
    m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

    m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);
    
    m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

    m_rearRight  = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    mPigeon = Pigeon.getInstance();
    mPigeon.setYaw(0.0);

    mOdometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      mPigeon.getYaw(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
    
    snapController = new PIDController(SnapConstants.kP, SnapConstants.kI, SnapConstants.kD);
    snapController.enableContinuousInput(0, 2 * Math.PI);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    mOdometry.update(
        mPigeon.getYaw(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

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
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxAttainableSpeed;
    double ySpeedDelivered = ySpeed * maxAttainableSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularVel;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                mPigeon.getYaw())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mPigeon.setYaw(0);
  }

  public void zeroHeading(double reset){
    mPigeon.setYaw(reset);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return mPigeon.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return mPigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Rotation2d getPitch() {
    return mPigeon.getPitch();
  }

  public Command followPathCommand(PathPlannerTrajectory path) {
    return new FollowPathWithEvents(new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.resetOdometry(path.getInitialHolonomicPose());
        }),
        new PPSwerveControllerCommand(
            path,
            mOdometry::getPoseMeters,
            DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            this::setModuleStates,
            false,
            this),
        new InstantCommand(() -> {
          stop();
        })), path.getMarkers(), AutoEvents.eventMap);
  }

  /**
   * @param desiredRotDeg Must be a value between 0 and 360
   */
  public void snapDrive(double xSpeed, double ySpeed, double desiredRotDeg, boolean fieldRelative) {
    double rot = snapController.calculate(Math.toRadians(mPigeon.getYaw().getDegrees()), Math.toRadians(desiredRotDeg));

    SmartDashboard.putNumber("Snap output", rot);
    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void stop() {
    drive(0, 0, 0, false);
  }

    /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

}
