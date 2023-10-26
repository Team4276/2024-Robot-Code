// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2024.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.Constants.SnapConstants;
import frc.team4276.lib.MAXSwerveModule;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  AprilTagFieldLayout aprilTagFieldLayout;

  private PhotonCamera m_PVcamera = new PhotonCamera("Arducam_12MP");
  private double camForwardMeters = 0.5;
  private double camSideMeters = 0.2;
  private double camUpMeters = 0.2;
  private double camPitchDegrees = 45.0;
  private double camPitchRadians = (camPitchDegrees / 360.0) * (2 * Math.PI);

  private Translation3d xlatCameraToRobot = new Translation3d(camForwardMeters, camSideMeters, camUpMeters);
  private Rotation3d camRotation = new Rotation3d(0, camPitchRadians, 0); // cam facing forward, tilted up
  private Transform3d xformCamToRobot = new Transform3d(xlatCameraToRobot, camRotation);

  private long nLogCounter = 0;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  SwerveDriveOdometry m_odometry_PV = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecondB;
  private double maxAttainableSpeed = DriveConstants.kMaxAttainableSpeedB;

  private PIDController snapController = new PIDController(SnapConstants.kP, SnapConstants.kI, SnapConstants.kD);

  private static DriveSubsystem mInstance;

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }

    return mInstance;
  }

  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() {
    snapController.enableContinuousInput(0, 2 * Math.PI);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());

    if (Robot.m_testMonitor.isTestMonitorEnabled()) {

      var result = m_PVcamera.getLatestResult();
      boolean hasTargets = result.hasTargets();
      if (hasTargets) {

        PhotonTrackedTarget tgt = result.getBestTarget();
        Transform3d xformCamToTarget = tgt.getBestCameraToTarget();
        int tagID = tgt.getFiducialId();
        Optional<Pose3d> opt = aprilTagFieldLayout.getTagPose(tagID);
        Pose3d tagPose = opt.get();

        Pose3d positionFix = PhotonUtils.estimateFieldToRobotAprilTag(xformCamToTarget, tagPose, xformCamToRobot);
        resetOdometry(positionFix.toPose2d());
      }

      m_odometry_PV.update(
          Rotation2d.fromDegrees(m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          });

      double distance = m_odometry_PV.getPoseMeters().getTranslation()
          .getDistance(m_odometry_PV.getPoseMeters().getTranslation());
      double diffHeading = m_odometry_PV.getPoseMeters().getRotation().getDegrees()
          - m_odometry.getPoseMeters().getRotation().getDegrees();

      SmartDashboard.putNumber("Odometry distance from PV", distance);
      SmartDashboard.putNumber("Odometry heading difference from PV", diffHeading);

      nLogCounter++;
      if (0 == nLogCounter % 200) {
        String msg = String.format("%ld, %f, %f\n", Robot.m_testMonitor.getTicks(), distance, diffHeading);
        Robot.m_testMonitor.logWrite(msg);
      }
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
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
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxAttainableSpeed;
    double ySpeedDelivered = ySpeed * maxAttainableSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
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
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Command followPathCommand(PathPlannerTrajectory path) {

    if (Robot.m_testMonitor.isTestMonitorEnabled()) {
      String msg = new String("Reset Odometry from holonomic pose\n");
      Robot.m_testMonitor.logWrite(msg);
    }

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.resetOdometry(path.getInitialHolonomicPose());
        }),
        new PPSwerveControllerCommand(
            path,
            this::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            this::setModuleStates,
            false,
            this),
        new InstantCommand(() -> {
          this.drive(0, 0, 0, false, false);
        }));
  }

  public double getPitch() {
    return m_gyro.getYComplementaryAngle();
  }

  public void shiftSpeedUp() {
    if (maxSpeed != DriveConstants.kMaxSpeedMetersPerSecond) {
      maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
    }

    if (maxAttainableSpeed != DriveConstants.kMaxAttainableSpeed) {
      maxAttainableSpeed = DriveConstants.kMaxAttainableSpeed;
    }
  }

  public void shiftSpeedDown() {
    if (maxSpeed != DriveConstants.kMaxSpeedMetersPerSecondB) {
      maxSpeed = DriveConstants.kMaxSpeedMetersPerSecondB;
    }

    if (maxAttainableSpeed != DriveConstants.kMaxAttainableSpeedB) {
      maxAttainableSpeed = DriveConstants.kMaxAttainableSpeedB;
    }
  }

  /**
   * @param desiredRotDeg Must be a value between 0 and 360
   */
  public void snapDrive(double xSpeed, double ySpeed, double desiredRotDeg, boolean fieldRelative, boolean rateLimit){
    double rot = snapController.calculate(Math.toRadians(m_gyro.getAngle()), Math.toRadians(desiredRotDeg));

    SmartDashboard.putNumber("Snap output", rot);
    //drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
  }



}
