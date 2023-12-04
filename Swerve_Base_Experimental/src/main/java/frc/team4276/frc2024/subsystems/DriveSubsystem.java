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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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


public class DriveSubsystem extends Subsystem {

  public enum DriveControlState {
      FORCE_ORIENT,
      OPEN_LOOP,
      HEADING_CONTROL,
      PATH_FOLLOWING,
  }

  // Create MAXSwerveModules
  private MAXSwerveModuleV2 m_frontLeft;
  private MAXSwerveModuleV2 m_frontRight;
  private MAXSwerveModuleV2 m_rearLeft;
  private MAXSwerveModuleV2 m_rearRight;

  // The gyro sensor
  private Pigeon mPigeon;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry mOdometry;

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;
  
  private KinematicLimits mKinematicLimits = DriveConstants.kUncappedLimits;

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
    m_frontLeft = new MAXSwerveModuleV2(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);

    m_frontRight = new MAXSwerveModuleV2(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);

    m_rearLeft = new MAXSwerveModuleV2(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);

    m_rearRight = new MAXSwerveModuleV2(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    mPigeon = Pigeon.getInstance();
    mPigeon.setYaw(0.0);

    mOdometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new ModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        });

    snapController = new PIDController(SnapConstants.kP, SnapConstants.kI, SnapConstants.kD);
    snapController.enableContinuousInput(0, 2 * Math.PI);

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
              new ModuleState[] {
                  m_frontLeft.getState(),
                  m_frontRight.getState(),
                  m_rearLeft.getState(),
                  m_rearRight.getState()
              });
        }
      }

      @Override
      public void onStop(double timestamp) {}
    });

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
        new ModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        },
        pose);
  }

  private void updateSetpoint() {        
    if (mControlState == DriveControlState.FORCE_ORIENT) return;

    Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
            mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
            Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
    Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
    ChassisSpeeds wanted_speeds = new ChassisSpeeds(
            twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);

    if (mControlState == DriveControlState.PATH_FOLLOWING) {
        SwerveModuleState[] real_module_setpoints = DriveConstants.kDriveKinematics.toSwerveModuleStates(wanted_speeds));
        mPeriodicIO.des_module_states = real_module_setpoints;
        return;
    }

    // Limit rotational velocity
    wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond) * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

    // Limit translational velocity
    double velocity_magnitude = Math.hypot(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
    if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
        wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
        wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
    }
    
    SwerveModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
    ChassisSpeeds prev_chassis_speeds = SwerveConstants.kKinematics.toChassisSpeeds(prev_module_states);
    SwerveModuleState[] target_module_states = SwerveConstants.kKinematics.toModuleStates(wanted_speeds);        

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
        prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar
    );

    ModuleState[] real_module_setpoints = SwerveConstants.kKinematics.toModuleStates(wanted_speeds);
    mPeriodicIO.des_module_states = real_module_setpoints;

}
  
  // /**
  //  * Method to drive the robot using joystick info.
  //  *
  //  * @param xSpeed        Speed of the robot in the x direction (forward).
  //  * @param ySpeed        Speed of the robot in the y direction (sideways).
  //  * @param rot           Angular rate of the robot.
  //  * @param fieldRelative Whether the provided x and y speeds are relative to the
  //  *                      field.
  //  */
  // public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
  //   // Convert the commanded speeds into the correct units for the drivetrain
  //   double xSpeedDelivered = xSpeed * maxAttainableSpeed;
  //   double ySpeedDelivered = ySpeed * maxAttainableSpeed;
  //   double rotDelivered = rot * DriveConstants.kMaxAngularVel;

  //   var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
  //       fieldRelative
  //           ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
  //               mPigeon.getYaw())
  //           : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
  //   SwerveDriveKinematics.desaturateWheelSpeeds(
  //       swerveModuleStates, maxSpeed);
  //   m_frontLeft.setDesiredState(swerveModuleStates[0]);
  //   m_frontRight.setDesiredState(swerveModuleStates[1]);
  //   m_rearLeft.setDesiredState(swerveModuleStates[2]);
  //   m_rearRight.setDesiredState(swerveModuleStates[3]);
  // }

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
    SwerveModuleState[] meas_module_states = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    Rotation2d heading = new Rotation2d();
    Rotation2d pitch = new Rotation2d();

    // Outputs
    SwerveModuleState[] des_module_states = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
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
            DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            this::setModuleStates,
            false,
            new EmptySubsystem()),
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
    //drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void stop() {
    //drive(0, 0, 0, false);
  }

  public void setX() {
    setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))}
    );
  }

}
