
package frc.team4276.frc2024.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1678.lib.swerve.SwerveDriveOdometry;
import frc.team4276.frc2024.Constants.DriveConstants;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Robot;

public class OdometryPhotonVision {

  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

  AprilTagFieldLayout aprilTagFieldLayout;

  SwerveDriveOdometry m_odometry_PV;

  private PhotonCamera m_PVcamera = new PhotonCamera("Arducam_12MP");
  private double camPitchRadians = (Constants.PhotonVisionConstants.kPitchDegrees / 360.0) * (2 * Math.PI);

  private Translation3d xlatCameraToRobot = new Translation3d(Constants.PhotonVisionConstants.kForwardMeters,
      Constants.PhotonVisionConstants.kSideMeters, Constants.PhotonVisionConstants.kUpMeters);
  private Rotation3d camRotation = new Rotation3d(0, camPitchRadians, 0); // cam facing forward, tilted up
  private Transform3d xformCamToRobot = new Transform3d(xlatCameraToRobot, camRotation);

  private int nLogCounter = 0;

  public OdometryPhotonVision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

      m_odometry_PV = new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          mDriveSubsystem.getModuleStates());
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public double diffDistanceMeters(Pose2d otherPose2d) {
    return m_odometry_PV.getPoseMeters().getTranslation().getDistance(m_odometry_PV.getPoseMeters().getTranslation());
  }

  public double diffHeadingDegrees(Pose2d otherPose2d) {
    return m_odometry_PV.getPoseMeters().getRotation().getDegrees() - mDriveSubsystem.getHeading().getDegrees();
  }

  public boolean hasTargets() {
    var result = m_PVcamera.getLatestResult();
    return result.hasTargets();
  }

  void updateOdometry_PV() {
    var result = m_PVcamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      PhotonTrackedTarget tgt = result.getBestTarget();
      Transform3d xformCamToTarget = tgt.getBestCameraToTarget();
      int tagID = tgt.getFiducialId();
      Optional<Pose3d> opt = aprilTagFieldLayout.getTagPose(tagID);
      Pose3d tagPose = opt.get();

      Pose3d positionFix = PhotonUtils.estimateFieldToRobotAprilTag(xformCamToTarget, tagPose, xformCamToRobot);
      m_odometry_PV.resetPosition(mDriveSubsystem.getModuleStates(), positionFix.toPose2d());
      
    } else {
      m_odometry_PV.update(
          mDriveSubsystem.getHeading(),
          mDriveSubsystem.getModuleStates());
    }
  }

  public void logOutput(Pose2d otherPose) {
    double distance = diffDistanceMeters(otherPose);
    double diffHeading = diffHeadingDegrees(otherPose);
    boolean hasTargets = hasTargets();

    SmartDashboard.putNumber("Odometry distance from PV", distance);
    SmartDashboard.putNumber("Odometry heading difference from PV", diffHeading);

    nLogCounter++;
    if (0 == nLogCounter % 1) {
      String msg = String.format("%d, %f, %f, %b\n", Robot.m_testMonitor.getTicks(), distance, diffHeading, hasTargets);
      Robot.m_testMonitor.logWrite(msg);
    }

  }
}