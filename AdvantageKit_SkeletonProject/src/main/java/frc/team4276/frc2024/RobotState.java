package frc.team4276.frc2024;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N2;
import frc.team254.lib.util.MovingAverage;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.shooting.RegressionMaps;

public class RobotState {
  private Translation2d mEstimatedPose = new Translation2d();
  private MovingAverage mEstimatedVisionHeading = new MovingAverage(100);

  private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
  private boolean mHasUpdated = false;

  private static final double kObservationBufferTime = 1.0;

  private final TimeInterpolatableBuffer<edu.wpi.first.math.geometry.Pose2d> mOdomPoseBuffer;

  private Field.POIs mPOIs = Field.Red.kPOIs;

  // not really sure what 6328 is doing with theirs so I made this
  public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {

    public FlywheelSpeeds(double leftSpeed, double rightSpeed) {
      this.leftSpeed = leftSpeed;
      this.rightSpeed = rightSpeed;
    }

    public static FlywheelSpeeds fromSpeaker(double robot_to_target) {
      double leftSpeed = RegressionMaps.kSpeakerFlywheelRPMs.get(robot_to_target);
      double rightSpeed = RegressionMaps.kSpeakerFlywheelRPMs.get(robot_to_target);
      return new FlywheelSpeeds(leftSpeed, rightSpeed);
    }

    public static FlywheelSpeeds fromFerry(double robot_to_target) {
      double leftSpeed = RegressionMaps.kFerryFlywheelRPMs.get(robot_to_target);
      double rightSpeed = RegressionMaps.kFerryFlywheelRPMs.get(robot_to_target);
      return new FlywheelSpeeds(leftSpeed, rightSpeed);
    }
  }

  public record AimingParameters(
      Rotation2d driveHeading,
      Rotation2d armAngle,
      double effectiveDistance,
      FlywheelSpeeds flywheelSpeeds) {}

  private static RobotState mInstance;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
    mOdomPoseBuffer = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);

    reset(0.0, new Pose2d());
  }

  public synchronized void reset(double start_time, Pose2d initial_pose) {
    mOdomPoseBuffer.addSample(start_time, initial_pose);
    mEstimatedPose = initial_pose.getTranslation();
  }

  public synchronized void resetKalmanFilters() {
    mKalmanFilter =
        new ExtendedKalmanFilter<N2, N2, N2>(
            Nat.N2(), // Dimensions of output (x, y)
            Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
            Nat.N2(), // Dimensions of vision (x, y)
            (x, u) -> u, // The derivative of the output is predicted shift (always 0)
            (x, u) -> x, // The output is position (x, y)
            Constants.RobotStateConstants
                .kStateStdDevs, // Standard deviation of position (uncertainty propagation
            // with no vision)
            Constants.RobotStateConstants
                .kLocalMeasurementStdDevs, // Standard deviation of vision measurements
            Constants.kLooperDt);
  }

  public synchronized Field.POIs getPOIs() {
    return mPOIs;
  }

  public synchronized void setBlue() {
    mPOIs = Field.Blue.kPOIs;
  }

  public synchronized void setRed() {
    mPOIs = Field.Red.kPOIs;
  }

  public synchronized void visionHeadingUpdate(double heading_rad) {
    mEstimatedVisionHeading.addNumber(heading_rad);
  }

  public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot) {
    mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);

    mOdomPoseBuffer.addSample(timestamp, odom_to_robot);
  }

  public static class VisionUpdate {
    public final double timestamp;
    public final Translation2d fieldToVis;
    public final double distStDev;

    public VisionUpdate(double timestamp, Translation2d fieldToVis, double distStDev) {
      this.timestamp = timestamp;
      this.fieldToVis = fieldToVis;
      this.distStDev = distStDev;
    }
  }

  public synchronized void visionUpdate(VisionUpdate update) {
    double visionTimestamp = update.timestamp;

    if (mOdomPoseBuffer.getInternalBuffer().lastKey() - kObservationBufferTime > visionTimestamp)
      return;

    Pose2d sample = mOdomPoseBuffer.getSample(visionTimestamp).get();

    mEstimatedPose =
        update.fieldToVis.plus(
            mOdomPoseBuffer
                .getInternalBuffer()
                .lastEntry()
                .getValue()
                .getTranslation()
                .minus(sample.getTranslation()));

    if (!mHasUpdated) {
      mKalmanFilter.setXhat(0, mEstimatedPose.getX());
      mKalmanFilter.setXhat(1, mEstimatedPose.getY());

      mHasUpdated = true;
      return;
    }

    try {
      mKalmanFilter.correct(
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(mEstimatedPose.getX(), mEstimatedPose.getX()),
          StateSpaceUtil.makeCovarianceMatrix(
              Nat.N2(), VecBuilder.fill(update.distStDev, update.distStDev)));
      mEstimatedPose.plus(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)));

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public AimingParameters getFerryAimingParameters() {
    return new AimingParameters(null, null, 0, FlywheelSpeeds.fromFerry(getPOIs().kBank.getNorm()));
  }

  public AimingParameters getSpeakerAimingParameters() {
    return new AimingParameters(
        null, null, 0, FlywheelSpeeds.fromSpeaker(getPOIs().kSpeakerCenter.getNorm()));
  }

  // Use on enabled init
  public synchronized double getHeadingFromVision() {
    if (mEstimatedVisionHeading.getSize() == 0) {
      return getLatestFieldToVehicle().getRotation().getRadians();
    }
    return mEstimatedVisionHeading.getAverage();
  }

  public synchronized Pose2d getLatestFieldToVehicle() {
    return new Pose2d(
        mEstimatedPose, mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getRotation());
  }
}
