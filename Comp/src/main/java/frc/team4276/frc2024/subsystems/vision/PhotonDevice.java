package frc.team4276.frc2024.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team254.lib.geometry.Translation2d;
import frc.team4276.frc2024.RobotState;
import frc.team4276.lib.drivers.Subsystem;

public class PhotonDevice extends Subsystem {
    private PhotonCamera mCamera;

    private PhotonPoseEstimator mPoseEstimator;

    public PhotonDevice(String cameraName, Transform3d robotToCamera) {
        mCamera = new PhotonCamera(cameraName);

        mPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mCamera, robotToCamera);

        mPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    //TODO: check vision readings vs gyro; check height reading (citrus ignore)
    @Override
    public void readPeriodicInputs() {
        PhotonPipelineResult result = mCamera.getLatestResult();

        if(!result.hasTargets()) 
            return;

        Optional<EstimatedRobotPose> estimatedRobotPose = mPoseEstimator.update(result);

        if(estimatedRobotPose.isPresent()) {
            RobotState.getInstance().visionUpdate(new RobotState.VisionUpdate(estimatedRobotPose.get().timestampSeconds, 
                new Translation2d(estimatedRobotPose.get().estimatedPose.getX(), estimatedRobotPose.get().estimatedPose.getY())));

            SmartDashboard.putNumber("PhotonCamera X", estimatedRobotPose.get().estimatedPose.getX());
            SmartDashboard.putNumber("PhotonCamera Y", estimatedRobotPose.get().estimatedPose.getY());
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
    }
}
