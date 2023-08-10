/*package frc.utils;

import java.io.IOException;
import java.util.Optional;

// if "import org.photonvision cannot be resolved"
// Install PhotonLib using 'Manage Vendor Libraries' ONLINE install, (offline not available as of April 2023)
//     https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json 
// Must build once while connected to outside Internet, then can disconnect
// More detail here:  https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html 

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldPose {

    public static boolean isValidPosition;
    public static Pose2d position;

    private static PhotonCamera cam;
    private static PhotonPoseEstimator photonPoseEstimator;

    private static boolean isInitialized = false;

    public FieldPose() {
        init();
    }

    private static boolean init() {
        cam = new PhotonCamera("HD_Pro_Webcam_C920");
        position = new Pose2d();     
        
        try {

            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    cam, robotToCam);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public static double getRotationDegrees() {
        return position.getRotation().getDegrees();
    }

    public static void updatePosition() {

        if(isInitialized) {
            photonPoseEstimator.setReferencePose(position);

            Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
            isValidPosition = pose.isPresent();
            if (isValidPosition) {
                position = pose.get().estimatedPose.toPose2d();
            }
        }
    }

}
*/