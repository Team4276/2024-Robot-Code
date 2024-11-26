package frc.team4276.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import frc.team4276.frc2024.subsystems.vision.VisionIOPhoton.PhotonDeviceConstants;

public class VisionConstants {
    public static final PhotonDeviceConstants kFrontCameraConstants = new PhotonDeviceConstants();
    static {
        kFrontCameraConstants.kCameraName = "Front Camera";
        kFrontCameraConstants.kCameraNameId = "Arducam_OV9281_USB_Camera";
        kFrontCameraConstants.kRobotToCamera = new Transform3d(
                Units.inchesToMeters(9.591351), Units.inchesToMeters(7.500000) * -1.0, Units.inchesToMeters(5.575688),
                new Rotation3d(0.0, Math.toRadians(20.0) * -1.0, 0.0));
    }

    public static final PhotonDeviceConstants kBackCameraConstants = new PhotonDeviceConstants();
    static {
        kFrontCameraConstants.kCameraName = "Back Camera";
        kBackCameraConstants.kCameraNameId = "Arducam_12MP";
        kBackCameraConstants.kRobotToCamera = new Transform3d(
                Units.inchesToMeters(7.837035), Units.inchesToMeters(8.750000), Units.inchesToMeters(6.072381),
                new Rotation3d(0.0, Math.toRadians(20.0) * -1.0, 0.0));
    }
}