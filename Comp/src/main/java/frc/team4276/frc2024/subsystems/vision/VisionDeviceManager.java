package frc.team4276.frc2024.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants.VisionConstants;
import frc.team4276.lib.drivers.Subsystem;

import frc.team4276.frc2024.RobotState;

public class VisionDeviceManager extends Subsystem {
    private PhotonDevice mFrontCamera;
    private PhotonDevice mRearCamera;

    private List<PhotonDevice> mAllCameras;

    private boolean mIsDisabled = false;
    
    private static VisionDeviceManager mInstance;

    public static VisionDeviceManager getInstance(){
        if(mInstance == null) {
            mInstance = new VisionDeviceManager();
        }

        return mInstance;
    }

    private VisionDeviceManager(){
        mFrontCamera = new PhotonDevice(VisionConstants.kFrontCameraConstants);
        mRearCamera = new PhotonDevice(VisionConstants.kBackCameraConstants);

        mAllCameras = List.of(mFrontCamera, mRearCamera);
    }

    public synchronized void setDisabled(boolean disable) {
        mIsDisabled = disable;
    }

    public synchronized boolean getIsDisabled() {
        return mIsDisabled;
    }

    @Override
    public void readPeriodicInputs() {
        mAllCameras.forEach(PhotonDevice::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mAllCameras.forEach(PhotonDevice::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        for(PhotonDevice camera : mAllCameras) {
            // Not technically error but its the only word I can think of
            Pose3d error = camera.getLatestUpdate().relativeTo(new Pose3d(RobotState.getInstance().getWPILatestFieldToVehicle()));
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " X Translation Error", error.getX());
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Y Translation Error", error.getY());
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Height Error", error.getZ());
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Yaw Error", error.getRotation().getZ());
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Roll Error", error.getRotation().getX());
            SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Pitch Error", error.getRotation().getY());
        }
    }
}
