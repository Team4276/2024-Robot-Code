package frc.team4276.frc2024.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants.VisionConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.frc2024.Constants;
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
        if(mIsDisabled) return;
        mAllCameras.forEach(PhotonDevice::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        if(mIsDisabled) return;
        mAllCameras.forEach(PhotonDevice::writePeriodicOutputs);
    }

    @Override
    public void outputTelemetry() {
        if(mIsDisabled) return;

        if(Constants.disableExtraTelemetry) return;

        for(PhotonDevice camera : mAllCameras) { //TODO: get rid of this probably when calibration is finished
            if(!camera.isConnected()) return;

            if (camera.getLatestUpdate() != null) {
                Pose3d displacement = camera.getLatestUpdate().relativeTo(new Pose3d(RobotState.getInstance().getWPILatestFieldToVehicle()));
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " X Translation Displacement", displacement.getX());
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Y Translation Displacement", displacement.getY());
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Height Displacement", displacement.getZ());
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Yaw Displacement", displacement.getRotation().getZ());
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Roll Displacement", displacement.getRotation().getX());
                SmartDashboard.putNumber("Debug/Vision Tuning/" + camera.getName() + " Pitch Displacement", displacement.getRotation().getY());
            }
        }
    }
}
