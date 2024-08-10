package frc.team4276.frc2024.subsystems.vision;

import java.util.List;

import frc.team4276.frc2024.Constants.VisionConstants;
import frc.team4276.lib.drivers.Subsystem;

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

    @Override
    public void readPeriodicInputs() {
        mAllCameras.forEach(PhotonDevice::readPeriodicInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        mAllCameras.forEach(PhotonDevice::writePeriodicOutputs);
    }

    public synchronized void setDisabled(boolean disable) {
        mIsDisabled = disable;
    }

    public synchronized boolean getIsDisabled() {
        return mIsDisabled;
    }
}
