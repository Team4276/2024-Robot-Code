package frc.team4276.frc2024.subsystems.vision;

import java.util.List;

import frc.team4276.frc2024.Constants.VisionConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.frc2024.Constants;

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

        for(PhotonDevice camera : mAllCameras) {
            camera.outputTelemetry();

            if(!camera.isConnected()) return;
        }
    }
}
