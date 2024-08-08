package frc.team4276.frc2024.subsystems.vision;

import frc.team4276.frc2024.Constants.VisionConstants;
import frc.team4276.lib.drivers.Subsystem;

public class VisionDeviceManager extends Subsystem {
    private PhotonDevice mFrontCamera;
    
    private static VisionDeviceManager mInstance;

    public static VisionDeviceManager getInstance(){
        if(mInstance == null) {
            mInstance = new VisionDeviceManager();
        }

        return mInstance;
    }

    private VisionDeviceManager(){
        mFrontCamera = new PhotonDevice(VisionConstants.kFrontCameraName, VisionConstants.kFrontCameraOffset);
    }

    @Override
    public void readPeriodicInputs() {
        mFrontCamera.readPeriodicInputs();
    }

    @Override
    public void writePeriodicOutputs() {
        mFrontCamera.writePeriodicOutputs();
    }
}
