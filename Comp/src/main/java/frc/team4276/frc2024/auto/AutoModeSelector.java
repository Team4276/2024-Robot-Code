package frc.team4276.frc2024.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING, 
        TEST_2,
        CLOSE_4NOTE_SAFE,
        CLOSE_4NOTE_FAST,
        CLOSE_5NOTE_MIDSTEAL,
        CENTER_PRELOAD_TAXI,
        SS_PRELOAD_TAXI

    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

    private static AutoModeSelector mInstance;

    public static AutoModeSelector getInstance() {
        if(mInstance == null) {
            mInstance = new AutoModeSelector();
        }

        return mInstance;
    }

    private AutoModeSelector() {
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test 2", DesiredMode.TEST_2);
        mModeChooser.addOption("CloseFourNoteSafe", DesiredMode.CLOSE_4NOTE_SAFE);
        mModeChooser.addOption("CloseFourNoteFast", DesiredMode.CLOSE_4NOTE_FAST);
        mModeChooser.addOption("CloseFourNoteMidSteal", DesiredMode.CLOSE_5NOTE_MIDSTEAL);
        mModeChooser.addOption("CenterPreloadTaxi", DesiredMode.CENTER_PRELOAD_TAXI);
        mModeChooser.addOption("SSPreloadTaxi", DesiredMode.SS_PRELOAD_TAXI);
        SmartDashboard.putData("Comp/Auto Mode", mModeChooser);

    }

    public void updateModeCreator(boolean force_regen) {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode || force_regen) {
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        case TEST_2:
            return Optional.of(new BoxTest());
        case CLOSE_4NOTE_SAFE:
            return Optional.of(new Close_4Note_Safe());
        case CLOSE_4NOTE_FAST:
            return Optional.of(new Close_4Note_Fast());
        case CLOSE_5NOTE_MIDSTEAL:
            return Optional.of(new Close_5Note_MidSteal());
        case CENTER_PRELOAD_TAXI:
            return Optional.of(new Center_Preload_Taxi());
        case SS_PRELOAD_TAXI:
            return Optional.of(new SS_Preload_Taxi());
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.of(new DoNothingMode());
    }

    public static SendableChooser<DesiredMode> getModeChooser() {
        return mModeChooser;
    }

    public DesiredMode getDesiredAutomode() {
        return mCachedDesiredMode;
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Comp/AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
