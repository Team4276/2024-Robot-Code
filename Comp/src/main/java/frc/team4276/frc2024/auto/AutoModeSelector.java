package frc.team4276.frc2024.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING, 
        EXAMPLE,
        SPIN_TEST,
        STRESS_TEST,
        MID_3PIECE_STAGE,
        SUB_MIDDLE_2_PIECE,
        TAXI,
        SHOOT_TEST,
        SUB_AS_2PIECE,
        SUB_SS_2Piece,
        Close5Piece,
        SHOOT,
        BOX_BOX

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
        mModeChooser.addOption("Example", DesiredMode.EXAMPLE);
        mModeChooser.addOption("Spin test", DesiredMode.SPIN_TEST);
        mModeChooser.addOption("Stress test", DesiredMode.STRESS_TEST);
        mModeChooser.addOption("Mid 3 Piece Stage", DesiredMode.MID_3PIECE_STAGE);
        mModeChooser.addOption("Taxi", DesiredMode.TAXI);
        mModeChooser.addOption("Box Box", DesiredMode.BOX_BOX);
        SmartDashboard.putData("Comp/Auto Mode", mModeChooser);

    }

    public void updateModeCreator(boolean force_regen) {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode || force_regen) {
            //System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        case EXAMPLE:
            return Optional.of(new ActionExample());
        case SPIN_TEST:
            return Optional.of(new PPTest("Spin test"));
        case STRESS_TEST:
            return Optional.of(new PPTest("Stress Test"));
        case MID_3PIECE_STAGE:
            return Optional.of(new PPTest("Mid3PieceStage"));
        case TAXI:
            return Optional.of(new PPTest("Taxi"));
        case BOX_BOX:
            return Optional.of(new PPTest("BoxBox"));
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
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
