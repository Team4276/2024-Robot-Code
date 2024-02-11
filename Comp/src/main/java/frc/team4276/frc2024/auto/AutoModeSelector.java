package frc.team4276.frc2024.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import frc.team4276.frc2024.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING, 
        EXAMPLE,
        SPIN_TEST,
        STRESS_TEST,
        MID_3PIECE_STAGE

    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

    public AutoModeSelector() {
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Example", DesiredMode.EXAMPLE);
        mModeChooser.addOption("Spin test", DesiredMode.SPIN_TEST);
        mModeChooser.addOption("Stress test", DesiredMode.STRESS_TEST);
        mModeChooser.addOption("Mid 3 Piece Stage", DesiredMode.MID_3PIECE_STAGE);
        SmartDashboard.putData("Auto Mode", mModeChooser);

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
            return Optional.of(new PPTest("Spin test", new Rotation2d()));
        case STRESS_TEST:
            return Optional.of(new PPTest("Stress Test", new Rotation2d(180)));
        case MID_3PIECE_STAGE:
            return Optional.of(new PPTest("Mid3PieceStage", new Rotation2d(180)));
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
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
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
