package frc.team4276.frc2024.field;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AllianceChooser {
    private SendableChooser<Alliance> mChooser;

    private Alliance mAlliance = Alliance.Blue;

    private static AllianceChooser mInstance;

    public static AllianceChooser getInstance(){
        if (mInstance == null){
            mInstance = new AllianceChooser();
        }

        return mInstance;
    }

    private AllianceChooser(){
        mChooser = new SendableChooser<Alliance>();

        mChooser.setDefaultOption("Blue", Alliance.Blue);
        mChooser.addOption("Red", Alliance.Red);

        SmartDashboard.putData("Comp/Alliance Chooser", mChooser);
    }

    public Alliance getAlliance(){
        if (DriverStation.isFMSAttached() && DriverStation.getAlliance().isPresent()){
            return DriverStation.getAlliance().get();
        }

        return mChooser.getSelected();
    }

    /** Since last call*/
    public boolean isAllianceChanged(){
        if (mAlliance != getAlliance()){
            mAlliance = getAlliance();
            return true;
        }

        return false;
    }

    public synchronized boolean isAllianceRed(){
        return getAlliance() == Alliance.Red;
    }
}
