package frc.team4276.frc2024.controlboard;

import frc.team4276.frc2024.Constants.OIConstants;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    public final BetterXboxController driver;
    public final BetterXboxController operator;

    public static ControlBoard getInstance(){
        if (mInstance == null){
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard(){
        driver = new BetterXboxController(OIConstants.kDriverControllerPort);
        operator = new BetterXboxController(OIConstants.kOpControllerPort);
    }

}