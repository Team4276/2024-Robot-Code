package frc.team4276.frc2024.controlboard;

import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.team1678.lib.Util;

public class ControlBoard {
    public final BetterXboxController driver;
    public final BetterXboxController operator;

    private final DigitalInput climberSetting;
    private final DigitalInput fourbarSetting;

    private static ControlBoard mInstance;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private ControlBoard() {
        driver = new BetterXboxController(OIConstants.kDriverControllerPort);
        operator = new BetterXboxController(OIConstants.kOpControllerPort);

        climberSetting = new DigitalInput(6);
        fourbarSetting = new DigitalInput(7);
    }

    // Driver Controls

    public Translation2d getSwerveTranslation() {
        double forwardAxis = -driver.getLeftY();
        double strafeAxis = -driver.getLeftX();

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < OIConstants.kJoystickDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, OIConstants.kJoystickDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y)
                    .scale(DriveSubsystem.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    private double kMaxTurnMagnitude = 0.75;

    public double getSwerveRotation() {
        double rotAxis = -driver.getRightX();

        if (Math.abs(rotAxis) < OIConstants.kJoystickDeadband) {
            return 0.0;
        } else {
            return kMaxTurnMagnitude * DriveSubsystem.getInstance().getKinematicLimits().kMaxAngularVelocity
                    * (rotAxis - (Math.signum(rotAxis) * OIConstants.kJoystickDeadband))
                    / (1 - OIConstants.kJoystickDeadband);
        }
    }

    public boolean wantZeroHeading(){
        return driver.getAButtonPressed();
    }

    public boolean wantXBrake(){
        return driver.getXButton();
    }

    boolean isDemo = false;
    boolean hasPressed = false;
    public boolean wantDemoLimits(){
        if(hasPressed && !driver.isPOVUPPressed()){
            hasPressed = false;
            if(isDemo){
                isDemo = false;
            } else {
                isDemo = true;
            }
            
        } 

        if(driver.isPOVUPPressed()){
            hasPressed = true;
        }

        return isDemo;
    }

    public boolean wantSlowtake(){
        return driver.getRightBumper();
    }

    public boolean wantFastake(){
        return driver.getRT();
    }
    
    public boolean wantPoop(){
        return driver.getBButton();
    }

    public boolean wantSLowerClimber(){
        return driver.getLeftBumper();
    }

    public boolean wantFLowerClimber(){
        return driver.getLT();
    }


    // Operator Controls
    public boolean wantStow(){
        return operator.getLeftStickButtonPressed();
    }

    public boolean wantReadyMiddle(){
        return operator.isPOVUPPressed();
    }

    public boolean wantReadyLow(){
        return operator.isPOVDOWNPressed();
    }

    public boolean wantSubClose(){
        return operator.isPOVRIGHTPressed();
    }

    public boolean wantPodium(){
        return operator.isPOVLEFTPressed();
    }

    public boolean wantAmp(){
        return operator.getYButtonPressed();
    }

    public boolean wantAutoLock(){
        return operator.getLeftBumperPressed();
    }

    // public boolean wantScore(){
    //     return operator.getRT();
    // }

    private boolean wantAutoScore = false;

    public boolean wantAutoScore(){
        if(!operator.getBButton()) return wantAutoScore;

        if(wantAutoScore){
            wantAutoScore = false;
        } else {
            wantAutoScore = true;
        }

        return wantAutoScore;
    }

    public boolean wantFoot(){
        return operator.getRT();
    }

    public boolean wantIntakeReverse(){
        return operator.getXButton();
    }

    public boolean wantRaiseClimber(){
        return operator.getRightBumper();
    }

    public boolean wantQueuedState(){
        return operator.getLT();
    }


    // Robot Button Board
    public boolean wantClimberCoastMode(){
        return climberSetting.get();
    }

    public boolean wantFourbarCoastMode(){
        return fourbarSetting.get();
    }


}