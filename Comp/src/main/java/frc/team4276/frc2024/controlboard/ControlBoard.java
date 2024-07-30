package frc.team4276.frc2024.controlboard;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.Constants.OIConstants;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

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
        
        climberSetting = new DigitalInput(Ports.CLIMBER_BRAKE_SWITCH);
        fourbarSetting = new DigitalInput(Ports.FOURBAR_BRAKE_SWITCH);
    }

    public void update(){
        driver.update();
        operator.update();
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
    boolean hasReleased = false;
    public boolean wantDemoLimits(){
        if(!driver.isPOVUPPressed()){
            hasReleased = true;
            
        }

        if(driver.isPOVUPPressed() && hasReleased){
            hasReleased = false;
            isDemo = !isDemo;
        }

        return isDemo;
    }

    public boolean wantIntake(){
        return driver.getRT();
    }
    
    public boolean wantExhaust(){
        return driver.getBButton();
    }

    public boolean wantSlowLowerClimber(){
        return driver.getLeftBumper();
    }

    public boolean wantLowerClimber(){
        return driver.getLT();
    }

    public boolean wantAutoLock(){
        return driver.getYButton();
    }

    // Operator Controls
    public boolean wantClimbMode(){
        return true;
    }

    public boolean wantStow(){
        return operator.getLeftStickButtonPressed();
    }

    public boolean wantPrep(){
        return operator.isPOVUPPressed();
    }

    public boolean wantDynamic(){
        return operator.getLeftBumperPressed();
    }

    public boolean wantShoot(){
        return operator.getRT();
    }

    public boolean wantIntakeReverse(){
        return operator.getXButton();
    }

    public boolean wantRaiseClimber(){
        return operator.getRightBumper();
    }

    // Robot Button Board
    public boolean wantClimberCoastMode(){
        return climberSetting.get();
    }

    public boolean wantFourbarCoastMode(){
        return fourbarSetting.get();
    }


}