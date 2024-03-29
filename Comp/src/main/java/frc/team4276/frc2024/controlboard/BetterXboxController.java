package frc.team4276.frc2024.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4276.frc2024.Constants.OIConstants;

public class BetterXboxController extends XboxController {
    public BetterXboxController(int port){
        super(port);
    }

    public void update(){
        if(rumbleStartTime + 0.25 > Timer.getFPGATimestamp()){
            setRumble(RumbleType.kBothRumble, 0.0);
            rumbleStartTime = -1.0;

        }
    }

    public boolean isPOVUPPressed(){
        if (getPOV() == 0){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVRIGHTPressed(){
        if (getPOV() == 90){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVDOWNPressed(){
        if (getPOV() == 180){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVLEFTPressed(){
        if (getPOV() == 270){
            return true;
        } else {
            return false;
        }
    }

    public boolean getLT(){
        if (getLeftTriggerAxis() > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public boolean getRT(){
        if (getRightTriggerAxis() > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public boolean leftYIsPushed(){
        if (Math.abs(getLeftY()) > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public double getLeftYDeadband(){
        return MathUtil.applyDeadband(getLeftY(), OIConstants.kJoystickDeadband);
    }

    public double getLeftXDeadband(){
        return MathUtil.applyDeadband(getLeftX(), OIConstants.kJoystickDeadband);
    }

    public double getRightYDeadband(){
        return MathUtil.applyDeadband(getRightY(), OIConstants.kJoystickDeadband);
    }

    public double getRightXDeadband(){
        return MathUtil.applyDeadband(getRightX(), OIConstants.kJoystickDeadband);
    }

    private double rumbleStartTime = -1.0;
    private boolean hasRumbled = false;

    public void shortRumble(){
        if(!hasRumbled){
            hasRumbled = true;
            rumbleStartTime = Timer.getFPGATimestamp();
            setRumble(RumbleType.kBothRumble, 0.25);

        }
    }
}
