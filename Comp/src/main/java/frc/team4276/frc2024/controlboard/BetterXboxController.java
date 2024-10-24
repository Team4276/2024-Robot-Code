package frc.team4276.frc2024.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import frc.team4276.frc2024.Constants.OIConstants;

public class BetterXboxController extends XboxController {
    public BetterXboxController(int port){
        super(port);
    }

    private boolean isRumbling = false;
    private double rumbleEndTime = -1;

    public void rumble(double time){
        if(!isRumbling){
            rumbleEndTime = time + Timer.getFPGATimestamp();
            isRumbling = true;
        }
    }

    public void update(){
        if(Timer.getFPGATimestamp() < rumbleEndTime){
            setRumble(RumbleType.kBothRumble, 0.5);
            return;
        }

        setRumble(RumbleType.kBothRumble, 0.0);
        isRumbling = false;
    }

    public boolean getPOVUP(){
        if (getPOV() == 0){
            return true;
        } else {
            return false;
        }
    }

    public boolean getPOVRIGHT(){
        if (getPOV() == 90){
            return true;
        } else {
            return false;
        }
    }

    public boolean getPOVDOWN(){
        if (getPOV() == 180){
            return true;
        } else {
            return false;
        }
    }

    public boolean getPOVLEFT(){
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
}
