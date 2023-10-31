package frc.team4276.frc2024.controlboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4276.frc2024.Constants.OIConstants;

public class BetterXboxController {
    private XboxController mController;

    public BetterXboxController(int port){
        mController = new XboxController(port);
    }

    public boolean isPOVUPPressed(){
        if (mController.getPOV() == 0){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVRIGHTPressed(){
        if (mController.getPOV() == 90){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVDOWNPressed(){
        if (mController.getPOV() == 180){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVLEFTPressed(){
        if (mController.getPOV() == 270){
            return true;
        } else {
            return false;
        }
    }

    public boolean getLT(){
        if (mController.getLeftTriggerAxis() > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public boolean getRT(){
        if (mController.getRightTriggerAxis() > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public boolean leftYIsPushed(){
        if (Math.abs(mController.getLeftY()) > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

    public XboxController getController() {
        return mController;
    }

    public double getLeftY(){
        return MathUtil.applyDeadband(mController.getLeftY(), OIConstants.kJoystickDeadband);
    }

    public double getLeftX(){
        return MathUtil.applyDeadband(mController.getLeftX(), OIConstants.kJoystickDeadband);
    }

    public double getRightY(){
        return MathUtil.applyDeadband(mController.getRightY(), OIConstants.kJoystickDeadband);
    }

    public double getRightX(){
        return MathUtil.applyDeadband(mController.getRightX(), OIConstants.kJoystickDeadband);
    }

}
