package frc.team4276.frc2024.controlboard;

import edu.wpi.first.wpilibj.XboxController;
import frc.team4276.frc2024.Constants.OIConstants;

public class BetterXboxController {
    
    XboxController xboxController;

    public BetterXboxController(XboxController xboxController){
        this.xboxController = xboxController;
    }

    /** Returns 0 for neutral 1-4 from top to left going clockwise and 5 for when it isn't one of these values */
    public int getPOV(){
        int pov;
        if (xboxController.getPOV() == -1){
            pov = 0;
        } else if (xboxController.getPOV() == 0){
            pov = 1;
        } else if (xboxController.getPOV() == 90){
            pov = 2;
        } else if (xboxController.getPOV() == 180){
            pov = 3;
        } else if (xboxController.getPOV() == 270){
            pov = 4;
        } else {
            pov = 5;
        }

        return pov;
    }

    public boolean isPOVUPPressed(){
        if (getPOV() == 1){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVRIGHTPressed(){
        if (getPOV() == 2){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVDOWNPressed(){
        if (getPOV() == 3){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPOVLEFTPressed(){
        if (getPOV() == 4){
            return true;
        } else {
            return false;
        }
    }

    public boolean getLT(){
        if (xboxController.getLeftTriggerAxis() > 0.1){
            return true;
        } else {
            return false;
        }
    }

    public boolean getRT(){
        if (xboxController.getRightTriggerAxis() > 0.1){
            return true;
        } else {
            return false;
        }
    }

    public boolean leftYIsPushed(){
        if (Math.abs(xboxController.getLeftY()) > OIConstants.kJoystickDeadband){
            return true;
        } else {
            return false;
        }
    }

}
