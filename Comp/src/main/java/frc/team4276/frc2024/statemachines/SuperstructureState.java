package frc.team4276.frc2024.statemachines;

import frc.team254.lib.util.Util;
import frc.team4276.frc2024.Constants.SuperstructureConstants;

public class SuperstructureState {
    public double armAngle = SuperstructureConstants.kFourBarConstants.kHomePosition;
    public double flyWheelRPM = 0.0;
    public double feederRPM = 0.0;
    public double intakeRPM = 0.0;
    public double armAllowableError = SuperstructureConstants.kLiberalArmAllowableError;
    public double flyWheelAllowableError = SuperstructureConstants.kLiberalFlyWheelAllowableError;
    public double feederAllowableError = SuperstructureConstants.kLiberalFeederAllowableError;
    public double intakeAllowableError = SuperstructureConstants.kLiberalIntakeAllowableError;

    public SuperstructureState(double armAngle, double flyWheelRPM, double feederRPM, double intakeRPM, 
        double armAllowableError, double flyWheelAllowableError, double feederAllowableError, double intakeAllowableError) {

        this.armAngle = armAngle;
        this.flyWheelRPM = flyWheelRPM;
        this.feederRPM = feederRPM;
        this.intakeRPM = armAngle;
        this.armAllowableError = armAllowableError;
        this.flyWheelAllowableError = flyWheelAllowableError;
        this.feederAllowableError = feederAllowableError;
        this.intakeAllowableError = intakeAllowableError;
    }

    public double getArmAngle() {
        return Util.limit(armAngle, SuperstructureConstants.kFourBarConstants.kMinPosition, SuperstructureConstants.kFourBarConstants.kMaxPosition);
    }
    
    public double getFlyWheelRPM() {
        return Util.limit(flyWheelRPM, SuperstructureConstants.kFourBarConstants.kMinPosition, SuperstructureConstants.kFourBarConstants.kMaxPosition);
    }
    
    public double getFeederRPM() {
        return Util.limit(feederRPM, SuperstructureConstants.kFourBarConstants.kMinPosition, SuperstructureConstants.kFourBarConstants.kMaxPosition);
    }

    public double getIntakeRPM() {
        return Util.limit(intakeRPM, SuperstructureConstants.kFourBarConstants.kMinPosition, SuperstructureConstants.kFourBarConstants.kMaxPosition);
    }

    public boolean isArmInRange(SuperstructureState other) {
        return isArmInRange(other, Math.min(this.armAllowableError, other.armAllowableError));
    }

    public boolean isFlyWheelInRange(SuperstructureState other) {
        return isFlyWheelInRange(other, Math.min(this.flyWheelAllowableError, other.flyWheelAllowableError));
    }

    public boolean isFeederInRange(SuperstructureState other) {
        return isFeederInRange(other, Math.min(this.feederAllowableError, other.feederAllowableError));
    }

    public boolean isIntakeInRange(SuperstructureState other) {
        return isIntakeInRange(other, Math.min(this.intakeAllowableError, other.intakeAllowableError));
    }

    public boolean isArmInRange(SuperstructureState other, double armAllowableError) {
        return Util.epsilonEquals(this.getArmAngle(), other.getArmAngle(), armAllowableError);
    }

    public boolean isFlyWheelInRange(SuperstructureState other, double flyWheelAllowableError) {
        return Util.epsilonEquals(this.getFlyWheelRPM(), other.getFlyWheelRPM(), flyWheelAllowableError);
    }

    public boolean isFeederInRange(SuperstructureState other, double feederAllowableError) {
        return Util.epsilonEquals(this.getFeederRPM(), other.getFeederRPM(), feederAllowableError);
    }

    public boolean isIntakeInRange(SuperstructureState other, double intakeAllowableError) {
        return Util.epsilonEquals(this.getIntakeRPM(), other.getIntakeRPM(), intakeAllowableError);
    }
}
