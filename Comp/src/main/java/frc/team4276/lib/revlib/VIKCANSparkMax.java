package frc.team4276.lib.revlib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

/**
 * Wrapper for CANSparkMax class
 */
public class VIKCANSparkMax extends CANSparkMax {
    protected boolean isForwardLimitSwitchEnabled = false;
    protected boolean isReverseLimitSwitchEnabled = false;

    protected SparkLimitSwitch mForwardLimitSwitch;
    protected SparkLimitSwitch mReverseLimitSwitch;

    protected Type mForwardLimitSwitchType;
    protected Type mReverseLimitSwitchType;

    protected boolean mForwardCancelNegative;
    protected boolean mReverseCancelNegative;

    public VIKCANSparkMax(int id, MotorType motorType) {
        super(id, motorType);
    }

    /**
     * @param enable
     * @param limitSwitchType
     * @param cancelNegative  if true then cancels any speed or voltage assignments
     *                        that are negative
     */
    public void enableForwardLimitSwitch(boolean enable, Type limitSwitchType, boolean cancelNegative) {
        isForwardLimitSwitchEnabled = enable;
        mForwardLimitSwitchType = limitSwitchType;
        mForwardCancelNegative = cancelNegative;
    }

    /**
     * @param enable
     * @param limitSwitchType
     * @param cancelNegative  if true then cancels any speed or voltage assignments
     *                        that are negative
     */
    public void enableReverseLimitSwitch(boolean enable, Type limitSwitchType, boolean cancelNegative) {
        isReverseLimitSwitchEnabled = enable;
        mReverseLimitSwitchType = limitSwitchType;
        mReverseCancelNegative = cancelNegative;
    }

    public boolean isForwardLimitPressed(){
        if(mForwardLimitSwitch == null) return false;
        return mForwardLimitSwitch.isPressed();
    }

    public boolean isReverseLimitPressed(){
        if(mReverseLimitSwitch == null) return false;
        return mReverseLimitSwitch.isPressed();
    }

    public void setVoltage(double output_volts) {
        super.setVoltage(checkLimits(output_volts));
    }

    public void set(double speed) {
        super.set(checkLimits(speed));
    }

    protected double checkLimits(double output_volts) {
        if (isReverseLimitSwitchEnabled) {
            if (mReverseLimitSwitch == null) {
                mReverseLimitSwitch = super.getReverseLimitSwitch(mReverseLimitSwitchType);
                mReverseLimitSwitch.enableLimitSwitch(false);
            }

            if(mReverseLimitSwitch.isPressed()){
                if(mReverseCancelNegative && output_volts < 0){
                    return 0.0;
                } else if(!mReverseCancelNegative && output_volts > 0){
                    return 0.0;
                }
            }
        }

        if (isForwardLimitSwitchEnabled) {
            if (mForwardLimitSwitch == null) {
                mForwardLimitSwitch = super.getForwardLimitSwitch(mForwardLimitSwitchType);
                mForwardLimitSwitch.enableLimitSwitch(false);
            }

            if(mForwardLimitSwitch.isPressed()){
                if(mForwardCancelNegative && output_volts < 0){
                    return 0.0;
                } else if(!mForwardCancelNegative && output_volts > 0){
                    return 0.0;
                }
            }
        }

        return output_volts;
    }
}
