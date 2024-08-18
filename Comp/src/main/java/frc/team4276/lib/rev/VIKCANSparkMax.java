package frc.team4276.lib.rev;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public class VIKCANSparkMax extends CANSparkMax {
    protected Supplier<Boolean> mForwardLimit;
    protected Supplier<Boolean> mReverseLimit;

    public VIKCANSparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    public double getAppliedVoltage() {
        return getAppliedOutput() * getBusVoltage();
    }

    public void setWantBrakeMode(boolean brake){
        IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;

        setIdleMode(mode);
    }

    public void setReference(double value, CANSparkBase.ControlType ctrl, int pidSlot, double arbFeedforward, SparkPIDController.ArbFFUnits arbFFUnits) {
        getPIDController().setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

    public void setPeriodicFramePeriodSec(PeriodicFrame frame, double periodSec) {
        setPeriodicFramePeriod(frame, (int)(periodSec * 1000));
    }

    public void enableForwardLimit(Supplier<Boolean> boolSupplier) {
        mForwardLimit = boolSupplier;
    }

    public void disableForwardLimit() {
        mForwardLimit = null;
    }

    public void enableReverseLimit(Supplier<Boolean> boolSupplier) {
        mReverseLimit = boolSupplier;
    }

    public void disableReverseLimit() {
        mReverseLimit = null;
    }

    @Override
    public void setVoltage(double outputVolts) {
        double output = outputVolts;
        
        if(mForwardLimit != null) {
            output = mForwardLimit.get() ? Math.min(0.0, output) : output;
            
        }

        if(mReverseLimit != null) {
            output = mReverseLimit.get() ? Math.max(0.0, output) : output;
            
        }

        super.setVoltage(output);
    }

    //TODO: test
    /**
     * @param zero current position
     */
    public void zeroAbsoluteEncoder(double zero) {
        AbsoluteEncoder e = getAbsoluteEncoder();

        e.setZeroOffset(e.getZeroOffset() + e.getPosition());
    }

    public void zeroAbsoluteEncoder() {
        zeroAbsoluteEncoder(0.0);
    }


}
