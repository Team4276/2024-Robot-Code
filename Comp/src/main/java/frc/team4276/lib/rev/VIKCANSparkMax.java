package frc.team4276.lib.rev;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;

import frc.team4276.lib.util.Util;

public class VIKCANSparkMax extends CANSparkMax {
    protected Supplier<Boolean> mForwardLimit;
    protected Supplier<Boolean> mReverseLimit;

    public VIKCANSparkMax(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    @Override
    public REVLibError burnFlash() {
        REVLibError e = super.burnFlash();

        requestPeriodicFrames();

        return e;
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

    double[] mQueuedPeriodicFrameTimes = {
        0.01,
        0.02,
        0.02,
        1.00,
        1.00,
        0.20,
        0.20,
        1.00
    };

    /**
     * Queued requests are set after burnflash
     */
    public void queuePeriodicFramePeriodSec(PeriodicFrame frame, double periodSec) {
        mQueuedPeriodicFrameTimes[frame.value] = periodSec;
    }
    
    //TODO: figure out wth is wrong with this

    /**
     * Sends set requests for periodic frames
     * Called automatically after burnflash
     * Call on motor reset
     */
    public void requestPeriodicFrames(){
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus0, mQueuedPeriodicFrameTimes[0]); // Applied Output / Faults / Follower Sends
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus1, mQueuedPeriodicFrameTimes[1]); // Voltage / Temp / Current / Internal Encoder Vel
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus2, mQueuedPeriodicFrameTimes[2]); // Internal Encoder Pos
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus3, mQueuedPeriodicFrameTimes[3]); // Analog Sensor
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus4, mQueuedPeriodicFrameTimes[4]); // Alternate Encoder
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus5, mQueuedPeriodicFrameTimes[5]); // Duty Cycle Absolute Encoder Pos
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus6, mQueuedPeriodicFrameTimes[6]); // Duty Cycle Absolute Encoder Vel / Sen Freq
        setPeriodicFramePeriodSec(PeriodicFrame.kStatus7, mQueuedPeriodicFrameTimes[7]); // No Documentation
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

    /**
     * @param zero current position
     */
    public void zeroAbsoluteEncoder(double zero) {
        AbsoluteEncoder e = getAbsoluteEncoder();

        e.setZeroOffset(Util.rescopeAngle(e.getZeroOffset() + e.getPosition() - zero, e.getPositionConversionFactor()));
    }

    public void zeroAbsoluteEncoder() {
        zeroAbsoluteEncoder(0.0);
    }


}
