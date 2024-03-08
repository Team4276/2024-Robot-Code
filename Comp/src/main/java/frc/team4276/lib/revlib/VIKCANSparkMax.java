package frc.team4276.lib.revlib;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

/**
 * Added features to CANSparkMax class
 */
public class VIKCANSparkMax extends CANSparkMax {
    protected SparkLimitSwitch mForwardLimitSwitch;
    protected SparkLimitSwitch mReverseLimitSwitch;

    protected boolean mForwardCancelNegative;
    protected boolean mReverseCancelNegative;

    protected EncoderLimit mPositiveEncoderLimit = null;
    protected EncoderLimit mNegativeEncoderLimit = null;

    protected DoubleSupplier mPositionSupplierForLimiting;

    public VIKCANSparkMax(int id, MotorType motorType) {
        super(id, motorType);
    }

    /**
     * Do not call during voltage or power updates
     * 
     * @param forward
     */
    public void disableLimitSwitch(boolean forward) {
        if (forward) {
            mForwardLimitSwitch = null;
            return;
        }

        mReverseLimitSwitch = null;
    }

    /**
     * Do not call during voltage or power updates
     * 
     * @param enable
     * @param limitSwitchType
     * @param cancelNegative  if true then cancels any speed or voltage assignments
     *                        that are negative
     */
    public void enableForwardLimitSwitch(Type limitSwitchType, boolean cancelNegative) {
        mForwardLimitSwitch = super.getForwardLimitSwitch(limitSwitchType);
        mForwardLimitSwitch.enableLimitSwitch(false);
        mForwardCancelNegative = cancelNegative;
    }

    /**
     * Do not call during voltage or power updates
     * 
     * @param enable
     * @param limitSwitchType
     * @param cancelNegative  if true then cancels any speed or voltage assignments
     *                        that are negative
     */
    public void enableReverseLimitSwitch(Type limitSwitchType, boolean cancelNegative) {
        mReverseLimitSwitch = super.getReverseLimitSwitch(limitSwitchType);
        mReverseLimitSwitch.enableLimitSwitch(false);
        mReverseCancelNegative = cancelNegative;

    }

    public boolean isForwardLimitPressed() {
        if (mForwardLimitSwitch == null)
            return false;
        return mForwardLimitSwitch.isPressed();
    }

    public boolean isReverseLimitPressed() {
        if (mReverseLimitSwitch == null)
            return false;
        return mReverseLimitSwitch.isPressed();
    }

    public void setEncoderForLimits(DoubleSupplier supplier) {
        mPositionSupplierForLimiting = supplier;
    }

    public void setVoltage(double output_volts) {
        if (output_volts == Double.NaN)
            return;

        super.setVoltage(checkLimits(output_volts));
    }

    public void set(double speed) {
        if (speed == Double.NaN)
            return;

        super.set(checkLimits(speed));
    }

    public static enum Direction {
        POSITIVE,
        NEGATIVE
    }

    private class EncoderLimit {
        double position;
        boolean limit_positive;
        double magnitude;

        EncoderLimit(double position, boolean limit_positive, double magnitude) {
            this.position = position;
            this.limit_positive = limit_positive;
            this.magnitude = magnitude;
        }
    }

    /**
     * Do not call during voltage or power updates
     * 
     * @param position
     * @param direction      positive: limit values greater than given position
     * @param limit_positive should limit positive inputs
     * @param magnitude
     */
    public void setEncoderLimit(double position, Direction direction, boolean limit_positive, double magnitude) {
        if (direction == Direction.POSITIVE) {
            mPositiveEncoderLimit = new EncoderLimit(position, limit_positive, magnitude);
        } else {
            mNegativeEncoderLimit = new EncoderLimit(position, limit_positive, magnitude);
        }
    }

    public void resetEncoderLimit(Direction direction) {
        if (direction == Direction.POSITIVE) {
            mPositiveEncoderLimit = null;
            return;
        }

        mNegativeEncoderLimit = null;
    }

    protected double checkLimits(double output_volts) {
        if (mReverseLimitSwitch != null) {
            if (mReverseLimitSwitch.isPressed()) {
                if (mReverseCancelNegative && output_volts < 0) {
                    return 0.0;
                } else if (!mReverseCancelNegative && output_volts > 0) {
                    return 0.0;
                }
            }
        }

        if (mForwardLimitSwitch != null) {
            if (mForwardLimitSwitch.isPressed()) {
                if (mForwardCancelNegative && output_volts < 0) {
                    return 0.0;
                } else if (!mForwardCancelNegative && output_volts > 0) {
                    return 0.0;
                }
            }
        }

        if (mPositiveEncoderLimit != null) {
            if (mPositionSupplierForLimiting.getAsDouble() > mPositiveEncoderLimit.position) {
                if (mPositiveEncoderLimit.limit_positive && output_volts > 0) {
                    return Math.abs(mPositiveEncoderLimit.magnitude) * 0.0;
                } else if (!mPositiveEncoderLimit.limit_positive && output_volts < 0) {
                    return Math.abs(mPositiveEncoderLimit.magnitude) * 0.0;
                }
            }
        }

        if (mNegativeEncoderLimit != null) {
            if (mPositionSupplierForLimiting.getAsDouble() < mNegativeEncoderLimit.position) {
                if (mNegativeEncoderLimit.limit_positive && output_volts > 0) {
                    return Math.abs(mNegativeEncoderLimit.magnitude) * 0.0;
                } else if (!mNegativeEncoderLimit.limit_positive && output_volts < 0) {
                    return Math.abs(mNegativeEncoderLimit.magnitude) * 0.0;
                }
            }
        }

        return output_volts;
    }
}
