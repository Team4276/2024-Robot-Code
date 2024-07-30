package frc.team4276.lib.motion;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import frc.team254.lib.util.Util;

//TODO: implement
public class ProfileFollower{
    protected double mKp;
    protected double mKi;
    protected double mKv;
    protected double mKffv;
    protected double mKffa;
    protected double mKffs;

    protected double mTotalError = 0;

    protected boolean isContinuous = false;
    protected double kMinInput;
    protected double kMaxInput;

    protected double kMinOutput = 1;
    protected double kMaxOutput = 1;

    protected double mKTol;
    protected double mKXTol;
    protected double mKDxTol;

    protected boolean atSetpoint;

    protected double prev_t = 0;

    public static class ProfileFollowerConstants {
        public double kP;
        public double kI;
        public double kV;
        public double kFFV;
        public double kFFA;
        public double kFFS;
        public double kTol;
        public double kXTol;
        public double kDxTol;
    }

    public ProfileFollower(ProfileFollowerConstants constants){
        this.mKp = constants.kP;
        this.mKi = constants.kI;
        this.mKv = constants.kV;
        this.mKffv = constants.kFFV;
        this.mKffa = constants.kFFA;
        this.mKffs = constants.kFFS;
        this.mKTol = constants.kTol;
        this.mKXTol = constants.kXTol;
        this.mKDxTol = constants.kDxTol;
    }

    public void reset(){
        mKp = 0;
        mKi = 0;
        mKv = 0;
        mKffv = 0;
        mKffa = 0;
        mKffs = 0;
        mKTol = 0;
        mKXTol = 0;
        mKDxTol = 0;

        mTotalError = 0;

        isContinuous = false;
    }

    public void resetIntegral(){
        mTotalError = 0;
    }

    public void setGains(double kP, double kI, double kV, double kFFV, double kFFA, double kFFS){
        this.mKp = kP;
        this.mKi = kI;
        this.mKv = kV;
        this.mKffv = kFFV;
        this.mKffa = kFFA;
        this.mKffs = kFFS;
    }

    public void enableContinuousInput(double kMinInput, double kMaxInput){
        isContinuous = true;

        this.kMinInput = kMinInput;
        this.kMaxInput = kMaxInput;
    }

    public void setOutputRange(double kMinOutput, double kMaxOutput){
        this.kMinOutput = kMinOutput;
        this.kMaxOutput = kMaxOutput;
    }

    /**
     * Sets tolerance for 
     * @param kTol
     */
    public void setTolerance(double kTol){
        this.mKTol = kTol;
    }

    public void setPositionTolerance(double kXTol){
        this.mKXTol = kXTol;
    }

    public void setVelocityTolerance(double kDxTol){
        this.mKDxTol = kDxTol;
    }

    public double calculate(double timeStamp, State des_state, State curr_state, State prev_state){
        State use_state = Math.abs(prev_state.position - curr_state.position) > mKXTol 
            || Math.abs(prev_state.velocity - curr_state.velocity) > mKDxTol ? curr_state : prev_state;

        final double dt = Math.max(0, timeStamp - prev_t);

        double pos_error = use_state.position - des_state.position;
        double vel_error = use_state.velocity - des_state.velocity;

        double output = (pos_error * mKp);

        output += (vel_error * mKv) + (use_state.velocity * mKffv) + (0 * mKffa);
        
        if(!Util.epsilonEquals(output, 0.0)){
            output += mKffs * Math.signum(output);
        }

        if (kMinOutput <= output && output <= kMaxOutput){
            mTotalError += pos_error * dt;
            output += mKi * mTotalError;
        } else {
            mTotalError = 0;
        }

        output = Math.min(kMaxOutput, Math.max(kMinOutput, output));

        return output;

    }

    public boolean atSetPoint(){
        return atSetpoint;
    }


}