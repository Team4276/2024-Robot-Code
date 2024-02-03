package frc.team4276.lib.motion;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ProfileFollower{
    private double mKp;
    private double mKi;
    private double mKv;
    private double mKffv;
    private double mKffa;
    private double mKffs;

    private boolean isContinuous = false;
    private double kMinInput;
    private double kMaxInput;

    public static class ProfileFollowerConstants {
        double kP;
        double kI;
        double kV;
        double kFFV;
        double kFFA;
        double kFFS;
    }

    public ProfileFollower(ProfileFollowerConstants constants){
        this.mKp = constants.kP;
        this.mKi = constants.kI;
        this.mKv = constants.kV;
        this.mKffv = constants.kFFV;
        this.mKffa = constants.kFFA;
        this.mKffs = constants.kFFS;
    }

    public void reset(){
        mKp = 0;
        mKi = 0;
        mKv = 0;
        mKffv = 0;
        mKffa = 0;
        mKffs = 0;

        isContinuous = false;
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

    public double calculate(State des_state, State curr_state){
        double output = 0;


        return output;

    }


}