package frc.team4276.lib.motion;

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

    public ProfileFollower(double kP, double kI, double kV, double kFFV, double kFFA, double kFFS){
        this.mKp = kP;
        this.mKi = kI;
        this.mKv = kV;
        this.mKffv = kFFV;
        this.mKffa = kFFA;
        this.mKffs = kFFS;
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


}