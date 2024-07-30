package frc.team4276.frc2024.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.Constants.FlywheelConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class FlywheelSubsystem extends Subsystem {
    private VIKCANSparkMax mTopMotor;
    private VIKCANSparkMax mBottomMotor;

    private RelativeEncoder mTopEncoder;
    private RelativeEncoder mBottomEncoder;

    private SimpleMotorFeedforward mTopFF;
    private SimpleMotorFeedforward mBottomFF;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private boolean mIsOpenLoop = true;

    private static FlywheelSubsystem mInstance;

    public static FlywheelSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new FlywheelSubsystem();
        }

        return mInstance;
    }

    private FlywheelSubsystem(){
        mTopMotor = CANSparkMaxFactory.createDefault(Ports.FLYWHEEL_TOP);
        mBottomMotor = CANSparkMaxFactory.createDefault(Ports.FLYWHEEL_BOTTOM);

        mTopMotor.setInverted(false);
        mBottomMotor.setInverted(true);

        mTopEncoder = mTopMotor.getEncoder();
        mBottomEncoder = mBottomMotor.getEncoder();
        
        mTopMotor.setWantBrakeMode(true);
        mBottomMotor.setWantBrakeMode(true);

        mTopMotor.setSmartCurrentLimit(40);
        mBottomMotor.setSmartCurrentLimit(40);

        mTopEncoder.setAverageDepth(8);
        mBottomEncoder.setAverageDepth(8);

        mTopEncoder.setMeasurementPeriod(10);
        mBottomEncoder.setMeasurementPeriod(10);

        mTopEncoder.setVelocityConversionFactor(1);
        mBottomEncoder.setVelocityConversionFactor(1);

        mTopMotor.burnFlash();
        mBottomMotor.burnFlash();

        mTopFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Top,FlywheelConstants.kV_Top,FlywheelConstants.kA);
        mBottomFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Bottom,FlywheelConstants.kV_Bottom,FlywheelConstants.kA);
    }

    public void setOpenLoop(double voltage){
        setOpenLoop(voltage, voltage);
    }

    public void setOpenLoop(double des_top_voltage, double des_bottom_voltage){
        if (!mIsOpenLoop) {
            mIsOpenLoop = true;
        }

        mPeriodicIO.top_demand = des_top_voltage;
        mPeriodicIO.bottom_demand = des_bottom_voltage;
    }

    public void setTargetRPM(double RPM){
        setTargetRPM(RPM, RPM);
    }

    public void setTargetRPM(double top_RPM, double bottom_RPM){
        if (mIsOpenLoop) {
            mIsOpenLoop = false;
        }

        mPeriodicIO.top_demand = top_RPM;
        mPeriodicIO.top_demand = bottom_RPM;
    }

    @Override
    public void stop() {
        setOpenLoop(0, 0);
    }

    private class PeriodicIO {
        // Inputs
        double curr_top_RPM = 0.0;
        double curr_bottom_RPM = 0.0;

        // Outputs
        double top_demand;
        double bottom_demand;

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.curr_top_RPM = mTopEncoder.getVelocity();
        mPeriodicIO.curr_bottom_RPM = mBottomEncoder.getVelocity();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
        
    }

    @Override
    public void writePeriodicOutputs() {
        if(mIsOpenLoop){
            mTopMotor.setVoltage(mPeriodicIO.top_demand);
            mBottomMotor.setVoltage(mPeriodicIO.top_demand);

        } else {
            mTopMotor.setVoltage(mTopFF.calculate(mPeriodicIO.top_demand));
            mTopMotor.setVoltage(mBottomFF.calculate(mPeriodicIO.bottom_demand));
            
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Top RPM", mPeriodicIO.curr_top_RPM);
        SmartDashboard.putNumber("Bottom RPM", mPeriodicIO.curr_bottom_RPM);
    }
}
