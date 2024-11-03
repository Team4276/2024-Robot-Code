package frc.team4276.frc2024.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Constants.FlywheelConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team1678.lib.Util;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.requests.Request;

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

    public static FlywheelSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new FlywheelSubsystem();
        }

        return mInstance;
    }

    private FlywheelSubsystem() {
        mTopMotor = CANSparkMaxFactory.createDefault(Ports.FLYWHEEL_TOP);
        mTopMotor.setInverted(true);
        mTopMotor.setIdleMode(FlywheelConstants.kIdleMode);
        mTopMotor.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);

        mTopEncoder = mTopMotor.getEncoder();
        mTopEncoder.setAverageDepth(FlywheelConstants.kAvgSamplingDepth);
        mTopEncoder.setMeasurementPeriod(FlywheelConstants.kMeasurementPeriod);
        mTopEncoder.setVelocityConversionFactor(FlywheelConstants.kUnitsPerRotation);

        mBottomMotor = CANSparkMaxFactory.createDefault(Ports.FLYWHEEL_BOTTOM);
        mBottomMotor.setInverted(true);
        mBottomMotor.setIdleMode(FlywheelConstants.kIdleMode);
        mBottomMotor.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);

        mBottomEncoder = mBottomMotor.getEncoder();
        mBottomEncoder.setAverageDepth(FlywheelConstants.kAvgSamplingDepth);
        mBottomEncoder.setMeasurementPeriod(FlywheelConstants.kMeasurementPeriod);
        mBottomEncoder.setVelocityConversionFactor(FlywheelConstants.kUnitsPerRotation);

        mTopMotor.burnFlash();
        mBottomMotor.burnFlash();

        mTopFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Top, FlywheelConstants.kV_Top, FlywheelConstants.kA);
        mBottomFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Bottom, FlywheelConstants.kV_Bottom,
                FlywheelConstants.kA);
    }

    public Request rpmRequest(double RPM) {
        return rpmRequest(RPM, RPM);
    }

    public Request rpmRequest(double topRPM, double botRPM) {
        return new Request() {
            @Override
            public void act() {
                setTargetRPM(topRPM, botRPM);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

    }


    public void setOpenLoop(double voltage) {
        setOpenLoop(voltage, voltage);
    }

    public void setOpenLoop(double des_top_voltage, double des_bottom_voltage) {
        if (!mIsOpenLoop) {
            mIsOpenLoop = true;
        }

        mPeriodicIO.top_demand = des_top_voltage;
        mPeriodicIO.bottom_demand = des_bottom_voltage;
    }

    public void setTargetRPM(double RPM) {
        setTargetRPM(RPM, RPM);
    }

    public void setTargetRPM(double top_RPM, double bottom_RPM) {
        if (mIsOpenLoop) {
            mIsOpenLoop = false;
        }

        mPeriodicIO.des_top_rpm = top_RPM;
        mPeriodicIO.des_bot_rpm = bottom_RPM;
        mPeriodicIO.top_demand = mTopFF.calculate(top_RPM);
        mPeriodicIO.bottom_demand = mBottomFF.calculate(bottom_RPM);
    }

    public boolean isSpunUp() {
        return isTopSpunUp() && isBottomSpunUp();
    }

    public boolean isTopSpunUp() {
        return Util.epsilonEquals(mPeriodicIO.des_top_rpm, mPeriodicIO.meas_top_RPM, FlywheelConstants.kFlywheelTolerance);
    }

    public boolean isBottomSpunUp() {
        return Util.epsilonEquals(mPeriodicIO.des_bot_rpm, mPeriodicIO.meas_bottom_RPM, FlywheelConstants.kFlywheelTolerance);
    }

    @Override
    public void stop() {
        setOpenLoop(0, 0);
    }

    private class PeriodicIO {
        // Inputs
        double meas_top_RPM = 0.0;
        double meas_bottom_RPM = 0.0;
        double top_voltage = 0.0;
        double bottom_voltage = 0.0;
        double des_top_rpm = 0.0;
        double des_bot_rpm = 0.0;

        // Outputs
        double top_demand;
        double bottom_demand;

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.meas_top_RPM = mTopEncoder.getVelocity();
        mPeriodicIO.meas_bottom_RPM = mBottomEncoder.getVelocity();

        mPeriodicIO.top_voltage = mTopMotor.getAppliedVoltage();
        mPeriodicIO.bottom_voltage = mBottomMotor.getAppliedVoltage();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setOpenLoop(0.0);
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });

    }

    @Override
    public void writePeriodicOutputs() {
        mTopMotor.setVoltage(mPeriodicIO.top_demand);
        mBottomMotor.setVoltage(mPeriodicIO.bottom_demand);
    }

    @Override
    public void outputTelemetry() {

        if(Constants.disableExtraTelemetry) return;
        
        SmartDashboard.putBoolean("Debug/Flywheels Spun Up", isSpunUp());
        SmartDashboard.putNumber("Debug/Des Top RPM", mPeriodicIO.des_top_rpm);
        SmartDashboard.putNumber("Debug/Des Bottom RPM", mPeriodicIO.des_bot_rpm);
        SmartDashboard.putNumber("Debug/Top RPM", mPeriodicIO.meas_top_RPM);
        SmartDashboard.putNumber("Debug/Bottom RPM", mPeriodicIO.meas_bottom_RPM);
        SmartDashboard.putNumber("Debug/Top Voltage", mPeriodicIO.top_voltage);
        SmartDashboard.putNumber("Debug/Bottom Voltage", mPeriodicIO.bottom_voltage);
    }
}
