package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Constants.FlywheelConstants;
import frc.team4276.lib.drivers.Subsystem;

public class FlywheelSubsystem extends Subsystem {
    private CANSparkMax mTopMotor;
    private CANSparkMax mBottomMotor;

    private RelativeEncoder mTopEncoder;
    private RelativeEncoder mBottomEncoder;

    private SimpleMotorFeedforward mTopFF;
    private SimpleMotorFeedforward mBottomFF;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private DesiredMode mDesiredMode = DesiredMode.VOLTAGE;

    private boolean topIdleFlip = true;
    private boolean useEncoder = true;
    private double flipDelay = 0.0;

    private static FlywheelSubsystem mInstance;

    public static synchronized FlywheelSubsystem getInstance(){
        if (mInstance == null){
            mInstance = new FlywheelSubsystem();
        }

        return mInstance;
    }

    private FlywheelSubsystem(){
        mTopMotor = new CANSparkMax(18, MotorType.kBrushless);
        mBottomMotor = new CANSparkMax(20, MotorType.kBrushless);

        mTopMotor.setInverted(false);
        mBottomMotor.setInverted(false);

        mTopEncoder = mTopMotor.getEncoder();
        mBottomEncoder = mBottomMotor.getEncoder();

        mTopMotor.restoreFactoryDefaults();
        mBottomMotor.restoreFactoryDefaults();
        
        mTopMotor.setIdleMode(IdleMode.kBrake);
        mBottomMotor.setIdleMode(IdleMode.kBrake);

        mTopMotor.setSmartCurrentLimit(60);
        mBottomMotor.setSmartCurrentLimit(60);

        mTopMotor.enableVoltageCompensation(12);
        mBottomMotor.enableVoltageCompensation(12);

        mTopEncoder.setAverageDepth(2);
        mBottomEncoder.setAverageDepth(2);

        mTopEncoder.setMeasurementPeriod(10);
        mBottomEncoder.setMeasurementPeriod(10);

        mTopEncoder.setVelocityConversionFactor(1);
        mBottomEncoder.setVelocityConversionFactor(1);

        mTopMotor.burnFlash();
        mBottomMotor.burnFlash();

        mTopFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Top,FlywheelConstants.kV,FlywheelConstants.kA);
        mBottomFF = new SimpleMotorFeedforward(FlywheelConstants.kS_Bottom,FlywheelConstants.kV,FlywheelConstants.kA);
    }

    public void setVoltage(double des_top_voltage, double des_bottom_voltage){
        if (mDesiredMode != DesiredMode.VOLTAGE) {
            mDesiredMode = DesiredMode.VOLTAGE;
        }

        mPeriodicIO.des_top_voltage = des_top_voltage;
        mPeriodicIO.des_bottom_voltage = des_bottom_voltage;
    }

    public void setTargetRPM(double des_top_RPM, double des_bottom_RPM){
        if (mDesiredMode != DesiredMode.RPM) {
            mDesiredMode = DesiredMode.RPM;
        }

        mPeriodicIO.des_top_RPM = des_top_RPM;
        mPeriodicIO.des_bottom_RPM = des_bottom_RPM;
    }

    public void setDefeeding() {
        if (mDesiredMode != DesiredMode.DEFEEDING) {
            mDesiredMode = DesiredMode.DEFEEDING;
        }
    }

    public void setFlip(boolean topIdleFlip, boolean useEncoder, double flipDelay){
        if (mDesiredMode != DesiredMode.WHAT_THE_FLIP) {
            mDesiredMode = DesiredMode.WHAT_THE_FLIP;
        }

        this.topIdleFlip = topIdleFlip;
        this.useEncoder = useEncoder;
        this.flipDelay = flipDelay;

        mPeriodicIO.flipTime = -1;
    }

    public boolean atSetpoint(){
        return false;
    }

    private enum DesiredMode {
        VOLTAGE,
        RPM,
        DEFEEDING,
        WHAT_THE_FLIP
    }

    private class PeriodicIO {
        // Inputs
        double des_top_RPM;
        double des_bottom_RPM;
        double curr_top_RPM;
        double curr_bottom_RPM;
        boolean flipping;
        double flipTime;

        // Outputs
        double des_top_voltage;
        double des_bottom_voltage;

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
            public void onStart(double timestamp) {
                
            }

            @Override
            public void onLoop(double timestamp) {
                switch (mDesiredMode) {
                    case VOLTAGE:
                        return;
                    case RPM:
                        break;
                    case DEFEEDING:
                        break;
                    case WHAT_THE_FLIP:
                        mPeriodicIO.flipping = shouldFlip(timestamp);
                    default:
                        break;
                }
                
                updateRPM();
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
        
    }

    @Override
    public void writePeriodicOutputs() {
        mTopMotor.setVoltage(mPeriodicIO.des_top_voltage);
        mBottomMotor.setVoltage(mPeriodicIO.des_bottom_voltage);
    }

    private void updateRPM(){
        mPeriodicIO.des_top_voltage = mTopFF.calculate(mPeriodicIO.des_top_RPM);
        mPeriodicIO.des_bottom_voltage = mBottomFF.calculate(mPeriodicIO.des_bottom_RPM);

        if (mDesiredMode == DesiredMode.WHAT_THE_FLIP && !mPeriodicIO.flipping) {
            if (topIdleFlip) {
                mPeriodicIO.des_top_voltage = 0.0;
            } else {
                mPeriodicIO.des_bottom_voltage = 0.0;
            }
        }

    }

    private boolean shouldFlip(double timestamp){
        boolean ready;

        if (useEncoder){
            if (topIdleFlip) {
                ready = Math.abs(mPeriodicIO.curr_top_RPM) > FlywheelConstants.kFlywheelAllowableError;
            } else {
                ready = Math.abs(mPeriodicIO.curr_bottom_RPM) > FlywheelConstants.kFlywheelAllowableError;
            }
        } else {
            ready = true;
        }

        if (!ready){
            return false;
        }

        delay(timestamp);

        if (mPeriodicIO.flipTime > timestamp){
            return false;
        }

        return ready;
    }

    private void delay(double timestamp){
        if (mPeriodicIO.flipTime == -1) {
            mPeriodicIO.flipTime = timestamp + flipDelay;
        }
    }
}
