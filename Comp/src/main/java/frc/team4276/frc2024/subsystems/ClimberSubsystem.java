package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.lib.drivers.Subsystem;

public class ClimberSubsystem extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;

    private RelativeEncoder mRightEncoder;
    private RelativeEncoder mLeftEncoder;

    private double mDesiredVoltage;

    public enum DesiredState {
        IDLE,
        RAISE,
        F_LOWER,
        S_LOWER
    }

    private DesiredState mDesiredState = DesiredState.IDLE;
    
    private static ClimberSubsystem mInstance;

    public static ClimberSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new ClimberSubsystem();
        }

        return mInstance;
    }

    private ClimberSubsystem(){
        mRightMotor = new CANSparkMax(15, MotorType.kBrushless);
        mLeftMotor = new CANSparkMax(14, MotorType.kBrushless);

        mRightEncoder = mRightMotor.getEncoder();
        mLeftEncoder = mLeftMotor.getEncoder();

        mRightMotor.restoreFactoryDefaults();
        mLeftMotor.restoreFactoryDefaults();

        mRightMotor.setIdleMode(IdleMode.kBrake);
        mLeftMotor.setIdleMode(IdleMode.kBrake);

        mRightMotor.setSmartCurrentLimit(40);
        mLeftMotor.setSmartCurrentLimit(40);

        mRightMotor.enableVoltageCompensation(12);
        mLeftMotor.enableVoltageCompensation(12);

        mLeftMotor.setInverted(true);

        mRightEncoder.setAverageDepth(2);
        mLeftEncoder.setAverageDepth(2);

        mRightEncoder.setMeasurementPeriod(10);
        mLeftEncoder.setMeasurementPeriod(10);

        mRightEncoder.setVelocityConversionFactor(1);
        mLeftEncoder.setVelocityConversionFactor(1);

        mRightMotor.follow(mLeftMotor, true);

        mRightMotor.burnFlash();
        mLeftMotor.burnFlash();
    }

    public void setDesiredState(DesiredState state){
        mDesiredState = state;
    }

    public void setIdleMode(IdleMode mode){
        if(mode == mLeftMotor.getIdleMode()) return;

        mRightMotor.setIdleMode(mode);
        mLeftMotor.setIdleMode(mode);
    }

    @Override
    public void stop() {
        mDesiredState = DesiredState.IDLE;
    }
    
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setIdleMode(IdleMode.kBrake);
            }

            @Override
            public void onLoop(double timestamp) {
                try{
                switch (mDesiredState) {
                    case IDLE:
                        mDesiredVoltage = 0.0;

                        break;
                    case RAISE:
                        mDesiredVoltage = 3.0;

                        break;
                    case F_LOWER:
                        mDesiredVoltage = -7.0;

                        break;
                    case S_LOWER:
                        mDesiredVoltage = -4.0;

                        break;
                    default:
                        break;
                }
            } catch (Exception e) {
                System.out.println(e.getMessage());
              }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    @Override
    public void writePeriodicOutputs() {
        mLeftMotor.setVoltage(mDesiredVoltage);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Climber State", mDesiredState.toString());
    }

}
