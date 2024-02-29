package frc.team4276.frc2024.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team254.lib.util.Util;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.drivers.FourBarFeedForward;
import frc.team4276.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;

public class SimpleFourbarSubsystem extends Subsystem {
    private CANSparkMax mMaster;

    private AbsoluteEncoder mAbsoluteEncoder;

    private PeriodicIO mPeriodicIO;

    private State mStateSetpoint;
    private double mProfileStartTime;

    private FourBarFeedForward mFourbarFF;
    private TrapezoidProfile mTrapezoidProfile;

    private boolean isCalibrating = true;

    private static SimpleFourbarSubsystem mInstance;

    public static SimpleFourbarSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SimpleFourbarSubsystem(SuperstructureConstants.kFourBarConstants);
        }
        return mInstance;
    }

    private SimpleFourbarSubsystem(ServoMotorSubsystemConstants constants) {
        mMaster = new CANSparkMax(constants.kMasterConstants.id, MotorType.kBrushless);
        mMaster.restoreFactoryDefaults();
        mMaster.enableVoltageCompensation(constants.kVoltageCompensation);
        mMaster.setSmartCurrentLimit(constants.kSmartCurrentLimit);
        mMaster.setIdleMode(constants.kIdleMode);
        // mMaster.setCANTimeout(10);
        // mMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);

        mAbsoluteEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);
        mAbsoluteEncoder.setPositionConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation);
        mAbsoluteEncoder.setInverted(constants.kIsInverted);
        mAbsoluteEncoder.setZeroOffset(constants.kOffset);

        mFourbarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
        mTrapezoidProfile = new TrapezoidProfile(new Constraints(constants.kMaxSpeed, constants.kMaxAccel));

        mMaster.burnFlash();

        mPeriodicIO = new PeriodicIO();

        if(isCalibrating){
            SmartDashboard.putNumber("Calibration Velocity Test", 0.0);
            SmartDashboard.putNumber("Calibration Voltage Test", 0.0);
            SmartDashboard.putNumber("Calibration Efficiency Test", 0.0);
            SmartDashboard.putNumber("Calibration Static Test", 0.0);
        }
    }

    public void setSetpointProfiledFF(double position_radians) {
        if (mStateSetpoint.position == position_radians)
            return;

        mStateSetpoint = new State(position_radians, 0.0);
    }

    private class PeriodicIO {
        // Inputs
        double timestamp;
        double meas_position_units;
        double meas_velocity_units;
        State meas_state;

        double velocity_test;
        double voltage_test;
        double efficiency_test;
        double static_test;

        // Outputs
        double demand;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.meas_position_units = mAbsoluteEncoder.getPosition();
        mPeriodicIO.meas_velocity_units = mAbsoluteEncoder.getVelocity();
        mPeriodicIO.meas_state = new State(mPeriodicIO.meas_position_units, mPeriodicIO.meas_velocity_units);


    }

    @Override
    public void writePeriodicOutputs() {
        if (isCalibrating) {
            SmartDashboard.putNumber("Time Since Start", mPeriodicIO.timestamp - mProfileStartTime);
            SmartDashboard.putNumber("Input Measured State Position", mPeriodicIO.meas_state.position);
            SmartDashboard.putNumber("Input Measured State Velocity", mPeriodicIO.meas_state.velocity);
            SmartDashboard.putNumber("Input Setpoint State Position", mStateSetpoint.position);
            SmartDashboard.putNumber("Input Setpoint State Velocity", mStateSetpoint.velocity);

            mPeriodicIO.velocity_test = SmartDashboard.getNumber("Calibration Velocity Test", 0.0);
            mPeriodicIO.voltage_test = SmartDashboard.getNumber("Calibration Voltage Test", 0.0);
            mPeriodicIO.efficiency_test = SmartDashboard.getNumber("Calibration Efficiency Test", 0.0);
            mPeriodicIO.static_test = SmartDashboard.getNumber("Calibration Static Test", 0.0);

            mFourbarFF.setEfficiency(mPeriodicIO.efficiency_test);
            mFourbarFF.setkS(mPeriodicIO.static_test);

            if(mPeriodicIO.voltage_test != 0.0) {
                mPeriodicIO.demand = mFourbarFF.calculate(mPeriodicIO.meas_position_units, mPeriodicIO.velocity_test);
            }

            mMaster.setVoltage(Util.limit(-mPeriodicIO.demand,4.8));
            SmartDashboard.putNumber("Fourbar Feedforward Voltage", -mPeriodicIO.demand);

            return;
        }

        State state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp - mProfileStartTime, mPeriodicIO.meas_state,
                mStateSetpoint);
        mPeriodicIO.demand = Util.limit(mFourbarFF.calculate(state.position, state.velocity), 4.8);

        mMaster.setVoltage(-mPeriodicIO.demand);
    }

}
