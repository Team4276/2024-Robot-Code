package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public abstract class ServoMotorSubsystem extends Subsystem {

    private CANSparkMax mLeader;
    private CANSparkMax[] mFollowers;

    private SparkMaxPIDController pidController;

    private ArmFeedforward ff;

    private ServoMotorSubsystemConstants mConstants;

    public static class CANSparkMaxConstants {
        public int id = -1;
        public boolean invertMotor = false;
        public IdleMode idleMode = IdleMode.kBrake;
    }

    // Recommend initializing in a static block!
    public static class ServoMotorSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public double kLooperDt = 0.01;

        public CANSparkMaxConstants kLeaderConstants = new CANSparkMaxConstants();
        public CANSparkMaxConstants[] kFollowerConstants = new CANSparkMaxConstants[0];

        public double kHomePosition = 0.0; // Radians
        public double kTicksPerUnitDistance = 1.0;
        public double kKp = 0; // Raw output / raw error
        public double kKi = 0; // Raw output / sum of raw error
        public double kKd = 0; // Raw output / (err - prevErr)
        public double kKf = 0; // Raw output / velocity in ticks/100ms
        public double kKa = 0; // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public int kCruiseVelocity = 0; // Ticks / 100ms
        public int kAcceleration = 0; // Ticks / 100ms / s
        public double kRampRate = 0.0; // s
        public double kMaxVoltage = 12.0;

        public int kCurrentLimit = 20; // amps

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;
    }

    protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
        mConstants = constants;
        mLeader = new CANSparkMax(mConstants.kLeaderConstants.id, MotorType.kBrushless);
        for (int i = 0; i < mFollowers.length; i++) {
            mFollowers[i] = new CANSparkMax(mConstants.kFollowerConstants[i].id, MotorType.kBrushless);
            mFollowers[i].setSmartCurrentLimit(mConstants.kCurrentLimit);
            mFollowers[i].setIdleMode(mConstants.kFollowerConstants[i].idleMode);
            mFollowers[i].follow(mLeader, mConstants.kFollowerConstants[i].invertMotor);
            mFollowers[i].burnFlash();
        }

        ff = new ArmFeedforward(0, 0, 0, 0);

        pidController = mLeader.getPIDController();

    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double position_ticks;
        public double position_units;
        public double velocity_ticks_per_100ms;
        public double active_trajectory_position; // ticks
        public double active_trajectory_velocity; // ticks/100ms
        public double active_trajectory_acceleration; // ticks/100ms/s
        public double output_percent;
        public double output_voltage;
        public double error_ticks;

        // OUTPUTS
        public double demand;
        public double feedforward;
    }

    protected enum ControlState {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }

    protected PeriodicIO mPeriodicIO = new PeriodicIO();
    protected ControlState mControlState = ControlState.OPEN_LOOP;

    private void update() {
        pidController.setSmartMotionMaxVelocity(0, 0);

    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
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
    public synchronized void zeroSensors() {

    }

    @Override
    public void stop() {
        mLeader.set(0);
    }

}
