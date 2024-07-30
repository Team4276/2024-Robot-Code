// package frc.team4276.frc2024.subsystems;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.SparkLimitSwitch;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase;
// import com.revrobotics.SparkAbsoluteEncoder;

// import frc.team4276.frc2024.Ports;
// import frc.team4276.frc2024.Constants;
// import frc.team4276.lib.characterizations.FourBarFeedForward;
// import frc.team4276.lib.drivers.Subsystem;
// import frc.team4276.lib.drivers.ServoMotorSubsystem;
// import frc.team4276.lib.rev.CANSparkMaxFactory;
// import frc.team4276.lib.rev.VIKCANSparkMaxServo;
// import frc.team4276.lib.motion.TrapezoidProfile;

// import frc.team1678.lib.loops.ILooper;
// import frc.team1678.lib.loops.Loop;

// public class LegacyFourbarSubsystem extends Subsystem {
//     private VIKCANSparkMaxServo mMaster;
//     private VIKCANSparkMaxServo mFollower;

//     private SparkLimitSwitch mLimitSwitchForward;
//     private SparkLimitSwitch mLimitSwitchReverse;

//     private SparkAbsoluteEncoder mAbsoluteEncoder;

//     private PeriodicIO mPeriodicIO;

//     private ControlState mControlState = ControlState.VOLTAGE;

//     public enum ControlState {
//         VOLTAGE,
//         SMART_MOTION, // Onboard PIDF
//         FUSE_MOTION // RIO FF w/ Onboard PID

//     }

//     private FourBarFeedForward mFourbarFF;
//     private TrapezoidProfile mTrapezoidProfile;

//     private SparkPIDController mSparkPIDController;

//     private double kMaxPosition;
//     private double kMinPosition;

//     private static LegacyFourbarSubsystem mInstance;

//     public static LegacyFourbarSubsystem getInstance() {
//         if (mInstance == null) {
//             mInstance = new LegacyFourbarSubsystem(Constants.FourbarConstants.kFourBarConstants);
//         }
//         return mInstance;
//     }

//     private LegacyFourbarSubsystem(ServoMotorSubsystem.ServoMotorSubsystemConstants constants) {
//         mMaster = CANSparkMaxFactory.createDefaultServo(Ports.FOURBAR_MASTER);
//         // TODO: check the defaults
//         // mMaster.setControlFramePeriodMs();
//         mMaster.setSmartCurrentLimit(constants.kSmartCurrentLimit);
//         mMaster.setIdleMode(constants.kIdleMode);
//         mMaster.setInverted(constants.kMasterConstants.isInverted);

//         mLimitSwitchForward = mMaster.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
//         mLimitSwitchForward.enableLimitSwitch(true);

//         mLimitSwitchReverse = mMaster.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
//         mLimitSwitchReverse.enableLimitSwitch(true);

//         mFollower = CANSparkMaxFactory.createDefaultServo(Ports.FOURBAR_FOLLOWER);
//         mFollower.setSmartCurrentLimit(constants.kSmartCurrentLimit);
//         mFollower.setIdleMode(constants.kIdleMode);
//         mFollower.follow(mMaster, constants.kFollowerConstants[0].isInverted);

//         mAbsoluteEncoder = mMaster.getAbsoluteEncoder();
//         mAbsoluteEncoder.setPositionConversionFactor(constants.encoderConfig.kUnitsPerRotation);
//         mAbsoluteEncoder.setVelocityConversionFactor(constants.encoderConfig.kUnitsPerRotation);
//         mAbsoluteEncoder.setInverted(constants.encoderConfig.kIsInverted);
//         mAbsoluteEncoder.setZeroOffset(constants.encoderConfig.kOffset);
//         mAbsoluteEncoder.setAverageDepth(constants.encoderConfig.kAvgSamplingDepth);

//         mSparkPIDController = mMaster.getPIDController();
//         mSparkPIDController.setFeedbackDevice(mAbsoluteEncoder);
//         if (constants.kIsCircular) {
//             mSparkPIDController.setPositionPIDWrappingEnabled(constants.kIsCircular);
//             mSparkPIDController.setPositionPIDWrappingMaxInput(constants.kMaxPosition);
//             mSparkPIDController.setPositionPIDWrappingMinInput(constants.kMaxPosition);
//         }

//         for (int i = 0; i < constants.pidfConfigs.length; i++) {
//             mSparkPIDController.setP(constants.pidfConfigs[i].kP, i);
//             mSparkPIDController.setI(constants.pidfConfigs[i].kI, i);
//             mSparkPIDController.setD(constants.pidfConfigs[i].kD, i);
//             mSparkPIDController.setFF(constants.pidfConfigs[i].kFF, i);
//             mSparkPIDController.setDFilter(constants.pidfConfigs[i].kDFilter, i);
//             mSparkPIDController.setIZone(constants.pidfConfigs[i].kIZone, i);
//             mSparkPIDController.setIMaxAccum(constants.pidfConfigs[i].kIMaxAccum, i);
//             mSparkPIDController.setOutputRange(-constants.pidfConfigs[i].kPIDOutputRange,
//                     constants.pidfConfigs[i].kPIDOutputRange, i);
//         }

//         mFourbarFF = new FourBarFeedForward(constants.kFourBarFFConstants);
//         mTrapezoidProfile = new TrapezoidProfile(constants.kMaxSpeed, constants.kMaxAccel);

//         this.kMaxPosition = constants.kMaxPosition;
//         this.kMinPosition = constants.kMinPosition;

//         mMaster.burnFlash();
//         mFollower.burnFlash();

//         mPeriodicIO = new PeriodicIO();

//     }

//     public void setVoltage(double voltage) {
//         if (mControlState != ControlState.VOLTAGE) {
//             mControlState = ControlState.VOLTAGE;
//         }

//         mPeriodicIO.demand = voltage;
//     }

//     public void setSmartMotionSetpoint(double position_radians) {
//         if (mControlState != ControlState.SMART_MOTION) {
//             mControlState = ControlState.SMART_MOTION;
//         } else if (mPeriodicIO.setpoint == position_radians) {
//             return;
//         }

//         mPeriodicIO.setpoint = position_radians;
//     }

//     public void setFuseMotionSetpoint(double position_radians) {
//         if (mControlState != ControlState.SMART_MOTION) {
//             mControlState = ControlState.SMART_MOTION;
//         } else if (mPeriodicIO.setpoint == position_radians) {
//             return;
//         }

//         mPeriodicIO.setpoint = Math.max(Math.min(position_radians, kMaxPosition), kMinPosition);
//         mPeriodicIO.start_time = mPeriodicIO.timestamp;
//         mPeriodicIO.start[0] = mPeriodicIO.meas_position;
//         mPeriodicIO.start[1] = mPeriodicIO.meas_velocity;

//     }

//     public void setBrakeMode(boolean brake) {
//         CANSparkBase.IdleMode mode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;

//         mMaster.setIdleMode(mode);
//         mFollower.setIdleMode(mode);
//     }

//     public boolean isBrakeMode() {
//         return mMaster.getIdleMode() == CANSparkBase.IdleMode.kBrake;
//     }

//     @Override
//     public void stop() {
//         setVoltage(0.0);
//     }

//     private class PeriodicIO {
//         // Inputs
//         double timestamp;
//         double start_time;

//         double setpoint;

//         double start[] = {Double.NaN, Double.NaN};
//         double goal[] = {Double.NaN, Double.NaN};

//         double meas_position;
//         double meas_velocity;

//         double meas_master_voltage;
//         double meas_follower_voltage;

//         boolean limit_forward_pressed;
//         boolean limit_reverse_pressed;

//         // Outputs
//         double feed_forward;
//         double demand;
//     }

//     @Override
//     public void readPeriodicInputs() {
//         mPeriodicIO.timestamp = Timer.getFPGATimestamp();
//         mPeriodicIO.meas_position = mAbsoluteEncoder.getPosition();
//         mPeriodicIO.meas_velocity = mAbsoluteEncoder.getVelocity();
//         mPeriodicIO.meas_master_voltage = mMaster.getAppliedVoltage();
//         mPeriodicIO.meas_follower_voltage = mFollower.getAppliedVoltage();
//         mPeriodicIO.limit_forward_pressed = mLimitSwitchForward.isPressed();
//         mPeriodicIO.limit_reverse_pressed = mLimitSwitchReverse.isPressed();

//     }

//     @Override
//     public void registerEnabledLoops(ILooper enabledLooper) {
//         enabledLooper.register(new Loop() {
//             @Override
//             public void onStart(double timestamp) {
//                 setBrakeMode(true);
//             }

//             @Override
//             public void onLoop(double timestamp) {
//                 switch (mControlState) {
//                     case VOLTAGE:
//                         break;

//                     case SMART_MOTION:
//                         break;

//                     case FUSE_MOTION:
//                         double[] state = mTrapezoidProfile.calculate(mPeriodicIO.timestamp - mPeriodicIO.start_time,
//                                 mPeriodicIO.start, mPeriodicIO.goal);

//                         mPeriodicIO.demand = state[1];
//                         mPeriodicIO.feed_forward = mFourbarFF.calculate(state[0], state[1]);

//                         break;

//                     default:
//                         break;
//                 }
//             }

//             @Override
//             public void onStop(double timestamp) {
//             }
//         });
//     }

//     @Override
//     public void writePeriodicOutputs() {
//         switch (mControlState) {
//             case VOLTAGE:
//                 mMaster.setVoltage(mPeriodicIO.demand);

//                 break;

//             case SMART_MOTION:
//                 mSparkPIDController.setReference(mPeriodicIO.setpoint, CANSparkBase.ControlType.kSmartMotion, 0,
//                         Constants.FourbarConstants.kFourBarConstants.pidfConfigs[0].kS, SparkPIDController.ArbFFUnits.kVoltage);

//                 break;

//             case FUSE_MOTION:
//                 mSparkPIDController.setReference(mPeriodicIO.demand, CANSparkBase.ControlType.kVelocity, 1, mPeriodicIO.feed_forward,
//                         SparkPIDController.ArbFFUnits.kVoltage);

//                 break;

//             default:
//                 break;
//         }
//     }

//     @Override
//     public void outputTelemetry() {
//         SmartDashboard.putNumber("Fourbar Applied Master Voltage", mPeriodicIO.meas_master_voltage);
//         SmartDashboard.putNumber("Fourbar Applied Follower Voltage", mPeriodicIO.meas_follower_voltage);
//         SmartDashboard.putBoolean("Front Limit", mPeriodicIO.limit_forward_pressed);
//         SmartDashboard.putBoolean("Back Limit", mPeriodicIO.limit_reverse_pressed);
//         SmartDashboard.putString("Simple Fourbar Controlstate", mControlState.name());
//         SmartDashboard.putNumber("Fourbar Position Degrees", Math.toDegrees(mPeriodicIO.meas_position));

//     }

// }
