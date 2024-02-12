package frc.team4276.lib.drivers;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;

// Subsystem class for NEO v1.1 Brushless motors

public class ServoMotorSubsystem extends Subsystem {
    private CANSparkMax mMaster;
    private CANSparkMax[] mFollowers;

    private AbsoluteEncoder mEncoder;

    private SparkPIDController mController;
    private ArmFeedforward mFeedforward;

    public class ServoMotorConstants {
        public int id;
        public boolean isInverted; //TODO: check if relative to the master
    }

    public class ServoMotorSubsystemConstants {//TODO: look into avg depth sampling
        public ServoMotorConstants kMasterConstants = new ServoMotorConstants();
        public ServoMotorConstants[] kFollowerConstants = new ServoMotorConstants[0];

        public boolean kIsInverted = false;
        public boolean kIsCircular = false;
        public double kUnitsPerRotation = 1.0;
        public double kOffset = 0; // Set in hardware client
        public double kHomePosition = 0.0;
        public double kMinPosition = Double.NEGATIVE_INFINITY;
        public double kMaxPosition = Double.POSITIVE_INFINITY;

        public double kP = 0.0;

        public IdleMode kIdleMode = IdleMode.kCoast;

        public int kSmartCurrentLimit = 40; 
        public double kVoltageCompensation = 12.0;
    }

    protected ServoMotorSubsystem(ServoMotorSubsystemConstants constants){
        mMaster = new CANSparkMax(constants.kMasterConstants.id, MotorType.kBrushless);
        mMaster.restoreFactoryDefaults();
        mMaster.setInverted(constants.kMasterConstants.isInverted);
        mMaster.enableVoltageCompensation(constants.kVoltageCompensation);
        mMaster.setSmartCurrentLimit(constants.kSmartCurrentLimit);
        mMaster.setIdleMode(constants.kIdleMode);

        for (int i = 0; i < mFollowers.length; i++){
            mFollowers[i] = new CANSparkMax(constants.kFollowerConstants[i].id, MotorType.kBrushless);
            mFollowers[i].setInverted(constants.kMasterConstants.isInverted);
            mFollowers[i].enableVoltageCompensation(constants.kVoltageCompensation);
            mFollowers[i].setSmartCurrentLimit(constants.kSmartCurrentLimit);
            mFollowers[i].setIdleMode(constants.kIdleMode);
        }

        mEncoder = mMaster.getAbsoluteEncoder(Type.kDutyCycle);
        mEncoder.setInverted(constants.kIsInverted);
        mEncoder.setPositionConversionFactor(constants.kUnitsPerRotation);
        mEncoder.setVelocityConversionFactor(constants.kUnitsPerRotation / 60.0);
        mEncoder.setZeroOffset(constants.kOffset);

        mController = mMaster.getPIDController();
        mController.setFeedbackDevice(mEncoder);
        mController.setPositionPIDWrappingEnabled(constants.kIsCircular);
        mController.setPositionPIDWrappingMinInput(constants.kMinPosition);
        mController.setPositionPIDWrappingMinInput(constants.kMaxPosition);
        mController.setP(0);
        mController.setI(0);
        mController.setD(0);
        mController.setReference(0, ControlType.kSmartMotion, 0, 0, ArbFFUnits.kVoltage);

        mMaster.burnFlash();
    }

}
