package frc.team4276.frc2024.subsystems.drive.flywheel;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2024.Ports;
import frc.team4276.lib.rev.SparkMaxFactory;
import frc.team4276.lib.rev.VIKSparkMax;

public class FlywheelIOSpark implements FlywheelIO {
  private VIKSparkMax mTopMotor;
  private VIKSparkMax mBottomMotor;

  private RelativeEncoder mTopEncoder;
  private RelativeEncoder mBottomEncoder;

  public FlywheelIOSpark() {
    mTopMotor = SparkMaxFactory.createDefault(Ports.FLYWHEEL_TOP);
    mTopMotor.setInverted(true);
    mTopMotor.setIdleMode(FlywheelConstants.kIdleMode);
    mTopMotor.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);

    mTopEncoder = mTopMotor.getEncoder();
    mTopEncoder.setAverageDepth(FlywheelConstants.kAvgSamplingDepth);
    mTopEncoder.setMeasurementPeriod(FlywheelConstants.kMeasurementPeriod);
    mTopEncoder.setVelocityConversionFactor(FlywheelConstants.kUnitsPerRotation);

    mBottomMotor = SparkMaxFactory.createDefault(Ports.FLYWHEEL_BOTTOM);
    mBottomMotor.setInverted(true);
    mBottomMotor.setIdleMode(FlywheelConstants.kIdleMode);
    mBottomMotor.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);

    mBottomEncoder = mBottomMotor.getEncoder();
    mBottomEncoder.setAverageDepth(FlywheelConstants.kAvgSamplingDepth);
    mBottomEncoder.setMeasurementPeriod(FlywheelConstants.kMeasurementPeriod);
    mBottomEncoder.setVelocityConversionFactor(FlywheelConstants.kUnitsPerRotation);

    mTopMotor.burnFlash();
    mBottomMotor.burnFlash();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.leftPositionRads = Units.rotationsToRadians(mTopEncoder.getPosition());
    inputs.leftVelocityRpm = mTopEncoder.getVelocity();
    inputs.leftAppliedVolts = mTopMotor.getAppliedOutput() * mTopMotor.getBusVoltage();
    inputs.leftSupplyCurrentAmps = mTopMotor.getOutputCurrent();
    inputs.leftTempCelsius = mTopMotor.getMotorTemperature();

    inputs.rightPositionRads = Units.rotationsToRadians(mBottomEncoder.getPosition());
    inputs.rightVelocityRpm = mBottomEncoder.getVelocity();
    inputs.rightAppliedVolts = mBottomMotor.getAppliedOutput() * mBottomMotor.getBusVoltage();
    inputs.rightSupplyCurrentAmps = mBottomMotor.getOutputCurrent();
    inputs.rightTempCelsius = mBottomMotor.getMotorTemperature();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    mTopMotor.setVoltage(leftVolts);
    mBottomMotor.setVoltage(rightVolts);
  }

  @Override
  public void runVelocity(double topFeedforward, double bottomFeedforward) {
    mTopMotor.setVoltage(topFeedforward);
    mBottomMotor.setVoltage(bottomFeedforward);
  }
  @Override
  public void stop(){
    mTopMotor.setVoltage(0);
    mBottomMotor.setVoltage(0);
  }
}
