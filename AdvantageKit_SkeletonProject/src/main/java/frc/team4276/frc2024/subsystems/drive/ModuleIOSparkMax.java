package frc.team4276.frc2024.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.team4276.lib.rev.SparkMaxFactory;
import frc.team4276.lib.rev.VIKSparkMax;

public class ModuleIOSparkMax implements ModuleIO {
  public static class ModuleConfig {
    public String kName = "ERROR_NO_NAME";
    public int kDriveId = -1;
    public int kTurnId = -1;
    public double kOffset = 0;
  }

  private VIKSparkMax driveMotor;
  private VIKSparkMax turnMotor;

  private AbsoluteEncoder turnAbsoluteEncoder;
  private double absoluteEncoderOffset;

  private RelativeEncoder driveEncoder;

  public ModuleIOSparkMax(ModuleConfig config) {
    // Init motor & encoder objects
    driveMotor = SparkMaxFactory.createDefault(config.kDriveId);
    turnMotor = SparkMaxFactory.createDefault(config.kTurnId);
    turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
    absoluteEncoderOffset = config.kOffset;
    driveEncoder = driveMotor.getEncoder();

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();
    driveMotor.setCANTimeout(250);
    turnMotor.setCANTimeout(250);

    for (int i = 0; i < 30; i++) {
      turnMotor.setInverted(true);
      driveMotor.setSmartCurrentLimit(40);
      turnMotor.setSmartCurrentLimit(30);
      driveMotor.enableVoltageCompensation(12.0);
      turnMotor.enableVoltageCompensation(12.0);

      driveEncoder.setPosition(0.0);
      driveEncoder.setMeasurementPeriod(10);
      driveEncoder.setAverageDepth(2);

      // driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 /
      // odometryFrequency));
      // turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, (int) (1000.0 /
      // odometryFrequency));
    }

    driveMotor.burnFlash();
    turnMotor.burnFlash();

    driveMotor.setCANTimeout(0);
    turnMotor.setCANTimeout(0);

    // absoluteEncoderValue = () -> Rotation2d.fromRotations(
    //         turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V())
    //         .minus(absoluteEncoderOffset);

    // drivePositionQueue =
    // SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    // turnPositionQueue = SparkMaxOdometryThread.getInstance()
    //         .registerSignal(() -> absoluteEncoderValue.get().getRadians());

    // // Init Controllers
    // driveController = new PIDController(moduleConstants.drivekP(), 0.0,
    // moduleConstants.drivekD());
    // turnController = new PIDController(moduleConstants.turnkP(), 0.0, moduleConstants.turnkD());
    // turnController.enableContinuousInput(-Math.PI, Math.PI);
  }
}
