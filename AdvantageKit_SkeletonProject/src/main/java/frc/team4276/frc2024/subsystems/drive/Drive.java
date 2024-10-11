package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving based on a trajectory. */
    TRAJECTORY,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN
  }

  private final GyroIO mGyroIO;
  private final Module[] mModules = new Module[4];

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    mGyroIO = gyroIO;
    mModules[0] = new Module(fl, 0);
    mModules[1] = new Module(fr, 1);
    mModules[2] = new Module(bl, 2);
    mModules[3] = new Module(br, 3);
  }

  @Override
  public void periodic() {}
}
