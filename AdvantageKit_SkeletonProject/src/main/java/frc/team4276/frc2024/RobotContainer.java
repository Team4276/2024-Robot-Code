package frc.team4276.frc2024;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team4276.frc2024.subsystems.flywheels.FlywheelIOSpark;
import frc.team4276.frc2024.subsystems.flywheels.Flywheels;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private Flywheels flywheels;

  RobotContainer(){
    switch (Constants.currentMode){
        case REAL:
            flywheels = new Flywheels(new FlywheelIOSpark());
        case REPLAY:
            break;
        case SIM:
            break;
        default:
            break;
    }

    bindTriggers();
  }
  
  private void bindShooting(){

  }
  private void bindDrive(){
    
  }
  private void bindIntake(){
    
  }
  private void bindFlywheelSysID(){
    //hold each button until the motor stops moving. Press the buttons in the order they are set here
    driver.a().whileTrue(flywheels.sysIdQuasistatic(Direction.kForward));
    driver.b().whileTrue(flywheels.sysIdQuasistatic(Direction.kReverse));
    driver.rightBumper().whileTrue(flywheels.sysIdDynamic(Direction.kForward));
    driver.leftBumper().whileTrue(flywheels.sysIdDynamic(Direction.kReverse));
  }

  private void bindTriggers(){
    if(!Constants.SysIdMode){
        bindShooting();
        bindDrive();
        bindIntake();
    }else{
        bindFlywheelSysID();
    }
  }
}
