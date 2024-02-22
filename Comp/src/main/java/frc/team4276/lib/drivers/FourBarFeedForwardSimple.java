package frc.team4276.lib.drivers;

import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.lib.drivers.EmptySubsystem;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.fasterxml.jackson.core.io.schubfach.DoubleToDecimal;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2024.subsystems.Superstructure;

public class FourBarFeedForwardSimple {
    public static Superstructure superStruct = Superstructure.getInstance();
    static CANSparkMax mController; 
    static AbsoluteEncoder mEncoder;
    static double fallOffRange;

    public static void motorSetpoint(double position){
       
        mController = superStruct.mFourBarSubsystem.mMaster;
        mEncoder = mController.getAbsoluteEncoder(Type.kDutyCycle);
        mEncoder.setZeroOffset(SuperstructureConstants.kFourBarConstants.kOffset);
        boolean arrived = true;
        SmartDashboard.putNumber("encoder pose", mEncoder.getPosition());

        double pos = mEncoder.getPosition();

        while(!arrived){
             pos = mEncoder.getPosition();
            
        } 
    }
}
