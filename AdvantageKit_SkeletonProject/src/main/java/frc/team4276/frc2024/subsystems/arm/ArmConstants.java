package frc.team4276.frc2024.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.team4276.frc2024.Ports;
import frc.team4276.lib.feedforwards.FourbarFeedForward;

public class ArmConstants {
    public static final int kMasterId = Ports.ARM_MASTER;
    public static final int kFollowerId = Ports.ARM_FOLLOWER;
    
    public static final FourbarFeedForward.FourbarFeedForwardConstants kFeedForwardConstants = new FourbarFeedForward.FourbarFeedForwardConstants();
    static {
        kFeedForwardConstants.kS = 0.14;

        kFeedForwardConstants.kMotorFreeSpeedRpm = 5676.0;
        kFeedForwardConstants.kGearRatio = 185.712;
        kFeedForwardConstants.kStallTorque = 3.28;
        kFeedForwardConstants.kMotorAmnt = 2;
        kFeedForwardConstants.kEfficiency = 0.1;

        kFeedForwardConstants.kBottomLength = Units.inchesToMeters(8.001578);
        kFeedForwardConstants.kMotorLegLength = Units.inchesToMeters(11.000000);
        kFeedForwardConstants.kTopLength = Units.inchesToMeters(12.000808);
        kFeedForwardConstants.kSupportLegLength = Units.inchesToMeters(9.750);

        kFeedForwardConstants.kMotorLegMass = Units.lbsToKilograms(0.86);
        kFeedForwardConstants.kTopMass = Units.lbsToKilograms(28.0);
        kFeedForwardConstants.kSupportLegMass = Units.lbsToKilograms(0.494);

        kFeedForwardConstants.kMotorToCom = new Translation2d(
                Units.inchesToMeters(5.5),
                Units.inchesToMeters(0));
        kFeedForwardConstants.kMotorLegToTopCom = new Translation2d(
                Units.inchesToMeters(4.799278),
                Units.inchesToMeters(1.121730));
        kFeedForwardConstants.kSupportToCom = new Translation2d(
                Units.inchesToMeters(5.076911),
                Units.inchesToMeters(1.096583));
    }

}
