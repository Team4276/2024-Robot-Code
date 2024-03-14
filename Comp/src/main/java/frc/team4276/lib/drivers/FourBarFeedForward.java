package frc.team4276.lib.drivers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.lib.util.Util;

import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.geometry.Rotation2d;

public class FourBarFeedForward {
    // Constants
    private double kS; // Volts
    private final double kV; // Volts * s / rad

    private final double kGearRatio;
    private final double kStallTorque;
    private final int kMotorAmnt;
    private double kEfficiency;

    /*
     * NOTE:
     * Angles are in standard position.
     * Motor Leg is on the left and support leg is on the right.
     */

    private final double kBottomLength;
    private final double kMotorLegLength;
    private final double kTopLength;
    private final double kSupportLegLength;

    // Kg
    private final double kMotorLegMass;
    private final double kTopMass;
    private final double kSupportLegMass;

    /*
     * Lay the line between the rotating points on the X AXIS
     * (if y value = 0 then Com is on the line between the two points of rotation)
     * The STATIC POINT should be on the origin.
     */
    private final Translation2d kMotorToCom;
    private final Translation2d kMotorLegToTopCom;
    private final Translation2d kSupportToCom;

    // Dynamics

    // Angles on the INSIDE of the quadrilateral (always positive)
    private double motor_inside_angle_;
    private double motor_leg_to_top_inside_angle_;
    private double support_inside_angle_;
    private double support_leg_to_top_inside_angle_;

    // Angles relevant for us
    private double bottom_to_motor_leg_radians_;
    private double bottom_to_top_radians_;
    private double bottom_to_support_leg_radians_; // Supplement of the inside angle
    /*
     * for bottom_to_support_leg_radians_ imagine the bottom support extends
     * to the RIGHT of the support leg and measure from there
     */

    private Translation2d motor_to_leg_com_;
    private Translation2d support_to_leg_com_;
    private Translation2d motor_leg_to_top_com_;

    // private Translation2d total_motor_to_com_;

    public static class FourBarFeedForwardConstants {
        public double kS;

        public double kMotorFreeSpeedRpm;
        public double kGearRatio;
        public double kStallTorque;
        public int kMotorAmnt;
        public double kEfficiency;

        public double kBottomLength;
        public double kMotorLegLength;
        public double kTopLength;
        public double kSupportLegLength;

        public double kMotorLegMass;
        public double kTopMass;
        public double kSupportLegMass;

        public Translation2d kMotorToCom;
        public Translation2d kMotorLegToTopCom;
        public Translation2d kSupportToCom;
    }

    public FourBarFeedForward(FourBarFeedForwardConstants constants) {
        this.kS = constants.kS;
        // nominal voltage / max speed RPM converted to radians per second
        this.kV = (12 / (constants.kMotorFreeSpeedRpm / constants.kGearRatio)) * 60 / (2 * Math.PI);

        this.kGearRatio = constants.kGearRatio;
        this.kStallTorque = constants.kStallTorque;
        this.kMotorAmnt = constants.kMotorAmnt;
        this.kEfficiency = constants.kEfficiency;

        this.kBottomLength = constants.kBottomLength;
        this.kMotorLegLength = constants.kMotorLegLength;
        this.kTopLength = constants.kTopLength;
        this.kSupportLegLength = constants.kSupportLegLength;

        this.kMotorLegMass = constants.kMotorLegMass;
        this.kTopMass = constants.kTopMass;
        this.kSupportLegMass = constants.kSupportLegMass;

        this.kMotorToCom = constants.kMotorToCom;
        this.kMotorLegToTopCom = constants.kMotorLegToTopCom;
        this.kSupportToCom = constants.kSupportToCom;
    }

    public void setkS(double static_voltage){
        kS = static_voltage;
    }

    public void setEfficiency(double efficiency){
        kEfficiency = efficiency;
    }

    /**
     * @param position_setpoint radians
     * @param velocity_setpoint radians / s
     * @return
     */
    public double calculate(double position_setpoint, double velocity_setpoint) {
        updateInsideAngles(position_setpoint);
        updateRelevantAngles();
        updateComs();

        double gravity_voltage = calcGravityVoltage(position_setpoint);
        double velocity_voltage = kV * velocity_setpoint;

        SmartDashboard.putNumber("Fourbar Gravity Voltage", gravity_voltage);
        SmartDashboard.putNumber("Fourbar Velocity Voltage", velocity_voltage);
        return gravity_voltage + (kS * Math.signum(velocity_setpoint)) + velocity_voltage;
    }

    // private double calcAccelerationVoltage(){
        
    //     // total COM distance * total mass * efficiency * nominal voltage
    //     return (0) * (kMotorLegMass + kTopMass + kSupportLegMass) * kEfficiency * 12 / (kStallTorque * kMotorAmnt * kGearRatio);
    // }

    private double calcGravityVoltage(double position_setpoint) {
        // desired torque * nominal voltage / max torque
        return calcGravityTorque() * kEfficiency * 12 / (kStallTorque * kMotorAmnt * kGearRatio);
    }

    private void updateInsideAngles(double position) {
        motor_inside_angle_ = position;

        double top_left_to_bottom_right = Util.LoCLength(kBottomLength, kMotorLegLength, motor_inside_angle_);

        support_leg_to_top_inside_angle_ = Util.LoCAngle(kTopLength, kSupportLegLength, top_left_to_bottom_right);

        motor_leg_to_top_inside_angle_ = Util.LoSAngle(top_left_to_bottom_right, motor_inside_angle_, kBottomLength)
                + Util.LoSAngle(top_left_to_bottom_right, support_leg_to_top_inside_angle_, kSupportLegLength);

        support_inside_angle_ = Util.LoSAngle(top_left_to_bottom_right, motor_inside_angle_, kMotorLegLength)
                + Util.LoSAngle(top_left_to_bottom_right, support_leg_to_top_inside_angle_, kTopLength);

    }

    private void updateRelevantAngles() {
        bottom_to_motor_leg_radians_ = motor_inside_angle_;

        bottom_to_support_leg_radians_ = Math.PI - support_inside_angle_;

        bottom_to_top_radians_ = motor_leg_to_top_inside_angle_ + bottom_to_motor_leg_radians_ - Math.PI;
    }

    private void updateComs() {
        motor_to_leg_com_ = kMotorToCom.rotateBy(Rotation2d.fromRadians(
                bottom_to_motor_leg_radians_));

        motor_leg_to_top_com_ = kMotorLegToTopCom.rotateBy(Rotation2d.fromRadians(bottom_to_top_radians_));

        support_to_leg_com_ = kSupportToCom.rotateBy(Rotation2d.fromRadians(bottom_to_support_leg_radians_));

        // total_motor_to_com_ = new Translation2d();

    }

    private double calcGravityTorque() {
        double transfered_force = calcSupportLegTorque() / (kSupportLegLength * Math.sin(support_leg_to_top_inside_angle_ % (Math.PI / 2)));

        double transfered_torque = transfered_force * Math.sin((motor_inside_angle_ - bottom_to_top_radians_) % (Math.PI / 2)) * kMotorLegLength;

        return calcMotorLegTorque() + transfered_torque;
    }

    private double calcMotorLegTorque() {
        return (kMotorLegMass * 9.81 * motor_to_leg_com_.x()) + 
            (calcShooterToMotorLeg() * Math.cos(bottom_to_motor_leg_radians_));
    }

    private double calcSupportLegTorque() {
        return (kSupportLegMass * 9.81 * support_to_leg_com_.x()) + 
            (calcShooterToSupportLeg() * Math.cos(bottom_to_support_leg_radians_));
    }

    /**
     * Returns force of top on motor leg
     */
    private double calcShooterToMotorLeg() {
        double gravity_torque = kTopMass * 9.81 * ((kTopLength * Math.cos(bottom_to_top_radians_)) - motor_leg_to_top_com_.x());

        return gravity_torque / (kTopLength * Math.cos(bottom_to_top_radians_));
    }

    /**
     * Returns force of top on support leg
     */
    private double calcShooterToSupportLeg() {
        double gravity_torque = kTopMass * 9.81 * motor_leg_to_top_com_.x();
        
        return gravity_torque / (kTopLength * Math.cos(bottom_to_top_radians_));
    }

}
