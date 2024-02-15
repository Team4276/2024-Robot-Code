package frc.team4276.lib.drivers;

// Omit acceleration b/c I'm lazy :p
public class FourBarFeedForward {
    private double kV; // V / Radian
    private double kS;
    private double kTotalStallTorque;
    private double kEfficiency;

    public double kTopLength; // Metres
    public double kBottomLength; // Metres
    public double kArmLength; // Metres; powered directly by motor
    public double kSupportLength; // Metres; not directly powered by motor
    public double kTopWeight; // Kg
    public double kArmWeight; // Kg
    public double kTopCOMToMotorSupport; // Metres
    public double kRotationToArmCOM;

    //TODO: add units to everything
    //TODO: add comments to everything
    //TODO: check math
    
    /**
     * Look at definition for specifics
     */
    public static class FourBarFeedForwardConstants {
        public double kS; // Find experimentally
        public double kMotorFreeSpeed; // RPM
        public double kGearRatio; // Rotations of motor per rotation of structure
        public double kStallTorque; 
        public double kEfficiency; // Value between 0.0 and 1.0
        public int kMotorAmnt;

        public double kTopLength; // Metres
        public double kBottomLength; // Metres
        public double kArmLength; // Metres; powered directly by motor
        public double kSupportLength; // Metres; not directly powered by motor
        public double kTopWeight; // Kg
        public double kArmWeight; // Kg
        public double kTopCOMToMotorSupport; // Metres
        public double kRotationToArmCOM; // Metres
    }

    public FourBarFeedForward(FourBarFeedForwardConstants constants){
        this.kS = constants.kS;
        this.kV = (12 / (constants.kMotorFreeSpeed / constants.kGearRatio)) * 60 / (2 * Math.PI); 
        this.kTotalStallTorque = constants.kStallTorque * constants.kMotorAmnt * constants.kGearRatio;
        this.kEfficiency = constants.kEfficiency;

        this.kTopLength = constants.kTopLength;
        this.kBottomLength = constants.kBottomLength;
        this.kArmLength = constants.kArmLength;
        this.kSupportLength = constants.kSupportLength;
        this.kTopWeight = constants.kTopWeight;
        this.kArmWeight = constants.kArmWeight;
        this.kTopCOMToMotorSupport = constants.kTopCOMToMotorSupport;
        this.kRotationToArmCOM = constants.kRotationToArmCOM;
    }

    /**
     * @param des_position radians; x+ is in the forward facing direction of the robot
     * @param des_velocity radians/s;
     * @return Volts
     */
    public double calculate(double des_position, double des_velocity){
        // Velocity Gain
        double vel = kV * des_velocity;

        // Gravity Gain
        double gravity = getkG(des_position) * Math.abs(Math.cos(des_position));

        double output = Math.signum(des_velocity) * kS + vel + gravity;

        // Clamp to 12 Volts
        return Math.max(Math.min(output, 12), -12);
    }

    private double getkG(double des_position){
        double complement = Math.PI - des_position;

        double t1 = findSideLength(kBottomLength, kArmLength, complement);

        double phi1 = kArmLength * Math.sin(complement) / t1 ;

        double ta1 = findAngle(kTopLength, kSupportLength, t1);

        double phi2 = kTopLength * Math.sin(ta1) / t1;

        double theta2 = phi1 + phi2;
 
        double Fn1 = ((kTopCOMToMotorSupport * Math.cos(des_position)) 
            + (kTopLength - kTopCOMToMotorSupport) * Math.cos(theta2)) 
            / ((kTopLength - kTopCOMToMotorSupport) * Math.cos(theta2) * 9.81 * kTopWeight);

        double Fn2 = (9.81 * kTopWeight) - Fn1;

        double Fg1 = applyNormal(Fn1, des_position);

        double Fg2 = applyNormal(Fn2, theta2);

        double endNewtons = Fg1 + Fg2;

        double COMDistance = ((kArmLength * endNewtons) + (kRotationToArmCOM * (kArmWeight * 9.81))) / (endNewtons + (kArmWeight * 9.81));

        return 12 * COMDistance * kEfficiency * endNewtons / kTotalStallTorque;
    }

    /** Function to find the length of a side in a triangle using the Law of Cosines
     * @param angleBetween radians
     */
    private double findSideLength(double side1, double side2, double angleBetween) {
        return Math.sqrt(Math.pow(side1, 2) + Math.pow(side2, 2) - 2 * side1 * side2 * Math.cos(angleBetween));
    }

    /** Function to find the angle opposite to the given side in a triangle using the Law of Cosines
     * @param side1
     * @param side2
     * @param side3 opposite side
     */ 
    private double findAngle(double side1, double side2, double side3) {        
        return Math.acos((Math.pow(side1, 2) + Math.pow(side2, 2) - Math.pow(side3, 2)) / (2 * side1 * side2));
    }

    private double applyNormal(double magnitude, double theta){
        return magnitude * Math.pow(Math.cos(theta), 2);
    }

}
