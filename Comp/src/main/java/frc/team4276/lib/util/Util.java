package frc.team4276.lib.util;

public class Util {
    // Utility class; do not instantiate
    private Util(){}

    /**
     * @param side1
     * @param side2
     * @param opposite_angle radians
     * @return side length opposite of given angle
     */
    public static double LoCLength(double side1, double side2, double opposite_angle) {
        return Math.sqrt(Math.pow(side1, 2) + Math.pow(side2, 2) - 2 * side1 * side2 * Math.cos(opposite_angle));
    }

    /**
     * @param side1
     * @param side2
     * @param opposite_side opposite side
     * @return radians
     */ 
    public static double LoCAngle(double side1, double side2, double opposite_side) {        
        return Math.acos((Math.pow(side1, 2) + Math.pow(side2, 2) - Math.pow(opposite_side, 2)) / (2 * side1 * side2));
    }

    /**
     * @param side1 
     * @param angle1 radians
     * @param opposite_angle radians 
     * @return
     */
    public static double LoSLength(double side1, double angle1, double opposite_angle) {
        return side1 * Math.sin(opposite_angle) / Math.sin(angle1);
    }

    //TODO: check if this will ever return NaN
    /**
     * @param side1
     * @param angle1 radians
     * @param opposite_side
     * @return radians
     */
    public static double LoSAngle(double side1, double angle1, double opposite_side) {
        double output = Math.asin(Math.sin(angle1) * opposite_side / side1);
        return output == Double.NaN ? null : Math.asin(Math.sin(angle1) * opposite_side / side1);
    }
}
