package frc.team4276.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {
  /**
   * @param side1
   * @param side2
   * @param opposite_angle radians
   * @return side length opposite of given angle
   */
  public static double LoCLength(double side1, double side2, double opposite_angle) {
    return Math.sqrt(
        Math.pow(side1, 2) + Math.pow(side2, 2) - 2 * side1 * side2 * Math.cos(opposite_angle));
  }

  /**
   * @param side1
   * @param side2
   * @param opposite_side opposite side
   * @return radians
   */
  public static double LoCAngle(double side1, double side2, double opposite_side) {
    return Math.acos(
        (Math.pow(side1, 2) + Math.pow(side2, 2) - Math.pow(opposite_side, 2))
            / (2 * side1 * side2));
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

  /**
   * @param side1
   * @param angle1 radians
   * @param opposite_side
   * @return radians
   */
  public static double LoSAngle(double side1, double angle1, double opposite_side) {
    return Math.asin(Math.sin(angle1) * opposite_side / side1);
  }

  public static Rotation2d Rescope360to180(Rotation2d rot) {
    if (rot.getRadians() > Math.PI) {
      return rot.rotateBy(Rotation2d.fromRadians(-2 * Math.PI));
    }

    return rot;
  }

  /**
   * @param angle
   * @param max assume min is 0
   * @return
   */
  public static double rescopeAngle(double angle, double max) {
    while (angle < 0.0) {
      angle += max;
    }

    angle %= max;

    return angle;
  }
}
