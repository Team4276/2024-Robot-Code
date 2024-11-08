package frc.team4276.frc2024.field;

import edu.wpi.first.math.geometry.Pose2d;

public class Apriltag {
    private int id;
    private Pose2d tagInField;
    private double height;
    private boolean isInit;

    public Apriltag(int id, Pose2d tagInField, double height) {
        this.id = id;
        this.tagInField = tagInField;
        this.height = height;
        isInit = true;
    }

    public int getId() {
        if (isInit) {
            return id;
        }

        return -2;
    }

    public Pose2d getTagInField() {
        if (isInit) {
            return tagInField;
        }

        return new Pose2d();
    }

    public double getHeight() {
        if (isInit) {
            return height;
        }

        return 0.0;
    }
}
