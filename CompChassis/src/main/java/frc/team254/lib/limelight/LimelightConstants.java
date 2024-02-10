package frc.team254.lib.limelight;

import frc.team254.lib.limelight.undistort.UndistortMap;
import frc.team254.lib.limelight.undistort.UndistortConstants;
import frc.team254.lib.geometry.Rotation2d;

public class LimelightConstants {
    private String id;
    private String name;
    private double height;
    private Rotation2d yawOffset;
    private Rotation2d horizontalPlaneToLens;

    private UndistortConstants undistortConstants;

    private UndistortMap undistortMap;



    public LimelightConstants(String id, String name, double height, Rotation2d yawOffset, Rotation2d horizontalPlaneToLens,  UndistortConstants undistortConstants, UndistortMap undistortMap) {
        this.id = id;
        this.name = name;
        this.height = height;
        this.yawOffset = yawOffset;
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.undistortConstants = undistortConstants;
        this.undistortMap = undistortMap;
    }

    public String getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public double getHeight() {
        return height;
    }

    public Rotation2d getYawOffset() {
        return yawOffset;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public UndistortConstants getUndistortConstants() {
        return undistortConstants;
    }

    public UndistortMap getUndistortMap() { return this.undistortMap; };

}