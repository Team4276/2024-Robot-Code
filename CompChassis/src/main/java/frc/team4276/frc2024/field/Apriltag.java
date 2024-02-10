package frc.team4276.frc2024.field;

import frc.team254.lib.geometry.Pose2d;

public class Apriltag {
    private int id;
    private Pose2d tagInField;
    private double height;
    
    public Apriltag(int id, Pose2d tagInField, double height){
        this.id = id;
        this.tagInField = tagInField;
        this.height = height;
    }

    public int getId(){
        return id;
    }

    public Pose2d getTagInField(){
        return tagInField;
    }

    public double getHeight(){
        return height;
    }
    
    
}
