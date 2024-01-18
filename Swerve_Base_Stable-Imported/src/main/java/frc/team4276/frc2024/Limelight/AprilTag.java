package frc.team4276.frc2024.Limelight;

import frc.team254.lib.geometry.Pose2d;

public class AprilTag {
    private int id;
    private Pose2d tagInField;
    
    public AprilTag(int id, Pose2d tagInField){
        this.id = id;
        this.tagInField = tagInField;
    }

    public int getId(){
        return id;
    }

    public Pose2d getTagInField(){
        return tagInField;
    }
    
    
}
