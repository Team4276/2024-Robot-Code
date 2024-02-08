package frc.team4276.frc2024.field;

import java.util.Optional;

import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.geometry.Rotation2d;

public class Area {
    private Translation2d[] corners;

    /**
     * Does not Accept Concave shapes.
     * List corners in clockwise order.s
     * Begin with any point that equals the lowest X value.
     * @param corners
     */
    public Area(Translation2d[] corners){
        this.corners = corners;
        
    }

    public boolean inArea(Translation2d point){
        int over = -1;

        for(int i = 0; i < corners.length; i++){
            Translation2d next_ = corners[i < corners.length - 1 ? i + 1 : 0];
            Translation2d curr_ = corners[i];

            if((point.x() >= next_.x() && point.x() <= curr_.x()) 
                || (point.x() <= next_.x() && point.x() >= curr_.x())){
                
                if(over == -1){
                    over = overPoint(curr_, next_, point);
                }
                
                if(over != overPoint(curr_, next_, point)){
                    break;
                }
            }
            
        }

        return false;
    }


    private int overPoint(Translation2d p1, Translation2d p2, Translation2d point){
        Translation2d to_next_point = p2.translateBy(p1.inverse());

        double unit_cos_to_next_point = to_next_point.x() / to_next_point.norm();
        double unit_cos_to_point = to_next_point.x() / point.norm();

        return unit_cos_to_next_point > unit_cos_to_point ? 0 : 1;
    }
}
