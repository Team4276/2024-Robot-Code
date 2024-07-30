package frc.team4276.lib.rev;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;

//TODO: implement
public class VIKAbsoluteEncoder implements VIKEncoder {
    private SparkAbsoluteEncoder mEncoder;

    public VIKAbsoluteEncoder(CANSparkBase sparkMax){
        mEncoder = sparkMax.getAbsoluteEncoder();
    }
    
}
