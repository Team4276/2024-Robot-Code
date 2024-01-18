package frc.team4276.frc2024.subsystems;

import edu.wpi.first.networktables.*;

public class positionSubsystem extends Subsystem {

  
  public static double[] findVisionPose() {
    NetworkTableInstance defaultInst = NetworkTableInstance.getDefault();
    defaultInst.setServerTeam(4276);
    //limelight docs for different entries https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    double[] pose = defaultInst.getTable("limelight").getEntry("botpose").getDoubleArray(new double[0]);
    return pose;
    /* 
    try {
    URL url = new URL("");
    HttpURLConnection con = (HttpURLConnection) url.openConnection();
    con.setRequestMethod("GET");

    int status = con.getResponseCode();

    if (status != 200) {return null;}

    BufferedReader in = new BufferedReader(
    new InputStreamReader(con.getInputStream()));
    String inputLine;
    StringBuffer content = new StringBuffer();
    while ((inputLine = in.readLine()) != null) {
      content.append(inputLine);
    } 
    in.close();
    con.disconnect();
  } catch (Exception e){
    DriverStation.reportError("", false);
  }
    return null;
  */
}

}
