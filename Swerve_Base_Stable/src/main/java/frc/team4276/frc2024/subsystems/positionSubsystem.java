package frc.team4276.frc2024.subsystems;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team1678.lib.requests.Request;
import edu.wpi.first.networktables.*;


import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.net.http.HttpClient;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;

public class positionSubsystem extends Subsystem {

  
  public static double[] findVisionPose() {

    final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
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

  // Fortnite Battle Pass
}

// 1 make api call
// 2 store result in a string (optional error handling, check if throw exception)