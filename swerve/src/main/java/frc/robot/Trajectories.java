package frc.robot;


import java.util.ArrayList;


import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.LinkedHashMap;

public class Trajectories {
   public static ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

   static boolean test = false;

   public Trajectories(){
      path_init();
   }

   public static void path_init(){
      File paths_folder = new File("/home/lvuser/deploy/paths");
      String name = paths_folder.getName();

      try {
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(name);
         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

         trajectories.add(trajectory);

      } catch (IOException ex) {
         DriverStation.reportError("Unable to open trajectory: " + name, ex.getStackTrace());
      }


      String userDirectory = null;
      userDirectory = System.getProperty("paths.dir");
      System.out.println(userDirectory);

      
      File paths = new File("/Users/thoma/Documents/2023-Robot-Code/swerve/src/main/deploy/paths/*");

      File[] pathslist = paths.listFiles();

      if (paths.listFiles() == null){
         test = true;
         

      } else {
         for (File path : paths.listFiles()){
            test = true;
            


         }
      }

      

      /*for (File path : paths.listFiles()){
         String trajectoryJSON = path.getName();

         try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            trajectories.add(trajectory);

         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         }

      } */
   }

   public static LinkedHashMap<String, Trajectory> loadPaths(){
      LinkedHashMap<String, Trajectory> paths = new LinkedHashMap<String, Trajectory>();

      paths.put("Test", getTraj("Test"));
      paths.put("Test2", getTraj("Test2"));

      return paths;
   }
   

   private static Trajectory getTraj(String name){
      try{ 
         Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
            .resolve("/home/lvuser/deploy/paths/" + name + ".wpilib.json");
         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

         return trajectory;

      }catch(IOException e){
          System.out.println("!!!!!!!!!! IO Exception on Reading Traj !!!!!!!!!!");
          return null;
      }
   }

   public static Trajectory getPath(String path_key){
      LinkedHashMap<String, Trajectory> paths = loadPaths();
      Trajectory path = paths.get(path_key);

      return path;
   }
}
