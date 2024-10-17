package frc.team4276.frc2024.subsystems.drive;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.Constants;

public class Drive extends SubsystemBase {
    public enum DriveMode {
        /** Driving with input from driver joysticks. (Default) */
        TELEOP,

        /** Driving based on a trajectory. */
        TRAJECTORY,

        /** Driving to a location on the field automatically. */
        AUTO_ALIGN
    }

    private DriveMode mode = DriveMode.TELEOP;
    
    public static class KinematicLimits {
        public double kMaxDriveVelocity = DriveConstants.kMaxVel; // m/s
        public double kMaxAccel = Double.MAX_VALUE; // m/s^2
        public double kMaxAngularVelocity = DriveConstants.kMaxAngularVel; // rad/s
        public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
        public String kName = "Default";
    }

    private KinematicLimits mKinematicLimits = DriveConstants.kUncappedLimits;

    private final GyroIO mGyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] mModules = new Module[4];

    private ChassisSpeeds setpoint = new ChassisSpeeds();
    private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        mGyroIO = gyroIO;
        mModules[0] = new Module(fl, 0);
        mModules[1] = new Module(fr, 1);
        mModules[2] = new Module(bl, 2);
        mModules[3] = new Module(br, 3);
    }

    @Override
    public void periodic() {
        mGyroIO.updateInputs(gyroInputs);

        for(Module module : mModules){
            module.updateInputs();
        }

        //TODO: fix all this bullshit
        Pose2d robot_pose_vel = new Pose2d(setpoint.vxMetersPerSecond * Constants.kLooperDt,
                setpoint.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(setpoint.omegaRadiansPerSecond * Constants.kLooperDt));

        var tv = new Pose2d().log(robot_pose_vel); //TODO: hacky but wtv it works; fix soon pls
        double scalar = 1.0 / Constants.kLooperDt;
        Twist2d twist_vel = new Twist2d(tv.dx * scalar, tv.dy * scalar, tv.dtheta * scalar);

        ChassisSpeeds wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);

        if (mode != DriveMode.TRAJECTORY) {
            // Limit rotational velocity
            wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
                    * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

            // Limit translational velocity
            double velocity_magnitude = Math.hypot(setpoint.vxMetersPerSecond,
                    setpoint.vyMetersPerSecond);
            if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
                wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude)
                        * mKinematicLimits.kMaxDriveVelocity;
                wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude)
                        * mKinematicLimits.kMaxDriveVelocity;
            }

            SwerveModuleState[] prev_module_states = moduleStates.clone(); // Get last setpoint to get
                                                                                    // differentials
            ChassisSpeeds prev_chassis_speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(prev_module_states);
            SwerveModuleState[] target_module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(wanted_speeds);

            //TODO: fix for better tolerances
            if (wanted_speeds.equals(new ChassisSpeeds())) {
                for (int i = 0; i < target_module_states.length; i++) {
                    target_module_states[i].speedMetersPerSecond = 0.0;
                    target_module_states[i].angle = prev_module_states[i].angle;
                }
            }

            double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
            double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
            double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

            double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
            double min_translational_scalar = 1.0;

            if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
                // Check X
                double x_norm = Math.abs(dx / max_velocity_step);
                min_translational_scalar = Math.min(min_translational_scalar, x_norm);

                // Check Y
                double y_norm = Math.abs(dy / max_velocity_step);
                min_translational_scalar = Math.min(min_translational_scalar, y_norm);

                min_translational_scalar *= max_velocity_step;
            }

            double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
            double min_omega_scalar = 1.0;

            if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
                double omega_norm = Math.abs(domega / max_omega_step);
                min_omega_scalar = Math.min(min_omega_scalar, omega_norm);

                min_omega_scalar *= max_omega_step;
            }
            
            wanted_speeds = new ChassisSpeeds(
                prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
                prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
                prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);
        }
        
        SmartDashboard.putNumber("Comp/Des X Speed: ", wanted_speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Y Speed: ", wanted_speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Comp/Des Rot Speed: ", wanted_speeds.omegaRadiansPerSecond);

        SwerveModuleState[] real_module_setpoints = DriveConstants.kDriveKinematics.toSwerveModuleStates(wanted_speeds);
        moduleStates = real_module_setpoints;

    }

    
    // public synchronized void teleopDrive(ChassisSpeeds speeds) {
    //     if (mControlState != DriveControlState.OPEN_LOOP && mControlState != DriveControlState.HEADING_CONTROL) {
    //         mControlState = DriveControlState.OPEN_LOOP;
    //     }

    //     if (mControlState == DriveControlState.HEADING_CONTROL) {
    //         if (Math.abs(speeds.omegaRadiansPerSecond) > 1.0) {
    //             mControlState = DriveControlState.OPEN_LOOP;
    //         } else {
    //             mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(
    //                     speeds.vxMetersPerSecond,
    //                     speeds.vyMetersPerSecond,
    //                     mHeadingController.update(mPeriodicIO.heading.getRadians(), mPeriodicIO.timestamp));
    //             return;
    //         }
    //     }

    //     mPeriodicIO.des_chassis_speeds = speeds;
    // }

    // public synchronized void updatePathFollowingSetpoint(ChassisSpeeds speeds) {
    //     if (mControlState != DriveControlState.PATH_FOLLOWING) {
    //         mControlState = DriveControlState.PATH_FOLLOWING;
    //     }

    //     mPeriodicIO.des_chassis_speeds = speeds;
    // }

    // public synchronized void updatePPPathFollowingSetpoint(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
    //     updatePathFollowingSetpoint(ChassisSpeeds.fromWPI(speeds));
    // }

    // public synchronized void setPathFollowingPath(PathPlannerPath path) {
    //     if (mControlState != DriveControlState.PATH_FOLLOWING) {
    //         mControlState = DriveControlState.PATH_FOLLOWING;
    //     }

    //     mMotionPlanner.setTrajectory(path, RobotState.getInstance().getLatestFieldToVehicle(), mPeriodicIO.meas_chassis_speeds, mPeriodicIO.timestamp);
    // }

    // public synchronized void feedTrackingSetpoint(Rotation2d angle) {
    //     mTrackingAngle = angle;
    // }

    // public synchronized void setHeadingSetpoint(Rotation2d angle) {
    //     if (mControlState != DriveControlState.HEADING_CONTROL) {
    //         mControlState = DriveControlState.HEADING_CONTROL;
    //     }

    //     if (mHeadingController.getTargetRad() != angle.getRadians()) {
    //         mHeadingController.setTarget(angle.getRadians());
    //     }
    // }

    public void runVelocity(ChassisSpeeds speeds){
        this.setpoint = speeds;
    }
}
