// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2024;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2024.subsystems.vision.VisionIOPhoton.PhotonDeviceConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double kLooperDt = 0.02;

    public static final boolean isTuning = false;

    public static final class RobotStateConstants {
        public static final boolean kVisionResetsHeading = false;

        public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1));
        public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.03, 1),
                Math.pow(0.03, 1));
    }

    public static final class VisionConstants {
        public static final PhotonDeviceConstants kFrontCameraConstants = new PhotonDeviceConstants();

        static {
            kFrontCameraConstants.kCameraName = "Front Camera";
            kFrontCameraConstants.kCameraNameId = "Arducam_OV9281_USB_Camera";
            kFrontCameraConstants.kRobotToCamera = new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(9.591351),
                            Units.inchesToMeters(7.500000) * -1.0,
                            Units.inchesToMeters(5.575688)),
                    new Rotation3d(0.0, Math.toRadians(20) * -1.0, 0.0));
        }

        public static final PhotonDeviceConstants kBackCameraConstants = new PhotonDeviceConstants();

        static {
            kFrontCameraConstants.kCameraName = "Back Camera";
            kBackCameraConstants.kCameraNameId = "PI_CAM_3";
            kBackCameraConstants.kRobotToCamera = new Transform3d();
        }
    }
}
