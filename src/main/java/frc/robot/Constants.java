// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;


import static edu.wpi.first.units.Units.Meters;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class VisionConstants {
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.6;
    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(2.0);
    public static final String[] CAMERA_NAMES = new String[] { "FrontLeft", "FrontRight", "BackLeft"};
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(new Translation3d(0.3810, 0.2540, 0.2286), new Rotation3d(0, .349, 0)),
        new Transform3d(new Translation3d(0.3810, -0.2540, 0.2286), new Rotation3d(0, .349, 3.4907)),
        new Transform3d(new Translation3d(-0.4064, 0.0000, 0.2921), new Rotation3d(0, .349, 2.7925)),};


    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(23, 6, 10);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.7, 0.7, 2);

    public static final Distance FIELD_LENGTH = Meters.of(17.548);
    public static final Distance FIELD_WIDTH = Meters.of(8.052);


  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }
  public static final class IntakeCoralConstants{
    public static final int kIntakeCoral = 51;
  }

  public static final class IntakeAlgaeConstants{
    public static final int kIntakeAlgae = 52;
  }

  public static final class WristConstants{
    public static final int kWrist = 50;
  }

  public static final class ClimbConstants{
    public static final int kClimb = 53;
  }

  public static final class LEDs{
    public static final int purple_Red = 148;
    public static final int purple_Green = 0;
    public static final int purple_Blue = 211;
    public static final int purple_Hue = 282;
    public static final int purple_Sat = 100;
    public static final int purple_Val = 83;

    public static final int yellow_Red = 255;
    public static final int yellow_Green = 90;
    public static final int yellow_Blue = 0;
    public static final int yellow_Hue = 21;
    public static final int yellow_Sat = 100;
    public static final int yellow_Val = 100;

    public static final int orange_Red = 251;
    public static final int orange_Green = 51;
    public static final int orange_Blue = 0;
    public static final int orange_Hue = 12;
    public static final int orange_Sat = 100;
    public static final int orange_Val = 98;

    public static final int red_Hue = 0;
    public static final int red_Sat = 100;
    public static final int red_Val = 100;

    public static final int blue_Hue = 240;
    public static final int blue_Sat = 100;
    public static final int blue_Val = 100;

    public static final int green_Hue = 240;
    public static final int green_Sat = 100;
    public static final int green_Val = 100;

    public static final int Red = 255;
    public static final int Green = 255;
    public static final int Blue = 255;
    public static final int white_Hue = 0;
    public static final int white_Sat = 0;
    public static final int white_Val = 100;

    public static final int aqua_Red = 72;
    public static final int aqua_Green = 255;
    public static final int aqua_Blue = 200;
    public static final int aqua_Hue = 162;
    public static final int aqua_Sat = 72;
    public static final int aqua_Val = 100;
  }

}
