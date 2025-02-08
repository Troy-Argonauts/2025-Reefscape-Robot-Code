// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public interface Elevator {
    int LEFT_MOTOR_ID = 1;
    int RIGHT_MOTOR_ID = 2;
    int TOP_LIMIT_SWITCH_SLOT = 0;
    int BOTTOM_LIMIT_SWITCH_SLOT = 1;

    double P = 0;
    double I = 0;
    double D = 0;
    double V = 0;
    double G = 0;
  }
}

