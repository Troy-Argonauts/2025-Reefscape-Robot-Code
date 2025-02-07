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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public interface Manipulator {

    public static final double P = 0;
    public static final double I = 0.;
    public static final double D = 0;
    public static final double V = 0;

    public static final int TOP_MOTOR_CAN_ID = 0;
    public static final int BOTTOM_MOTOR_CAN_ID = 0;
    public static final int LATERATOR_MOTOR_CAN_ID = 0;
    public static final int LATERATOR_LIMIT_SWITCH = 0;
    public static final int FUNNEL_BEAM_BREAK = 0;
    public static final int MANIPULATOR_A_BEAM_BREAK = 0;
    public static final int MANIPULATOR_B_BEAM_BREAK = 0;

    public static final double MAX_LATERATOR_POSITION = 0.0;
  } 
} //Evan is not cool. Ved is cool.

