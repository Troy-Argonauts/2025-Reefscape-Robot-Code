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
    public interface Climber {
  
        int LEFT_MOTOR_ID = 1;
        int RIGHT_MOTOR_ID = 2;
        int ALIGN_MOTOR_ID = 3;
        int TONGUE_MOTOR_ID = 4;

        int ARM_LIMIT_SWITCH = 0;
        int TONGUE_LIMIT_SWITCH = 0;
        int ALIGN_LIMIT_SWITCH = 0;

        public static final double MAX_ARM_POSITION = 0.0;
        public static final double MAX_Tongue_POSITION = 0.0;
        public static final double MAX_Align_POSITION = 0.0;

    }
}
