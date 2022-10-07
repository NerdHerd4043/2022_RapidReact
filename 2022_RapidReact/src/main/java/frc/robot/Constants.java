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
    public static final class DriveConstants {
        public static final int frontLeftMotorID = 10;
        public static final int frontRightMotorID = 8;
        public static final int backLeftMotorID = 6;
        public static final int backRightMotorID = 3;

        public static final double highGearRamp = 0.2;
        public static final double lowGearRamp = 0.1;

        public static final int currentLimit = 40;

        public static final int shifterID = 1;

        public static boolean harvesterIsFront = true;

        public static final class Gears {
            public static final boolean highGear = true;
            public static final boolean lowGear = false;
            public static boolean isHighGear = true;
        }
    }

    public static final class RobotConstants {
        public static final int PCMD = 1;
        public static final double elevatorWaitTime = 1;
    }
    public static final class ElevatorConstants {
        public static final int beltMotorID = 15;
    }

    public static final class IntakeConstants {
        public static final int harvesterMotorID = 13;
        public static final int kickupMotorID = 11;
        public static final int miniHarvesterID = 14;

        public static final int intakePistonID = 2;
    }

    public static final class EjectorConstants {
        public static final int ejectorMotorRID = 9;
        public static final int ejectorMotorLID = 5;
        public static final double speed = -0.3;

    }

    public static final class DashboardStrings {
        public static final String waitInput = "Auto Wait Time";
        public static final String gearMode = "Gear Mode";
        public static final String drivetrainDirection = "Drivetrain Direction";
        public static final String climberLock = "Climber Lock On";
    }
    public static final class ClimbConstants {
        public static final int climbMotorID = 14;
        public static final double speed = 1;
        public static boolean firstEncoderCheck = true;
        public static boolean climberLock = true;
    }
}