// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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

        public static final int[] leftEncoderPorts = new int[] {10, 6};
        public static final int[] rightEncoderPorts = new int[] {8, 3};

        public static final boolean leftEncoderReversed = false;
        public static final boolean rightEncoderReversed = true;

        public static final int encoderCPR = 1; //according to sysId, rev already handles the encoder value
        public static final double wheelDiameter = 6; //in inches
        public static final double encoderDistancePerPulse = (Units.inchesToMeters(wheelDiameter) * Math.PI) / (double) encoderCPR;

        public static final double highGearRamp = 0.2;
        public static final double lowGearRamp = 0.1;

        public static final int stallLimit = 80; //current limit when motors are stopped
        public static final int freeLimit = 80; //max current limit

        public static final int shifterID = 1;

        public static boolean harvesterIsFront = true;

        public static final class Gears {
            public static final boolean highGear = true;
            public static final boolean lowGear = false;
            public static boolean isHighGear = true;
        }

        public static final class AutoTrajectory {
            public static final double gearRatio = 24; //24:1
            public static final double trackWidth = 0.63; //in meters
            public static final double kS = -0.038805; //These 4 numbers might be different since i recalibrated the gyro, 
            public static final double kV = 6.0223;    //but I dont have room to run the SysId again
            public static final double kA = 5.8034;
            public static final double kP = 5.3654;

            public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

            public static final double maxSpeed = 0.25; //meters per second
            public static final double maxAcceleration = 0.25; //meters per second squared

            public static final double ramseteB = 2;  //these two numbers work for most robots (supposedly)
            public static final double ramseteZeta = 0.7; //i don't know what they mean
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
        public static final int miniHarvestMotorID = 15;
        public static final int kickupMotorID = 11;
        
        public static final int intakePistonID = 2;

        public static boolean harvestUp = true;
    }

    public static final class EjectorConstants {
        public static final int ejectorMotorRID = 9;
        public static final int ejectorMotorLID = 5;
        public static final double speed = -1;

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