// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardStrings;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;


public class Drivebase extends SubsystemBase {
  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);
  
  private Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);

  private final DifferentialDriveOdometry odometry;

  Pose2d pose;

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  private final Encoder leftEncoder = new Encoder(
    DriveConstants.leftEncoderPorts[0],
    DriveConstants.leftEncoderPorts[1],
    DriveConstants.leftEncoderReversed);

  private final Encoder rightEncoder = new Encoder(
    DriveConstants.rightEncoderPorts[0],
    DriveConstants.rightEncoderPorts[1],
    DriveConstants.rightEncoderReversed);

  AHRS gyro = new AHRS(SPI.Port.kMXP); 
  
  /** Creates a new Drivebase. */
  public Drivebase() {
    backLeftMotor.restoreFactoryDefaults();
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
   
    //backLeftMotor.setIdleMode(IdleMode.kCoast);
    //backRightMotor.setIdleMode(IdleMode.kCoast);
    //frontLeftMotor.setIdleMode(IdleMode.kCoast);
    //frontRightMotor.setIdleMode(IdleMode.kCoast);
   
    frontLeftMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);
    frontRightMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    frontRightMotor.setInverted(true);

    diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    //putWaitInput();

    PIDController pid = new PIDController(TrajectoryConstants.kP, TrajectoryConstants.kI, TrajectoryConstants.kD);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));

  }

  public void arcadeDrive(double fwd, double rot) {
    diffDrive.arcadeDrive(fwd, rot, true);
    // frontLeftMotor.set(fwd);
  }

  public void shift(boolean a) {
    shifter.set(a);

    if(a){
      frontLeftMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);
      frontRightMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);
    }
    else{
      frontLeftMotor.setOpenLoopRampRate(DriveConstants.lowGearRamp);
      frontRightMotor.setOpenLoopRampRate(DriveConstants.lowGearRamp);
    }
  }

  public double getAverageEncoderValue() {
    System.out.println(frontRightMotor.getEncoder().getCountsPerRevolution());
    // System.out.println("L: " + frontLeftMotor.getEncoder().getPosition() + " -- R: " + frontRightMotor.getEncoder().getPosition());
    return (-frontLeftMotor.getEncoder().getPosition() + -frontRightMotor.getEncoder().getPosition()) / 2;
  }

  public void putWaitInput() {
    SmartDashboard.putNumber(DashboardStrings.waitInput, 2);
  }

  public double getWaitInput() {
    return SmartDashboard.getNumber(DashboardStrings.waitInput, 2);
  }

  public Pose2d getRobotPos() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelVelocity() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void setRobotPos(Pose2d startingPose) {
    resetEncoders();
    odometry.resetPosition(startingPose, gyro.getRotation2d());
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

    pose = odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }
}