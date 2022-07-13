// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*information on how the trajectory pathing works can be found here: 
https://docs.google.com/spreadsheets/d/16xm2-9WJ5vWOSsAj0IUChquzY4FGqDyriKc_F4rh2h0/edit#gid=0*/

package frc.robot.subsystems;

import java.io.NotActiveException;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
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


public class Drivebase extends SubsystemBase {
  // private DifferentialDrive diffDrive;

  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
  
  private Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);

  AHRS gyro = new AHRS(SPI.Port.kMXP); 

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  public final DifferentialDriveOdometry odometry;

  private final Encoder leftEncoder = new Encoder(
    DriveConstants.leftEncoderPorts[0],
    DriveConstants.leftEncoderPorts[1],
    DriveConstants.leftEncoderReversed);

  private final Encoder rightEncoder = new Encoder(
    DriveConstants.rightEncoderPorts[0],
    DriveConstants.rightEncoderPorts[1],
    DriveConstants.rightEncoderReversed);

  // DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.AutoTrajectory.trackWidth); //0.63 is the track width in meters
  // DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  // Pose2d pose;

  // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward
  // (DriveConstants.AutoTrajectory.kS, 
  // DriveConstants.AutoTrajectory.kV, 
  // DriveConstants.AutoTrajectory.kA);
  
  // PIDController leftPIDController = new PIDController(DriveConstants.AutoTrajectory.kP, 0, 0);
  // PIDController rightPIDController = new PIDController(DriveConstants.AutoTrajectory.kP, 0, 0);

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
   
    // frontLeftMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);
    // frontRightMotor.setOpenLoopRampRate(DriveConstants.highGearRamp);   //uncomment later

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    // frontLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    // backLeftMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    // frontRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);
    // backRightMotor.setSmartCurrentLimit(DriveConstants.currentLimit);

    // frontLeftMotor.setSmartCurrentLimit(DriveConstants.stallLimit, DriveConstants.freeLimit);  uncomment later
    // backLeftMotor.setSmartCurrentLimit(DriveConstants.stallLimit, DriveConstants.freeLimit);
    // frontRightMotor.setSmartCurrentLimit(DriveConstants.stallLimit, DriveConstants.freeLimit);
    // backRightMotor.setSmartCurrentLimit(DriveConstants.stallLimit, DriveConstants.freeLimit);
    frontRightMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(DriveConstants.encoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.encoderDistancePerPulse);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    zeroHeading();

    // diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    // putWaitInput();

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

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public void resetEncoders(){
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getAverageEncoderDistance(){
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder(){
    return leftEncoder;
  }

  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput){
    diffDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate(){
    return -gyro.getRate();
  }

//---------------------------------------------------------------------------------------------------------------------
  // public Rotation2d getHeading(){
  //   return Rotation2d.fromDegrees(-gyro.getAngle());
  // }

  // public DifferentialDriveWheelSpeeds getSpeeds() {
  //     return new DifferentialDriveWheelSpeeds(
  //       frontRightMotor.getEncoder().getVelocity() / DriveConstants.AutoTrajectory.gearRatio * Math.PI * Units.inchesToMeters(DriveConstants.AutoTrajectory.wheelDiameter) / 60,
  //       frontLeftMotor.getEncoder().getVelocity() / DriveConstants.AutoTrajectory.gearRatio * Math.PI * Units.inchesToMeters(DriveConstants.AutoTrajectory.wheelDiameter) / 60
  //     );
  // }

  // public SimpleMotorFeedforward getFeedforward() {
  //   return feedforward;
  // }

  // public PIDController getLeftPIDController() {
  //   return leftPIDController;
  // }

  // public PIDController getRightPIDController() {
  //   return rightPIDController;
  // }

  // public DifferentialDriveKinematics getKinematics() {
  //   return kinematics;
  // }

  // public Pose2d getPose() {
  //   return pose;
  // }

  // public void setOutput(double leftVolts, double rightVolts){
  //   frontLeftMotor.set(leftVolts / 12);
  //   frontRightMotor.set(rightVolts / 12);
  // }
//-----------------------------------------------------------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pose = odometry.update(getHeading(),
    // frontRightMotor.getEncoder().getVelocity() / DriveConstants.AutoTrajectory.gearRatio * Math.PI * Units.inchesToMeters(DriveConstants.AutoTrajectory.wheelDiameter) / 60,
    // frontLeftMotor.getEncoder().getVelocity() / DriveConstants.AutoTrajectory.gearRatio * Math.PI * Units.inchesToMeters(DriveConstants.AutoTrajectory.wheelDiameter) / 60
    // );
    //come back to this, don't know if these parameters are right

    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance()); 
    // System.out.println(gyro.getRotation2d() + ", " + getHeading() + ", " + getTurnRate());   
  }
}