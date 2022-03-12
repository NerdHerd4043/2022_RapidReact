// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardStrings;
import frc.robot.Constants.DriveConstants;


public class Drivebase extends SubsystemBase {
  private DifferentialDrive diffDrive;

  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);
  
  private Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);
  
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

//hi