// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.IntakeConstants;

public class Intake extends SubsystemBase 
{
  private CANSparkMax harvesterMotor = new CANSparkMax(IntakeConstants.harvesterMotorID, MotorType.kBrushless);
  private WPI_TalonSRX kickupMotor = new WPI_TalonSRX(IntakeConstants.kickupMotorID);

  /** Creates a new Intake. */
  public Intake() 
  {
    kickupMotor.setSafetyEnabled(false);
  }

  public void spinIntake(double hSpeed, double kSpeed)
  {
    harvesterMotor.set(hSpeed);
    kickupMotor.set(kSpeed);
  }

  public void stopIntake()
  {
    harvesterMotor.stopMotor();
    kickupMotor.stopMotor();
  }

  public boolean isDown()
  {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
