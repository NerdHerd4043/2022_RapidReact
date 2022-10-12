// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase 
{
  private CANSparkMax harvesterMotor = new CANSparkMax(IntakeConstants.harvesterMotorID, MotorType.kBrushless);
  private CANSparkMax kickupMotor = new CANSparkMax(IntakeConstants.kickupMotorID, MotorType.kBrushless);
  private CANSparkMax miniHarvester = new CANSparkMax(IntakeConstants.miniHarvesterID, MotorType.kBrushless);

  private Solenoid intakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.intakePistonID);
  /** Creates a new Intake. */
  public Intake() 
  {
    kickupMotor.restoreFactoryDefaults();
    harvesterMotor.restoreFactoryDefaults();
    miniHarvester.restoreFactoryDefaults();

    kickupMotor.setIdleMode(IdleMode.kBrake);
    harvesterMotor.setIdleMode(IdleMode.kBrake);
    miniHarvester.setIdleMode(IdleMode.kBrake);

    harvesterMotor.setOpenLoopRampRate(0.5f);
  }

  public void spinIntake(double hSpeed, double kSpeed)
  {
    harvesterMotor.set(hSpeed);
    kickupMotor.set(-kSpeed);
    miniHarvester.set(kSpeed);
  }

  public void stopIntake()
  {
    harvesterMotor.stopMotor();
    kickupMotor.stopMotor();
    miniHarvester.stopMotor();
  }

  public void setIntakePiston(boolean z){
   intakePiston.set(z);
   System.out.println("Piston: " + z);
  }

  public boolean isDown() {
    return intakePiston.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
