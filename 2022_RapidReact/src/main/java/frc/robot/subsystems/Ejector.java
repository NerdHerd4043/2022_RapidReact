// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.EjectorConstants;

public class Ejector extends SubsystemBase {
  private CANSparkMax ejectorMotorL = new CANSparkMax (EjectorConstants.ejectorMotorRID, MotorType.kBrushless);
  private CANSparkMax ejectorMotorR = new CANSparkMax (EjectorConstants.ejectorMotorLID, MotorType.kBrushless);
  /** Creates a new Intake. */
  public Ejector() {
    ejectorMotorL.follow(ejectorMotorR);
    ejectorMotorL.setInverted(true);
  }

  public void spinShoot(double speed){
    ejectorMotorL.set(speed);
  }

  public void stopShoot(){
    ejectorMotorL.stopMotor();;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
