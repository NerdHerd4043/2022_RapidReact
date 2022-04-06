// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private CANSparkMax climbMotor = new CANSparkMax (ClimbConstants.climbMotorID, MotorType.kBrushless);
  
  /** Creates a new Climb. */
  public Climb() {
    climbMotor.restoreFactoryDefaults();

    climbMotor.setIdleMode(IdleMode.kBrake);
    // climbMotor.setSmartCurrentLimit(25, 25, 0);
  
  }

  public double getEncoderValue()
  {
    return climbMotor.getEncoder().getPosition();
  }

  public void moveClimb(double speed){
    climbMotor.set(speed);
  }

  public void stopClimb() {
    climbMotor.stopMotor();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
