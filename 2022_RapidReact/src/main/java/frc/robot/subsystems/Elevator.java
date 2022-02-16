// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants.ElevatorConstants;

public class Elevator extends SubsystemBase {
private WPI_TalonSRX beltMotor = new WPI_TalonSRX(ElevatorConstants.beltMotorID);


  /** Creates a new Elevator. */
  public Elevator() {
    beltMotor.setSafetyEnabled(false);
  }

  public void moveBelt(double speed){
    beltMotor.set(speed);
  }

  public void stopBelt() {
    beltMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
