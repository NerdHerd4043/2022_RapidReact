// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EjectorConstants;

public class Ejector extends SubsystemBase {
  private WPI_TalonSRX ejectorMotorL = new WPI_TalonSRX(EjectorConstants.ejectorMotorRID);
  private WPI_TalonSRX ejectorMotorR = new WPI_TalonSRX(EjectorConstants.ejectorMotorLID);
  /** Creates a new Intake. */
  public Ejector() {
    // ejectorMotorL.follow(ejectorMotorR, true);
  }

  public void spinShoot(double speed){
    ejectorMotorR.set(-speed);
    ejectorMotorL.set(speed);
  }

  public void stopShoot(){
    ejectorMotorR.stopMotor();
    ejectorMotorL.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
