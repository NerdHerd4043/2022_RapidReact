// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ejector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;

public class Shoot extends CommandBase {
  public final Ejector shoot;
  public final Elevator elevator;
  public final double kSpeed;
  public final double eSpeed;

  /**
   * Creates a new Shoot
   * @param shoot The Ejector
   * @param elevator The Elevator
   * @param eSpeed Elevator Speed
   * @param kSpeed Ejector Kickout Speed
   */
  public Shoot(Ejector shoot, Elevator elevator, double eSpeed, double kSpeed) {
    this.shoot = shoot;
    this.elevator = elevator;
    this.kSpeed = kSpeed;
    this.eSpeed = eSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shoot);
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      shoot.spinShoot(kSpeed);
      elevator.moveBelt(eSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoot.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
