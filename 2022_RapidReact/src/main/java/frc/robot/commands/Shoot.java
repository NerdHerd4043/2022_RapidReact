// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ejector;

public class Shoot extends CommandBase {
  public final Ejector shoot;
  public final double speed;

  /** Creates a new Shoot. */
  public Shoot(Ejector shoot, double speed) {
    this.shoot = shoot;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      shoot.spinShoot(speed);
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
