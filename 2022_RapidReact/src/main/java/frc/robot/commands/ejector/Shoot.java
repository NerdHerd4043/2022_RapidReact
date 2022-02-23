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
  public final double speed;

  /** Creates a new Shoot. */
  public Shoot(Ejector shoot, Elevator elevator, double speed) {
    this.shoot = shoot;
    this.elevator = elevator;
    this.speed = speed;

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
      shoot.spinShoot(speed);
      elevator.moveBelt(speed);
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
