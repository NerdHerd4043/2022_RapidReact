// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.Timer;


public class ShootAndWait extends CommandBase {
  /** Creates a new ShootAndWait. */
  public final Ejector shoot;
  public final Elevator elevator;
  private double timerGoal;
  private double timerStart;

  public ShootAndWait(Elevator elevator, Ejector shoot) {
    this.elevator = elevator;
    this.shoot = shoot;
    timerStart = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.elevator, this.shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = Timer.getFPGATimestamp();
    timerGoal = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.spinShoot(1);
    elevator.moveBelt(.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - timerStart) > timerGoal;
  }
}
