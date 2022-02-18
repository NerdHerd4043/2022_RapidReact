// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Harvest extends CommandBase {
  public final Intake intake;
  public final double hSpeed;
  public final double kSpeed;

  /** Creates a new DefaultIntake. */
  public Harvest(Intake intake, double hSpeed, double kSpeed)
  {
    this.intake = intake;
    this.hSpeed = hSpeed;
    this.kSpeed = kSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(intake.isDown()) {
      intake.spinIntake(hSpeed, kSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

//hi