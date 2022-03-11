// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivebase;

public class DriveForward extends CommandBase {

  private final Drivebase drivebase;
  private double encoderStart;


  /** Creates a new Auto. */
  public DriveForward(Drivebase drivebase) {
    this.drivebase = drivebase;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderStart = drivebase.getAverageEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    drivebase.arcadeDrive(-.7, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 37.4 per foot
    return drivebase.getAverageEncoderValue() - encoderStart > (37.6 * 4); //must go at least 7 feet
  }
}
