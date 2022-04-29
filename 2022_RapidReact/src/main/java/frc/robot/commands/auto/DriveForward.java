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
    System.out.println(drivebase.getAverageEncoderValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*~14 per foot <--- not true
    42 ticks per revolution?
    low gear is 24:1?             pain
    high gear is 9.4:1?
    circumfrence of wheel is about 9.8175*/
    return drivebase.getAverageEncoderValue() - encoderStart > (14 * 8);
    // return drivebase.getAverageEncoderValue() - encoderStart > (37.6 * 4); //must go at least 7 feet
  }
}
