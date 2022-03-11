// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveBaseWait extends CommandBase {
  private final Drivebase drivebase;
  private double timerGoal;
  private double timerStart;
  private double waitTime;

  /** Creates a new DriveBaseWait. */
  public DriveBaseWait(Drivebase drivebase, double elevatorTime) {
    this.drivebase = drivebase;
    waitTime = elevatorTime;
    timerStart = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = Timer.getFPGATimestamp();
    timerGoal = drivebase.getWaitInput() - waitTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Ran stuff in Wait" + timerGoal);
    return (Timer.getFPGATimestamp() - timerStart) > timerGoal;
  }
}
