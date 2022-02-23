// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants.DriveConstants;

public class ShiftUp extends InstantCommand {
  Drivebase drivebase;
  /** Creates a new ShiftUp. */
  public ShiftUp(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.drivebase = drivebase;

  addRequirements(this.drivebase);
}
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.shift(DriveConstants.Gears.highGear);
  }
  // Called every time the scheduler runs while the command is scheduled.
}
