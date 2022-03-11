// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlipFront extends InstantCommand {
  // Drivebase drivebase;

  public FlipFront(/*Drivebase drivebase*/) {
    // this.drivebase = drivebase;

    // // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriveConstants.elevatorIsFront){
      DriveConstants.elevatorIsFront = false;
    }
    else{
      DriveConstants.elevatorIsFront = true;
    }
  }
}
