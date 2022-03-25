// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DashboardStrings;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleLock extends InstantCommand {
  public ToggleLock() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ClimbConstants.climberLock){
      ClimbConstants.climberLock = false;
      SmartDashboard.putString(DashboardStrings.climberLock, "Lock is off");
    }
    else{
      ClimbConstants.climberLock = true;
      SmartDashboard.putString(DashboardStrings.climberLock, "Lock is on");
    }
  }
}
