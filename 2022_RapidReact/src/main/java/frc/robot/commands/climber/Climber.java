// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class Climber extends CommandBase {
  public final Climb climb;
  public final double speed;
  public double startPosition;
  // private final double startPosition;

  /** Creates a new climber. */
  public Climber(Climb climb, double speed /*, Double startPosition*/) {
    this.climb = climb;
    this.speed = speed;
    // this.startPosition = startPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climb);  
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ClimbConstants.firstEncoderCheck)
    {
      startPosition = climb.getEncoderValue();
      ClimbConstants.firstEncoderCheck = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((speed > 0.05 && !ClimbConstants.climberLock) || 
    (climb.getEncoderValue() - startPosition >= -75 && speed < -0.05)){
      climb.moveClimb(speed);
    }
    else{
      climb.stopClimb();
    }
    //one rotation  is 1.732 inches
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopClimb(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
