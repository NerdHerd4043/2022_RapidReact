// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ejector;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.EjectorConstants;
import frc.robot.subsystems.Ejector;

public class HoldBall extends CommandBase {

  public final DoubleSupplier triggerPosition;
  public final Ejector shoot;
  /** Creates a new HoldBall. */
  public HoldBall(Ejector shoot, DoubleSupplier triggerPosition) {
    this.triggerPosition = triggerPosition;
    this.shoot = shoot;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(triggerPosition.getAsDouble() > 0.05){
      shoot.spinShoot(EjectorConstants.speed);
    } else {
      shoot.stopShoot();
    }
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
