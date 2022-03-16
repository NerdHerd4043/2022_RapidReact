// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.commands.intake.HarvestDown;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;

public class IntakeAndDrive extends CommandBase {

  private final Drivebase drivebase;
  private final Intake intake;
  private final double speed;
  private final boolean runIntake;
  private final double distance;
  private double encoderStart;


  /** Creates a new IntakeAndDrive. */
  public IntakeAndDrive(Drivebase drivebase, Intake intake, double speed, double distance, boolean runIntake) {
    this.drivebase = drivebase;
    this.intake = intake;
    this.speed = speed;
    this.distance = distance;
    this.runIntake = runIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase, this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderStart = drivebase.getAverageEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(runIntake){
      intake.spinIntake(0.9, 0.9);
    }
    drivebase.arcadeDrive(-speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if(speed > 0){ 
      return drivebase.getAverageEncoderValue() - encoderStart > (distance); //must go at least 12 ft 9 in
     }
     else{
      return drivebase.getAverageEncoderValue() - encoderStart < -(distance); //must go at least 12 ft 9 in
     }
  }
}
