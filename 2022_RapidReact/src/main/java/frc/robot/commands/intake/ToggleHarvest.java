// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleHarvest extends InstantCommand {
  public final Intake intake;

  public ToggleHarvest(Intake intake) {
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Harvest(intake, 0, 0);
    if(IntakeConstants.harvestUp){
      new HarvestDown(intake);
      IntakeConstants.harvestUp = false;
    }
    else{
      new HarvestUp(intake);
      IntakeConstants.harvestUp = true;
    }
  }
}
