// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import frc.robot.commands.intake.HarvestDown;
import frc.robot.commands.intake.HarvestUp;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootThenIntake extends SequentialCommandGroup {
  /** Creates a new ShootThenIntake. */
  public ShootThenIntake(Drivebase drivebase, Elevator elevator, Ejector shoot, Intake intake, double elevatorWait) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ShootAndWait(elevator, shoot, elevatorWait), 
    new DriveBaseWait(drivebase, elevatorWait), 
    new HarvestDown(intake),
    new IntakeAndDrive(drivebase, intake, .85, 4.1, true),
    new IntakeAndDrive(drivebase, intake, .5, 1, true),
    new HarvestUp(intake),
    new IntakeAndDrive(drivebase, intake, -.8, 5.1, false),
    new ShootAndWait(elevator, shoot, elevatorWait)
    );
  }
}