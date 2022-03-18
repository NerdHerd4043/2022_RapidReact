// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;
import frc.robot.commands.autoCommands.DriveBaseWait;
import frc.robot.commands.autoCommands.IntakeAndDrive;
import frc.robot.commands.autoCommands.ShootAndWait;
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
public class TwoBallWallAuto extends SequentialCommandGroup {
  /** Creates a new ShootThenIntake. */
  public TwoBallWallAuto(Drivebase drivebase, Elevator elevator, Ejector shoot, Intake intake, double elevatorWait) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //distance is 14 * feet
    addCommands(
    new ShootAndWait(elevator, shoot, elevatorWait), 
    new DriveBaseWait(drivebase, elevatorWait), 
    new HarvestDown(intake),
    new IntakeAndDrive(drivebase, intake, .85, 14 * 6, true),
    new IntakeAndDrive(drivebase, intake, .5, 14 * 1.5, true),
    new IntakeAndDrive(drivebase, intake, -.85, 14 * 7.5, true),
    new ShootAndWait(elevator, shoot, elevatorWait),
    new HarvestUp(intake)
    );
  }
}