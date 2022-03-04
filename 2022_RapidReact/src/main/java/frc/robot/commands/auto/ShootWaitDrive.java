// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWaitDrive extends SequentialCommandGroup {
  /** Creates a new ShootThenDrive. */
  public ShootWaitDrive(Drivebase drivebase, Elevator elevator, Ejector shoot, double elevatorWait) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootAndWait(elevator, shoot, elevatorWait), new WaitThenForward(drivebase, elevatorWait));
  }
}
