// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivebase drivebase = new Drivebase();
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();
  private final Ejector shoot = new Ejector();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private static XboxController driveStick = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivebase.setDefaultCommand( 
      new Drive(
        drivebase,
        () -> driveStick.getLeftY(),
        () -> driveStick.getRightX()));

    elevator.setDefaultCommand(
      new DefaultElevator(
        elevator, 
        () -> driveStick.getRightTriggerAxis() - driveStick.getLeftTriggerAxis())
    );

    shoot.setDefaultCommand(
      new HoldBall(
        shoot, 
        () -> driveStick.getRightTriggerAxis())
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    new JoystickButton(driveStick, Button.kLeftBumper.value).toggleWhenPressed(new Harvest(intake, .5, .5), true);
    new JoystickButton(driveStick, Button.kRightBumper.value).toggleWhenPressed(new Harvest(intake, -.5, -.5), true);

    new JoystickButton(driveStick, Button.kB.value).whenPressed(new HarvestDown(), true);
    new JoystickButton(driveStick, Button.kX.value).whenHeld(new Shoot(shoot, 1), true);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
