// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.Climber;
import frc.robot.commands.climber.ToggleLock;
import frc.robot.commands.intake.*;
import frc.robot.commands.ejector.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.drivebase.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivebase drivebase = new Drivebase();
  private final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  private final Ejector shoot = new Ejector();
  public final Climb climb = new Climb();
  
  private final DriveForward driveForward = new DriveForward(drivebase);
  private final DriveForwardWithWait waitThenForward = new DriveForwardWithWait(drivebase, 0);
  private final OneBallAuto oneBallAuto = new OneBallAuto(drivebase, elevator, shoot, RobotConstants.elevatorWaitTime);
  private final TwoBallAuto twoBallAuto = new TwoBallAuto(drivebase, elevator, shoot, intake, RobotConstants.elevatorWaitTime);
  private final TwoBallWallAuto twoBallWallAuto = new TwoBallWallAuto(drivebase, elevator, shoot, intake, RobotConstants.elevatorWaitTime);

  public Double startPosition = climb.getEncoderValue();

  SendableChooser<Command> commandChooser = new SendableChooser<>();

  private UsbCamera usbCamera = new UsbCamera("Intake Camera", 0);
  private MjpegServer mjpegServer = new MjpegServer("serv_Intake Camera", 1181);

  private static XboxController driveStick = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mjpegServer.setSource(usbCamera);

    commandChooser.addOption("WaitThenDrive", waitThenForward);
    commandChooser.addOption("DriveForwardsImmediatly", driveForward);
    commandChooser.addOption("1 Ball Auto", oneBallAuto);
    commandChooser.setDefaultOption("2 Ball Auto", twoBallAuto); 
    commandChooser.addOption("2 Ball Wall Auto", twoBallWallAuto); 

    SmartDashboard.putData(commandChooser);
    
    CameraServer.startAutomaticCapture();

    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            () -> driveStick.getLeftY(),
            () -> driveStick.getRightX()));

    elevator.setDefaultCommand(
        new DefaultElevator(
            elevator,
            () -> driveStick.getRightTriggerAxis() - driveStick.getLeftTriggerAxis()));

    shoot.setDefaultCommand(
        new HoldBall(
            shoot,
            () -> driveStick.getRightTriggerAxis()));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driveStick,
    // Button.kLeftBumper.value).toggleWhenPressed(new Harvest(intake, .5, .5),
    // true);
    // new JoystickButton(driveStick,
    // Button.kRightBumper.value).toggleWhenPressed(new Harvest(intake, -.5, -.5),
    // true); 
    new JoystickButton(driveStick, Button.kA.value).whenPressed(new HarvestDown(intake), true);
    new JoystickButton(driveStick, Button.kA.value).whenPressed(new Harvest(intake, 0.9, 0.9), true);
    new JoystickButton(driveStick, Button.kB.value).whenPressed(new HarvestUp(intake), true);
    new JoystickButton(driveStick, Button.kB.value).whenPressed(new Harvest(intake, 0, 0), true);
    new JoystickButton(driveStick, Button.kX.value).whenHeld(new Shoot(shoot, elevator, 1, 0.65), true);
    new JoystickButton(driveStick, Button.kY.value).whenPressed(new FlipFront(), true);
    new JoystickButton(driveStick, Button.kLeftBumper.value).whenPressed(new ShiftDown(drivebase), true);
    new JoystickButton(driveStick, Button.kRightBumper.value).whenPressed(new ShiftUp(drivebase), true);
    
    new JoystickButton(driveStick, Button.kBack.value).whenPressed(new Harvest(intake, 0, 1), true);
    new JoystickButton(driveStick, Button.kStart.value).whenHeld(new Harvest(intake, -.5, -.5), true);
    new POVButton(driveStick, 0).whenHeld(new Climber(climb, ClimbConstants.speed));
    new POVButton(driveStick, 180).whenHeld(new Climber(climb, -ClimbConstants.speed));
    new POVButton(driveStick, 90).whenPressed(new ToggleLock());
    // new JoystickButton(driveStick, Button.kStart.value).whenReleased(new Harvest(intake, .5, .5), true);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return commandChooser.getSelected();
  }


}

// hi