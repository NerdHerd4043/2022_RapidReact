// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  // private final TestTrajectory testTrajectory = new TestTrajectory(drivebase);
  
//-------------------------------------------------------------------------------------------------------------------------
  // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
  // config.setKinematics(drivebase.getKinematics());

  // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
  //   Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), //starts at 0, drives forward 1 meter at 0 degrees
  //   config
  // );

  // RamseteCommand command = new RamseteCommand(
  //   trajectory,
  //   drivebase::getPose,
  //   new RamseteController(2.0, 0.7),
  //   drivebase.getFeedforward(),
  //   drivebase.getKinematics(),
  //   drivebase::getSpeeds,
  //   drivebase.getLeftPIDController(),
  //   drivebase.getRightPIDController(),
  //   drivebase::setOutput,
  //   drivebase
  // );

//------------------------------------------------------------------------------------------------------------------------
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
    // commandChooser.addOption("Follows a set path?", command);

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
    new JoystickButton(driveStick, Button.kA.value).whenPressed(new Harvest(intake, 0.7, 0.9), true);
    new JoystickButton(driveStick, Button.kB.value).whenPressed(new HarvestUp(intake), true);
    new JoystickButton(driveStick, Button.kB.value).whenPressed(new Harvest(intake, 0, 0), true);
    new JoystickButton(driveStick, Button.kX.value).whenHeld(new Shoot(shoot, elevator, 1, 0.65), true);
    new JoystickButton(driveStick, Button.kY.value).whenPressed(new FlipFront(), true);
    new JoystickButton(driveStick, Button.kLeftBumper.value).whenPressed(new ShiftDown(drivebase), true);
    new JoystickButton(driveStick, Button.kRightBumper.value).whenPressed(new ShiftUp(drivebase), true);
    
    new JoystickButton(driveStick, Button.kBack.value).whenPressed(new Harvest(intake, 0, 1), true);
    new JoystickButton(driveStick, Button.kStart.value).whenHeld(new Harvest(intake, -.5, -.5), true);
    new POVButton(driveStick, 0).whenHeld(new Climber(climb, ClimbConstants.speed, false));
    new POVButton(driveStick, 180).whenHeld(new Climber(climb, -ClimbConstants.speed, false));
    new POVButton(driveStick, 90).whenPressed(new ToggleLock());
    new POVButton(driveStick, 270).whenPressed(new Climber(climb, 0, true));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.AutoTrajectory.kS, 
        DriveConstants.AutoTrajectory.kV,
        DriveConstants.AutoTrajectory.kA), 
        DriveConstants.AutoTrajectory.kinematics, 
        10);

    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.AutoTrajectory.maxSpeed, 
      DriveConstants.AutoTrajectory.maxAcceleration)
      .setKinematics(DriveConstants.AutoTrajectory.kinematics)
      .addConstraint(autoVoltageConstraint);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), //starting point 
      List.of(new Translation2d(0.25, 0)), //pass through this point
      new Pose2d(0.5, 0, new Rotation2d(0)), //end at 1 meters facing forwards 
      config);

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      drivebase::getPose, 
      new RamseteController(DriveConstants.AutoTrajectory.ramseteB, DriveConstants.AutoTrajectory.ramseteZeta), 
      new SimpleMotorFeedforward(
        DriveConstants.AutoTrajectory.kS, 
        DriveConstants.AutoTrajectory.kV,
        DriveConstants.AutoTrajectory.kA), 
      DriveConstants.AutoTrajectory.kinematics, 
      drivebase::getWheelSpeeds, 
      new PIDController(DriveConstants.AutoTrajectory.kP, 0, 0), 
      new PIDController(DriveConstants.AutoTrajectory.kP, 0, 0), 
      drivebase::tankDriveVolts, 
      drivebase);

    drivebase.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0));

    // commandChooser.addOption("aaaaahhh", pathTracer);

    // return commandChooser.getSelected();
  }


}

// hi