// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

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
  private final SwerveDriveSubsystem frontLeftSub = new SwerveDriveSubsystem(
      Constants.DriveID.FRONT_LEFT.getID(),
      Constants.SpinID.FRONT_LEFT.getID());
  private final SwerveDriveSubsystem backLeftSub = new SwerveDriveSubsystem(
      Constants.DriveID.BACK_LEFT.getID(), Constants.SpinID.BACK_LEFT.getID());
  private final SwerveDriveSubsystem backRightSub = new SwerveDriveSubsystem(14, 15);
  private final SwerveDriveSubsystem frontRightSub = new SwerveDriveSubsystem(16, 17);

  private final SwerveDrive swerve = new SwerveDrive(frontLeftSub, frontRightSub, backLeftSub, backRightSub);

  private final SwerveDriveCommand m_teleopCommand = new SwerveDriveCommand(swerve);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public Command getTeleopCommand() {
    // An ExampleCommand will run in autonomous
    return m_teleopCommand;
  }
}
