// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.SwerveSubsystem;
import frc.Constants.OIConstants;
import frc.SwerveJoystickCmd;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx) ));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    //whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
