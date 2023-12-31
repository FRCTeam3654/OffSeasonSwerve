// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/*
 *  reference to: https://github.com/SuperiorRoboworksTeam857/2023SwerveTest/blob/main/src/main/java/frc/robot/RobotContainer.java 
 * 
 *           and Team857's 2023 competition code: https://github.com/SuperiorRoboworksTeam857/2023ChargedUp/blob/main/src/main/java/frc/robot/RobotContainer.java 
 * 
 *  some issue with syntax:  () -> -driver.getRawAxis(translationAxis)
 *  Just remove the lambda function syntax, it should be ok:  driver.getRawAxis(translationAxis)
 */
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static OI oi;
  
  /* Controllers */
  //private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;




  // Added on 9/8/2023, copied from https://github.com/SuperiorRoboworksTeam857/2023SwerveTest/blob/main/src/main/java/frc/robot/RobotContainer.java
  //private final JoystickButton slowSpeed =
      //new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    oi = new OI();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -oi.driverStick.getRawAxis(translationAxis),
            () -> -oi.driverStick.getRawAxis(strafeAxis),
            () -> -oi.driverStick.getRawAxis(rotationAxis),
            () -> oi.robotCentric.getAsBoolean(),
            () -> oi.slowSpeed.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    //oi.zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    oi.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


    oi.turnLeft180Button.onTrue(new TurnToAngleCommand(s_Swerve, 180, 3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
