// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;

public class VisionTurnToAngle extends CommandBase {

  private final Swerve m_robotDrive;
  private final PhotonCamera m_camera;
  private boolean complete = false;
  private double angle;
  private Timer timer = new Timer();
  private double timeout;
  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
  
  public VisionTurnToAngle(Swerve subsystem, PhotonCamera camera, double timeoutS){
      m_robotDrive = subsystem;
      m_camera = camera;
      timeout = timeoutS;
      addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
      timer.reset();
      timer.start();
      complete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double forwardSpeed;
    double gyroAngle = m_robotDrive.getYaw().getDegrees(); 
    // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = m_camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                RobotMap.VISION_CAMERA_HEIGHT_METERS,
                                RobotMap.VISION_TARGET_HEIGHT_METERS,
                                RobotMap.VISION_CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));

                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -forwardController.calculate(range, RobotMap.VISION_GOAL_RANGE_METERS);

                // Also calculate angular power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                angle = result.getBestTarget().getYaw();
                angle = Math.toDegrees(angle);
            } else {
                // If we have no targets, stay still.
                complete = true;
                angle = gyroAngle;
                forwardSpeed = 0;
            }
        
      final double kP = 0.06;
      SmartDashboard.putNumber("gyroAngle", gyroAngle);
  
      double err = angle - gyroAngle;
      double speed = MathUtil.clamp(err * kP, -Constants.Swerve.maxAngularVelocity*0.5, Constants.Swerve.maxAngularVelocity*0.5);
  
      if (Math.abs(err) > 2 && timer.get() < timeout) {
          m_robotDrive.drive(new Translation2d(forwardSpeed,0), speed, false, true);
      } else {
          complete = true;
      } 
  } 

  @Override
  public void end(boolean inturrupted){
     // m_robotDrive.drive(new Translation2d(0,0), 0, false, true);
     // timer.stop();
  }

  @Override
  public boolean isFinished(){
      return complete;
  }
}
