package frc.robot.subsystems;
/*
 * Need combine the code from https://github.com/SuperiorRoboworksTeam857/2023SwerveTest/blob/main/src/main/java/frc/robot/subsystems/Swerve.java 
 *   since its code is updated to 2023 library code while https://github.com/frc3512/SwerveBot-2022/blob/main/src/main/java/frc/robot/subsystems/Swerve.java is still using 2022 library
 * 
 * 
 */
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap;


import edu.wpi.first.math.kinematics.SwerveModulePosition; // added 9/8/2023

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());  // constructor changed from 2022 to 2023

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants, false),
          new SwerveModule(1, Constants.Swerve.Mod1.constants, true),
          new SwerveModule(2, Constants.Swerve.Mod2.constants, false),
          new SwerveModule(3, Constants.Swerve.Mod3.constants, true)
        };


    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                //? ChassisSpeeds.fromFieldRelativeSpeeds(
                    //translation.getX(), translation.getY(), rotation, getYaw())
                //: new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
                ? new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
                    
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }


  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    //swerveOdometry.resetPosition(pose, getYaw());
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  // Added on 9/8/2023, copied from https://github.com/SuperiorRoboworksTeam857/2023SwerveTest/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
  //              or similarly from  https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    //return (Constants.Swerve.invertGyro)
    //    ? Rotation2d.fromDegrees(360 - gyro.getYaw())
    //    : Rotation2d.fromDegrees(gyro.getYaw());
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public double getYawInDegree() {
    return gyro.getYaw();
  }


  @Override
  public void periodic() {
    //swerveOdometry.update(getYaw(), getStates());
    // see the code from https://github.com/SuperiorRoboworksTeam857/2023SwerveTest/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
