package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean isFieldRelative;

  private boolean driveStraightFlag = false;
  private double driveStraightAngle = 0;


  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
  }

  @Override
  public void execute() {

    double speedMultiplier = slowSpeedSup.getAsBoolean() ? 0.8 : 0.5; //0.2, 0.5 //0.05, 0.2

    double joystickX = 0.0;
    
    /* Get Values, Deadband*/
     double translationVal =
        translationLimiter.calculate(
            speedMultiplier *
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            speedMultiplier *
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            speedMultiplier *
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));


    // handle drive straight 
    if (RobotContainer.oi.driverStick.getRightTriggerAxis() > 0.4) { //drive straight button
              if (!driveStraightFlag) {
                driveStraightAngle = s_Swerve.getYawInDegree();
                driveStraightFlag = true;
              }
              double vinniesError = driveStraightAngle - s_Swerve.getYawInDegree();
              joystickX = vinniesError * RobotMap.driveStraightProportion;

              // in drive straight mode, ignore rotation and strafe
              rotationVal = joystickX;
              strafeVal = 0;

    }
    else {
      driveStraightFlag = false;
    }
        
        
        
        
    //strafeVal = 0;
    
    /* Drive */  
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        //new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        //rotationVal * 0,
        //false,
        !robotCentricSup.getAsBoolean(), 
        RobotMap.isOpenLoop);
        
  }
}
