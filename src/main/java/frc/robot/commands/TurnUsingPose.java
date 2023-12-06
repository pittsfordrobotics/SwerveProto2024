// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;


// the goal:
// when a button is held the robot's target angle will be pointing at a specified point on the field


public class TurnUsingPose extends CommandBase {
  private Swerve swervedrive;
  private CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private double rotateX;
  private double rotateY;
  private Rotation2d targetAngle;

  /** Creates a new TurnUsingPose. */
  public TurnUsingPose(Swerve swervedrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervedrive);
    this.swervedrive = swervedrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxis = MathUtil.applyDeadband(-driverController.getLeftY(), SwerveConstants.driverControllerLeftDeadband); // update all controller inputs, Xbox controller has different X and Y directions
    double yAxis = MathUtil.applyDeadband(-driverController.getLeftX(), SwerveConstants.driverControllerLeftDeadband);

    Double robotx = swervedrive.getPose().getX();
    Double roboty = swervedrive.getPose().getY();

    Double targetx = 1.0;
    Double targety = 1.0;  

    Double xdiff = robotx - targetx;
    Double ydiff = roboty - targety;

    /*My understanding of the field system:
     * 0,0,0 is the "bottom left" corner of the field with the robots alliance grid being on the bottom left.
     * x is along the long side of the field
     * 0 theta is to the right of the field (pointing towards the opposing alliance's grid)
     * theta increases as you go counter clockwise
     */

    // Should work as long as the gyro is zeroed to the right
      targetAngle = Rotation2d.fromRadians(Math.atan2(ydiff, xdiff) + Math.PI);
  

    swervedrive.updateSwerveModuleStates(xAxis, yAxis, targetAngle);

    swervedrive.drive();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   // if the right joystick is moved, end the command
  //   rotateX = -driverController.getRightY();
  //   rotateY = -driverController.getRightX();
  //   double radius = MathUtil.applyDeadband(Math.sqrt(Math.pow(rotateX, 2) + Math.pow(rotateY, 2)), SwerveConstants.driverControllerRightDeadband);
  //   if (!(radius == 0)){
  //     return true;
  //   }
    return false;
  }
}
