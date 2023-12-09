// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;

public class GoToPointUsingPose_VisionProof extends CommandBase {
  /** Creates a new GoToPointUsingPose_VisionProof. */
  private Swerve swervedrive;

  public GoToPointUsingPose_VisionProof(Swerve swervedrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervedrive);
    this.swervedrive = swervedrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    Optional<Pose3d> pointofintrest = FieldConstants.aprilTags.getTagPose(2);
    Double targetx = pointofintrest.get().getX() + 2;
    Double targety = pointofintrest.get().getY();

    Rotation2d targetRotation = new Rotation2d(180);

    Pose2d targetPose = new Pose2d(targetx, targety, targetRotation);

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      // low limits for testing
        1.5, 2.0,
        Units.degreesToRadians(180), Units.degreesToRadians(360));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    pathfindingCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
