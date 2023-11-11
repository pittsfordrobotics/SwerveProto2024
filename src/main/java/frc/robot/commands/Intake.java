// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class Intake extends CommandBase {
  /** Creates a new Intake. */
  private EndEffector endEffector;
  public Intake(EndEffector endEffector) {
    addRequirements(endEffector);
    this.endEffector = endEffector;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.start(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.stop();
    System.out.println("stopping");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return endEffector.hasIntaken();
  }
}
