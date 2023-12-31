// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToPointUsingPose_VisionProof;
import frc.robot.commands.Intake;
import frc.robot.commands.SwerveDriveXbox;
import frc.robot.commands.SwerveZeroWheelAngle;
import frc.robot.commands.TurnUsingPose_VisionProof;
import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Vision.Vision;
// import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final EndEffector m_endEffector = new EndEffector(); 
  private final Swerve m_swerveDrive = Swerve.getInstance();
  private final Vision vision = Vision.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_swerveDrive.setDefaultCommand(new SwerveDriveXbox(m_swerveDrive));
    ZeroGyro zeroGyro = new ZeroGyro(m_swerveDrive);
    zeroGyro.schedule();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // cancelling on release.
    Intake intakeCommand = new Intake(m_endEffector);
    m_driverController.a().whileTrue(intakeCommand);

    // Calls the command ZeroGyro when the Startbutton on the drivers controller is pressed
    ZeroGyro zeroGyro = new ZeroGyro(m_swerveDrive);
    m_driverController.start().whileTrue(zeroGyro);

    // Calls the command turnUsingPose when the rightbumper on the drivers controller is pressed
    TurnUsingPose_VisionProof turnUsingPose = new TurnUsingPose_VisionProof(m_swerveDrive);
    m_driverController.rightBumper().whileTrue(turnUsingPose);
 
    // Calls the command goToPointUsingPose when the b button on the drivers controller is pressed
    GoToPointUsingPose_VisionProof goToPointUsingPose = new GoToPointUsingPose_VisionProof(m_swerveDrive);
    m_driverController.b().whileTrue(goToPointUsingPose);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
