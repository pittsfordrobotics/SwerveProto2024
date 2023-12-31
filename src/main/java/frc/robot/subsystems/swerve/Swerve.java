// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.nio.file.attribute.PosixFileAttributes;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDriveXbox;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Swerve extends SubsystemBase {

  // Defining swerve modules
  private SwerveModuleIO moduleFL;
  private SwerveModuleIO moduleFR;
  private SwerveModuleIO moduleBL;
  private SwerveModuleIO moduleBR;

  private final SwerveModuleIO[] moduleIO;
  private final SwerveModuleState[] lastModuleStates = new SwerveModuleState[4];
  private ChassisSpeeds actualRobotRelativeChassisSpeeds;
  private ChassisSpeeds targetFieldRelativeSpeeds;
  private ChassisSpeeds targetRobotRelativeChassisSpeeds;

  private final SwerveDrivePoseEstimator poseEstimator;
  // private final SwerveDrivePoseEstimator poseEstimator;

  // Initialize a PID controller for calculating the wanted angular velocity based on the desired angle
  PIDController SwerveTargetAngleVPID = new PIDController(SwerveConstants.AngV_P, SwerveConstants.AngV_I, SwerveConstants.AngV_D);

  SwerveModuleState[] wantedModuleStates = new SwerveModuleState[4];
  private final SwerveDriveKinematics kinematics = SwerveConstants.DRIVE_KINEMATICS;
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];


  private Rotation2d robotRelativeAngle = new Rotation2d();
  private Rotation2d targetAngle = new Rotation2d();
  private WPI_Pigeon2 pigeon= new WPI_Pigeon2(SwerveConstants.CAN_PIGEON);


  private final static Swerve INSTANCE = new Swerve();
  public static Swerve getInstance() {
    return INSTANCE;
  }

  /** Creates a new Swerve Drive object with 4 modules specified by SwerveConstants */
  public Swerve() {    
    moduleFL = new SwerveModuleIO(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, SwerveConstants.FL_OFFSET);
    moduleFR = new SwerveModuleIO(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, SwerveConstants.FR_OFFSET);
    moduleBL = new SwerveModuleIO(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, SwerveConstants.BL_OFFSET);
    moduleBR = new SwerveModuleIO(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, SwerveConstants.BR_OFFSET);

    moduleIO = new SwerveModuleIO[]{moduleFL, moduleFR, moduleBL, moduleBR}; // initializes motors and encoders for all 4 swerve modules.
    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      modulePositions[i] = new SwerveModulePosition(moduleIO[i].drivePositionMeters, Rotation2d.fromRadians(moduleIO[i].steerOffsetAbsolutePositionRad));
      lastModuleStates[i] = new SwerveModuleState();
    }

    // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive_path_planner, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
    );




    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRobotRelativeAngle(), modulePositions, new Pose2d(), VecBuilder.fill(0.003, 0.003, 0.0002), VecBuilder.fill(0.9, 0.9, 0.9));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the swerve module states
    for (int i = 0; i < 4; i++) {
      moduleIO[i].updateInputs();
      modulePositions[i] = new SwerveModulePosition(moduleIO[i].drivePositionMeters, Rotation2d.fromRadians(moduleIO[i].steerOffsetAbsolutePositionRad));
    }
    
    // Update the pose estimator
    poseEstimator.update(getRobotRelativeAngle(), modulePositions);
    SmartDashboard.putNumber("Robot Pose X", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    SmartDashboard.putNumber("Robot Pose Theta", getPose().getRotation().getDegrees());
   }

   /**Gets the robot's current orientation. Returns the CCW+ angle in a Rotation2d object. */
  private Rotation2d getRobotRelativeAngle(){
    double robotRelativeAngleDeg = pigeon.getYaw();
    return Rotation2d.fromRadians(MathUtil.angleModulus(Math.toRadians(robotRelativeAngleDeg)));
  }

  /** <p>Updates swerve module inputs for all swerve modules.</p> 
   *  <p>Performs inverse kinematics to create target swerve module states from controller inputs</p>
   *  <p>All controller inputs should be between -1 and 1</p>
   * @param xAxis Translation x-axis input (left stick to left)
   * @param yAxis Translation y-axis input (left stick up)
   * @param rotateX Rotation x-axis input (right stick left)
   * @param rotateY Rotation y-axis input (right stick up)
  */
  public void updateSwerveModuleStates(double xAxis, double yAxis, Rotation2d TargetAngle) {  
    this.targetAngle = TargetAngle;
    updateSwerveModuleStates(xAxis, yAxis);
  };


  public void updateSwerveModuleStates(double xAxis, double yAxis) {
    // Set X and Y speeds based on max motor RPM.
    double targetSpeedX = xAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
    double targetSpeedY = yAxis * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND;
    SwerveModuleState[] actualStates = getactualStates();

    // New code
    // Decides how to calculate the target angular velocity based on the controller inputs (2 variable booleons and a constant boolean)
    double targetAngularVelocity = 0;
    // Logging
    SmartDashboard.putNumber("Target Angular Velocity", targetAngularVelocity);
    SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
    SmartDashboard.putNumber("Robot Gyro Theta", robotRelativeAngle.getDegrees());

    // // Just RightJoystick Code but with PID
    SwerveTargetAngleVPID.enableContinuousInput(-Math.PI, Math.PI);
    SwerveTargetAngleVPID.setTolerance(1/120);
    targetAngularVelocity = SwerveTargetAngleVPID.calculate(robotRelativeAngle.getRadians(), targetAngle.getRadians()); // Target angular velocity is a linear function of input, with a steep slope.
    targetAngularVelocity = MathUtil.clamp(targetAngularVelocity, -SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Clamp the angular velocity at the max allowed value.

    actualRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(actualStates);
    ChassisSpeeds.fromFieldRelativeSpeeds(actualRobotRelativeChassisSpeeds, new Rotation2d().minus(robotRelativeAngle));
    targetFieldRelativeSpeeds = new ChassisSpeeds(targetSpeedX, targetSpeedY, targetAngularVelocity);
    targetRobotRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldRelativeSpeeds, robotRelativeAngle); // Convert target field relative speeds into chassis speeds
    wantedModuleStates = kinematics.toSwerveModuleStates(targetRobotRelativeChassisSpeeds); // Use inverse kinematics to get target swerve module states.
    SwerveDriveKinematics.desaturateWheelSpeeds(wantedModuleStates, SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
    for (int i = 0; i < 4; i++) {
      wantedModuleStates[i] = SwerveModuleState.optimize(wantedModuleStates[i], actualStates[i].angle);
    }
    
  };

  /** Used for testing alignment, stops all modules and points them forward */
  public void driveZeroOffset() {
    for (int i = 0; i < 4; i++) {
      wantedModuleStates[i] = new SwerveModuleState(0, new Rotation2d());
      moduleIO[i].drive(wantedModuleStates[i], false);
    }
  }

  /** Drive all swerve modules */
  public void drive() {
    for (int i = 0; i < 4; i++) {
      moduleIO[i].drive(wantedModuleStates[i], false);
    }
  }

  public void drive_path_planner(ChassisSpeeds RobotRelativeChassisSpeeds) {
    SwerveModuleState[] actualStates = getactualStates();

    wantedModuleStates = kinematics.toSwerveModuleStates(RobotRelativeChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(wantedModuleStates,
        SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
    for (int i = 0; i < 4; i++) {
      wantedModuleStates[i] = SwerveModuleState.optimize(wantedModuleStates[i], actualStates[i].angle);
    }

    drive();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] actualStates = getactualStates();
    actualRobotRelativeChassisSpeeds = kinematics.toChassisSpeeds(actualStates);
    return actualRobotRelativeChassisSpeeds;
  }

public SwerveModuleState[] getactualStates() {
  SwerveModuleState[] actualStates = new SwerveModuleState[4];
  for (int i = 0; i < 4; i++) {
    moduleIO[i].updateInputs();
    actualStates[i] = new SwerveModuleState(moduleIO[i].driveVelocityMetersPerSec,
        Rotation2d.fromRadians(moduleIO[i].steerAbsolutePositionRad));
  }
  return actualStates;
}

  /** Reset the pigeon angle */
  public void zeroGyro() {
    pigeon.setYaw(0);
    // tells the pose the gyro was reset
    Pose2d pose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
    poseEstimator.resetPosition(getRobotRelativeAngle(), modulePositions, pose);

  }

  /** Reset the pose estimator, input is desired pose */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getRobotRelativeAngle(), modulePositions, pose);
  }

  /** Get the robot's current pose */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Adds vision data to the swerve pose estimator -- called in vision subsystem
  public void addVisionData(Pose2d pose, double time, Matrix<N3, N1> vec) {
    poseEstimator.addVisionMeasurement(pose, time, vec);
  }
}
