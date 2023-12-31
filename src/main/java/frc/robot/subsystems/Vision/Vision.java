// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionIO.Pipelines;
import frc.robot.subsystems.swerve.Swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {

    // Initialization
    private final VisionIO[] io;

    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    private static final Vision INSTANCE = new Vision(VisionConstants.LIMELIGHT1);

    public static Vision getInstance() {
        return INSTANCE;
    }

    private Vision(VisionIO ioLimelight1) {
        io = new VisionIO[] { ioLimelight1 };
        FieldConstants.aprilTags.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, 0.0));
    }

    // To add more cameras:
    // Update VisionConstants.java to include the new camera.
    // Update the following two lines to include the camera.
    private final VisionIO.VisionIOInputs[] inputs = new VisionIO.VisionIOInputs[] { new VisionIO.VisionIOInputs() };
    private final String[] camNames = new String[] { VisionConstants.LIMELIGHT1_NAME };
    
    private Pipelines pipeline = Pipelines.Test; // default pipeline

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            // update the inputs from the netwrork tables named camNames[i]
            io[i].updateInputs(inputs[i], camNames[i]);

            // keeps the pipeline always the same
            io[i].setPipeline(pipeline, camNames[i]);
        }
        List<Pose2d> allRobotPoses = new ArrayList<>();

        // Pose estimation
        if (!DriverStation.isAutonomous()) {
            for (int i = 0; i < io.length; i++) {
                // exit if data is bad
                if (Arrays.equals(inputs[i].botXYZ, new double[] { 0.0, 0.0, 0.0 }) || inputs[i].botXYZ.length == 0
                        || !inputs[i].connected) {
                    continue;
                }

                // Gets robot pose from the current camera
                Pose3d robotPose3d = new Pose3d(inputs[i].botXYZ[0], inputs[i].botXYZ[1], inputs[i].botXYZ[2],
                    new Rotation3d(
                        Math.toRadians(inputs[i].botRPY[0]),
                        Math.toRadians(inputs[i].botRPY[1]),
                        Math.toRadians(inputs[i].botRPY[2])
                    )
                );
                Pose2d robotPose = robotPose3d.toPose2d();
                //Pose2d robotPose = new Pose2d(inputs[i].botXYZ[0], inputs[i].botXYZ[1],  Rotation2d.fromDegrees(inputs[i].botRPY[2]));

                SmartDashboard.putBoolean("Vision Not exited?", true);
                SmartDashboard.putNumber("Vision/Pose" + i + "/X", robotPose.getX());
                SmartDashboard.putNumber("Vision/Pose" + i + "/Y", robotPose.getY());
                SmartDashboard.putNumber("Vision/Pose" + i + "/Theta", robotPose.getRotation().getDegrees());


                // exit if off the field (fix later)
                if (robotPose3d.getX() < -VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getY() < -VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN
                        || robotPose3d.getZ() < -VisionConstants.Z_MARGIN
                        || robotPose3d.getZ() > VisionConstants.Z_MARGIN) {
                    continue;
                }





                // Get tag poses and update last detection times
                List<Pose3d> tagPoses = new ArrayList<>();
                for (int z = 0; z < inputs[i].tagIDs.length; z++) {
                    int tagId = (int) inputs[i].tagIDs[z];
                    lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                    Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) inputs[i].tagIDs[z]);
                    tagPose.ifPresent(tagPoses::add);
                }

                // Calculate average distance to tag
                double totalDistance = 0.0;
                Pose2d[] tagPoses2d = new Pose2d[tagPoses.size()];
                int num = 0;
                for (Pose3d tagPose : tagPoses) {
                    tagPose = FieldConstants.allianceFlipper(tagPose, DriverStation.getAlliance());
                    totalDistance += tagPose.getTranslation().getDistance(robotPose3d.getTranslation());
                    tagPoses2d[num] = tagPose.toPose2d();
                    num++;
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Calculate standard deviation to give to the .addVisionData() swerve method
                // The larger the STD the less the data is trusted, here the STD is proportional
                // to the distance to the tag
                // Increase VisionConstants.XY_STD_DEV_COEF and
                // VisionConstants.THETA_STD_DEV_COEF to trust vision in general less
                double xyStdDev = VisionConstants.XY_STD_DEV_COEF * Math.pow(avgDistance, 2.0) / tagPoses.size();
                double thetaStdDev = VisionConstants.THETA_STD_DEV_COEF * Math.pow(avgDistance, 2.0) / tagPoses.size();
                // SmartDashboard.putNumber("Vision/XYstd", xyStdDev);
                // SmartDashboard.putNumber("Vision/ThetaStd", thetaStdDev);

                // Add vision data to swerve pose estimator
                Swerve.getInstance().addVisionData(robotPose, inputs[i].captureTimestamp,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

                // Add robot pose from this camera to a list of all robot poses
                allRobotPoses.add(robotPose);
                List<Pose3d> allTagPoses = new ArrayList<>();
                for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
                    if (Timer.getFPGATimestamp() - detectionEntry.getValue() < VisionConstants.TARGET_LOG_SECONDS
                            && FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).isPresent()) {
                        allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
                    }
                }
            }
        }
        // SmartDashboard.putNumber("Vision/NumPoses", allRobotPoses.size());
        // SmartDashboard.putNumberArray("Vision/NumTags", (Double[]) allRobotPoses.toArray());
    }
}
