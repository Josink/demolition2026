// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  //private static final String LL_FRONT = "limelight-front";  
  private static final String LL_LEFT  = "limelight-left";   
  private static final String LL_RIGHT = "limelight-right";

  int[] validIDs = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 20, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

  Pose2d hubAprilTag;

  public Optional<Alliance> alliance;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", validIDs);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", validIDs);
  }

  @Override
  public void periodic() {
    alliance = DriverStation.getAlliance();
    LimelightHelpers.SetRobotOrientation("limelight-left", drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", drivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
    PoseEstimate estimate = getBestPoseEstimate();
    
    if (!isValid(estimate)) return;

    if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 3.0)
        return;

    var stdDevs = estimate.tagCount >= 2
        ? VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(3))
        : VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(10));

    drivetrain.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        stdDevs
    );
  }

  public PoseEstimate getBestPoseEstimate() {
    PoseEstimate left;
    PoseEstimate right;
    
    if(alliance.isPresent() && alliance.get() == Alliance.Blue) {
      left  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_LEFT);
      right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_RIGHT);
      hubAprilTag = Constants.fieldConstants.aprilTagIDToPose(10);
    } else {
      left  = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LL_LEFT);
      right = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(LL_RIGHT);
      hubAprilTag = Constants.fieldConstants.aprilTagIDToPose(26);
    }

    // Pick the camera with most tags
    PoseEstimate best = null;

    if (isValid(left)  && (best == null || left.tagCount  > best.tagCount)) best = left;
    if (isValid(right) && (best == null || right.tagCount > best.tagCount)) best = right;

    return best;
  }

  private boolean isValid(PoseEstimate pose) {
    return pose != null
        && pose.tagCount > 0
        && pose.pose.getX() > 0.0
        && pose.pose.getX() < 17.0
        && pose.pose.getY() > 0.0
        && pose.pose.getY() < 9.0;
  }

  public Pose2d getRobotPose() {
    PoseEstimate est = getBestPoseEstimate();
    return est != null ? est.pose : null;
  }

  public Distance getDistanceToHub() {
    PoseEstimate est = getBestPoseEstimate();
    if (est == null) return null;

    double distanceToHub = est.pose.getTranslation().getDistance(hubAprilTag.getTranslation());
    return Meters.of(distanceToHub);
  }

  public Angle getAngleToHub() {
    PoseEstimate est = getBestPoseEstimate();
    if (est == null) return null;

    double angleToHub = est.pose.getRotation().minus(hubAprilTag.getRotation()).getDegrees();
    return Degree.of(angleToHub);
  }
}
