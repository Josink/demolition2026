// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  private static final String LL_FRONT = "limelight-front";  
  private static final String LL_LEFT  = "limelight-left";   
  private static final String LL_RIGHT = "limelight-right";

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    PoseEstimate estimate = getBestPoseEstimate();
  
    if (!isValid(estimate)) return;
  
    // Reject bad data
    if (estimate.tagCount < 1) return;
  
    // Reject if robot rotating too fast (optional but recommended)
    if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 3.0)
        return;
  
    // Dynamically adjust trust
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

    PoseEstimate front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_FRONT);
    PoseEstimate left  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_LEFT);
    PoseEstimate right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_RIGHT);

    // Pick the camera with most tags
    PoseEstimate best = null;

    if (isValid(front)) best = front;
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
}
