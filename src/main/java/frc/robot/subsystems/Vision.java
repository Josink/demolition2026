// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private static final String LL_FRONT = "limelight-front";  
  private static final String LL_LEFT  = "limelight-left";   
  private static final String LL_RIGHT = "limelight-right";

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
    return pose != null && pose.tagCount > 0;
  }

  public Pose2d getRobotPose() {
    PoseEstimate est = getBestPoseEstimate();
    return est != null ? est.pose : null;
  }
}
