// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  private static final String LL_LEFT  = "limelight-left";   
  private static final String LL_RIGHT = "limelight-right";

  int[] validIDs = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 20, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

  private Pose2d hubAprilTag;
  private PoseEstimate estimate;
  private Optional<Alliance> alliance = Optional.empty();

  // Throttle vision updates
  private double lastVisionUpdate = 0.0;

  // Cache robot heading to avoid redundant calls
  private double lastHeading = 0.0;

  // Cached vectors for standard deviations (Vector<N3>)
  private final Vector<N3> cachedStdDev2Tags = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(3));
  private final Vector<N3> cachedStdDev1Tag  = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(10));

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    LimelightHelpers.SetFiducialIDFiltersOverride(LL_LEFT, validIDs);
    LimelightHelpers.SetFiducialIDFiltersOverride(LL_RIGHT, validIDs);
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    // Only update vision every 0.05s (~20 Hz)
    if (now - lastVisionUpdate < 0.05) return;
    lastVisionUpdate = now;

    // Cache alliance
    if (alliance.isEmpty()) {
      alliance = DriverStation.getAlliance();
    }

    // Only update robot orientation if heading changed significantly
    double currentHeading = drivetrain.getPose().getRotation().getDegrees();
    if (Math.abs(currentHeading - lastHeading) > 0.1) {
      LimelightHelpers.SetRobotOrientation(LL_LEFT, currentHeading, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(LL_RIGHT, currentHeading, 0, 0, 0, 0, 0);
      lastHeading = currentHeading;
    }

    // Get the best pose estimate (from left or right Limelight)
    estimate = getBestPoseEstimate();
    if (!isValid(estimate)) return;

    // Skip if rotating too fast
    if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 3.0) return;

    // Use cached VecBuilder for standard deviations
    Vector<N3> stdDevs = estimate.tagCount >= 2 ? cachedStdDev2Tags : cachedStdDev1Tag;
    drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);
  }

  public PoseEstimate getBestPoseEstimate() {
    PoseEstimate left;
    PoseEstimate right;

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
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

  public Optional<Angle> getAngleToHub(double shotSpeed) {
    PoseEstimate est = getBestPoseEstimate();
    if (est == null) return Optional.empty();

    Pose2d robotPose = est.pose;

    double vx = drivetrain.getState().Speeds.vxMetersPerSecond;
    double vy = drivetrain.getState().Speeds.vyMetersPerSecond;

    double t = getShotTimeSeconds(shotSpeed);

    // Predict where robot will be when ball exits
    double futureX = robotPose.getX() + vx * t;
    double futureY = robotPose.getY() + vy * t;

    double dx = hubAprilTag.getX() - futureX;
    double dy = hubAprilTag.getY() - futureY;

    double angle = Math.toDegrees(Math.atan2(dy, dx));

    return Optional.of(Degree.of(angle));
  }

  public double getShotTimeSeconds(double shotSpeed) {
    Distance dist = getDistanceToHub();
    if (dist == null) return 0.0;

    double d = dist.in(Meters);
    return d / shotSpeed;
  }
}