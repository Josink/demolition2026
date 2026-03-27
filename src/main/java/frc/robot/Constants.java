package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static final class turretConstants {
        public static final int TurretMotorID = 50;
        public static final int rShootingMotorID = 51;
        public static final int lShootingMotorID = 52;
        public static final int funnelMotorID = 53;
        public static final int TurretEncoderID = 54;

        public static final int SensorToMechanismRatio = 48;
        public static final double ForwardSoftLimitThreshold = 0.25; //0
        public static final double ReverseSoftLimitThreshold = -0.5; //no more than one half rotation either way
        public static final double encoderOffset = -0.6533203125;

        //slot 0 PID values TURRET MOTOR
        public static final double kP = 43;
        public static final double kI = 15;
        public static final double kD = 15;
        public static final double kv = 0.0;
        public static final double ks = 0.1;
        public static final double ka = 0.0;

        //motion magic values
        public static final int MotionMagicCruiseVelocity = 1500;
        public static final int MotionMagicAcceleration = 600;
        public static final int MotionMagicJerk = 1000;

        //slot 0 PID values SHOOTER MOTORS
        public static final double SkP = 0.20399;
        public static final double SkI = 0;
        public static final double SkD = 0;
        public static final double Skv = 0.12018;
        public static final double Sks = 0.22694;
        public static final double Ska = 0.0052615;

        //slot 0 PID values FUNNEL MOTOR
        public static final double FkP = 0.32867;
        public static final double FkI = 0.15;
        public static final double FkD = 0.08;
        public static final double Fkv = 0.22169;
        public static final double Fks = 0.1826;
        public static final double Fka = 0.006896;

        //motion magic values
        public static final int SMotionMagicCruiseVelocity = 1500;
        public static final int SMotionMagicAcceleration = 600;
        public static final int SMotionMagicJerk = 1000;

        //setpoints
        public static final int ScoreSetpoint = 50;
        public static final int shootSpeedSetPoint = 30;
    }

    public static final class intakeConstants {
        public static final int intakeMotorID = 60;
        public static int intakeRotateMotorID = 61;

        //slot 0 PID values
        public static final double kP = 10;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kv = 0.0;
        public static final double ks = 0.0;
        public static final double ka = 0.0;

        //motion magic values
        public static final int MotionMagicCruiseVelocity = 1500;
        public static final int MotionMagicAcceleration = 600;
        public static final int MotionMagicJerk = 1000;

        //velocity setpoint
        public static final int intakeSpeedSetPoint = 30;
        
        public static final double intakeRotateMotorSensorToMechanismRatio = 25;
        public static final double ReverseSoftLimitThreshold = 0;
        public static final double ForwardSoftLimitThreshold = 0;
    }

    public static final class indexerConstants {
        public static final int indexerMotorID = 54;

        //slot 0 PID values
        public static final double kP = 10;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kv = 0.0;
        public static final double ks = 0.0;
        public static final double ka = 0.0;

        //motion magic values
        public static final int MotionMagicCruiseVelocity = 1500;
        public static final int MotionMagicAcceleration = 600;
        public static final int MotionMagicJerk = 1000;

        //velocity setpoint
        public static final int indexSpeedSetPoint = 30;

        
        //slot 0 PID values
        public static final double mkP = 0.1;
        public static final double mkI = 0.0;
        public static final double mkD = 0.0;
        public static final double mkv = 0.0;
        public static final double mks = 0.0;
        public static final double mka = 0.0;

        //motion magic values
        public static final int mMotionMagicCruiseVelocity = 1500;
        public static final int mMotionMagicAcceleration = 600;
        public static final int mMotionMagicJerk = 1000;

        //velocity setpoint
        public static final int mindexSpeedSetPoint = 30;

    }

    public static final class fieldConstants {
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static Pose2d aprilTagIDToPose(int id){
            return aprilTagFieldLayout.getTagPose(id).get().toPose2d();
        }

        public Translation2d getAprilTagTranslation(int id){
            return aprilTagFieldLayout.getTagPose(id).get().toPose2d().getTranslation();
        }
    }

}
