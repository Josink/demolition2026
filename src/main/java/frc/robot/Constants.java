package frc.robot;

public class Constants {

    public static final class turretConstants {
        public static final int TurretMotorID = 50;
        public static final int rShootingMotorID = 51;
        public static final int lShootingMotorID = 52;
        public static final int TurretEncoderID = 0;

        public static final int SensorToMechanismRatio = 1;
        public static final int ForwardSoftLimitThreshold = 1000;
        public static final int ReverseSoftLimitThreshold = -1000;

        //slot 0 PID values TURRET MOTOR
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kv = 0.0;
        public static final double ks = 0.0;
        public static final double ka = 0.0;

        //motion magic values
        public static final int MotionMagicCruiseVelocity = 1500;
        public static final int MotionMagicAcceleration = 600;
        public static final int MotionMagicJerk = 1000;

        //slot 0 PID values SHOOTER MOTORS
        public static final double SkP = 0.1;
        public static final double SkI = 0.0;
        public static final double SkD = 0.0;
        public static final double Skv = 0.0;
        public static final double Sks = 0.0;
        public static final double Ska = 0.0;

        //motion magic values
        public static final int SMotionMagicCruiseVelocity = 1500;
        public static final int SMotionMagicAcceleration = 600;
        public static final int SMotionMagicJerk = 1000;

        //setpoints
        public static final int ScoreSetpoint = 50;
        public static final int shootSpeedSetPoint = 30;
    }

    public static final class intakeConstants {
        public static final int intakeMotorID = 4;

        //slot 0 PID values
        public static final double kP = 0.1;
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
    }

    public static final class indexerConstants {
        public static final int indexerMotorID = 5;
        public static final int mMotorID = 6;

        //slot 0 PID values
        public static final double kP = 0.1;
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

    


}
