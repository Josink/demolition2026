// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class indexer extends SubsystemBase {
  /** Creates a new indexer. */
  private TalonFX indexerMotor = new TalonFX(Constants.indexerConstants.indexerMotorID);
  
  public indexer() {
    applyIndexerMotorConfigs();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void applyIndexerMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration();

    talonconfigs.Slot0.kP = Constants.indexerConstants.kP;
    talonconfigs.Slot0.kI = Constants.indexerConstants.kI;
    talonconfigs.Slot0.kD = Constants.indexerConstants.kD;
    talonconfigs.Slot0.kV = Constants.indexerConstants.kv;
    talonconfigs.Slot0.kS = Constants.indexerConstants.ks;
    talonconfigs.Slot0.kA = Constants.indexerConstants.ka;

    var motionMagicConfigs = talonconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.indexerConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.indexerConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.indexerConstants.MotionMagicJerk;

    talonconfigs.Feedback.FeedbackRemoteSensorID = indexerMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    
    indexerMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = talonconfigs.MotorOutput;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    indexerMotor.getConfigurator().apply(motorOutputConfigs);

  }
}
