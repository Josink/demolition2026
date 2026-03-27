// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFX intakeMotor = new TalonFX(Constants.intakeConstants.intakeMotorID, "4998Canivore");
  private TalonFX intakeRotateMotor = new TalonFX(Constants.intakeConstants.intakeRotateMotorID, "4998Canivore");

  private DutyCycleOut intake = new DutyCycleOut(0);
  private DutyCycleOut intakeRotate = new DutyCycleOut(0);

  public Intake() {
    applyIntakeMotorConfigs();
    applyIntakeRotateMotorConfigs();
    intakeRotateMotor.setPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void suck(double speed){
    intakeMotor.set(speed);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void rotateToVelocity(double velocity){
    intake.Output = velocity;
    intake.EnableFOC = true;
    intakeMotor.setControl(intake);
  }

  public void up(double speed){
    intakeRotate.Output = speed;
    intakeRotate.EnableFOC = true;
    intakeRotateMotor.setControl(intakeRotate);
  }

  public void down(double speed){
    intakeRotate.Output = speed;
    intakeRotate.EnableFOC = true;
    intakeRotateMotor.setControl(intakeRotate);
  }

  public void off(){
    intakeRotateMotor.set(0);
  }

  private void applyIntakeMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration();

    talonconfigs.Slot0.kP = Constants.intakeConstants.kP;
    talonconfigs.Slot0.kI = Constants.intakeConstants.kI;
    talonconfigs.Slot0.kD = Constants.intakeConstants.kD;
    talonconfigs.Slot0.kV = Constants.intakeConstants.kv;
    talonconfigs.Slot0.kS = Constants.intakeConstants.ks;
    talonconfigs.Slot0.kA = Constants.intakeConstants.ka;

    var motionMagicConfigs = talonconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.intakeConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.intakeConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.intakeConstants.MotionMagicJerk;

    talonconfigs.Feedback.FeedbackRemoteSensorID = intakeMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    intakeMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(motorOutputConfigs);
  }

  private void applyIntakeRotateMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration();

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonconfigs.SoftwareLimitSwitch;
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = false;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = Constants.intakeConstants.ForwardSoftLimitThreshold;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = false;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = Constants.intakeConstants.ReverseSoftLimitThreshold;

    talonconfigs.Feedback.FeedbackRemoteSensorID = intakeRotateMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonconfigs.Feedback.SensorToMechanismRatio = Constants.intakeConstants.intakeRotateMotorSensorToMechanismRatio;

    intakeRotateMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(motorOutputConfigs);
  }
}
