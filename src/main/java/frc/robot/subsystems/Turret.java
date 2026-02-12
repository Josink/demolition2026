// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private TalonFX turretMotor = new TalonFX(Constants.turretConstants.TurretMotorID);
  private TalonFX lShootingMotor = new TalonFX(Constants.turretConstants.lShootingMotorID);
  private TalonFX rShootingMotor = new TalonFX(Constants.turretConstants.rShootingMotorID);
  
   public Turret() {
    applyTurretMotorConfigs();
    applyShootingMotorConfigs();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getTurretPosition(){
    return turretMotor.getPosition().getValueAsDouble();
  }

  public void setTurretPosition(double position){
    turretMotor.setPosition(position);
  }

  public void resetTurretPosition(){
    turretMotor.setPosition(0);
  }

  public void spinTurret() {
    turretMotor.set(.8);
  }

  public void shootTurret() {
    lShootingMotor.set(.8);
    rShootingMotor.set(-0.8);
  }

  public void rotateToPos(double position){
    final MotionMagicTorqueCurrentFOC request =  new MotionMagicTorqueCurrentFOC(position);
    turretMotor.setControl(request);
  }

  private void applyTurretMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration(); 

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.turretConstants.SensorToMechanismRatio;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = Constants.turretConstants.ForwardSoftLimitThreshold;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = Constants.turretConstants.ReverseSoftLimitThreshold;

    talonconfigs.Slot0.kP = Constants.turretConstants.kP;
    talonconfigs.Slot0.kI = Constants.turretConstants.kI;
    talonconfigs.Slot0.kD = Constants.turretConstants.kD;
    talonconfigs.Slot0.kV = Constants.turretConstants.kv;
    talonconfigs.Slot0.kS = Constants.turretConstants.ks;
    talonconfigs.Slot0.kA = Constants.turretConstants.ka;

    var motionMagicConfigs = talonconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.turretConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.turretConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.turretConstants.MotionMagicJerk;

    talonconfigs.Feedback.FeedbackRemoteSensorID = turretMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
   
    turretMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    turretMotor.getConfigurator().apply(motorOutputConfigs);
  }


  public void rotateToVolocity(double velocity){
    final MotionMagicTorqueCurrentFOC request =  new MotionMagicTorqueCurrentFOC(velocity);
    rShootingMotor.setControl(request);
  }

  private void applyShootingMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration(); 

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.turretConstants.SensorToMechanismRatio;

    
    talonconfigs.Slot0.kP = Constants.turretConstants.kP;
    talonconfigs.Slot0.kI = Constants.turretConstants.kI;
    talonconfigs.Slot0.kD = Constants.turretConstants.kD;
    talonconfigs.Slot0.kV = Constants.turretConstants.kv;
    talonconfigs.Slot0.kS = Constants.turretConstants.ks;
    talonconfigs.Slot0.kA = Constants.turretConstants.ka;

    var motionMagicConfigs = talonconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.turretConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.turretConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.turretConstants.MotionMagicJerk;

    talonconfigs.Feedback.FeedbackRemoteSensorID = lShootingMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
   
    lShootingMotor.getConfigurator().apply(talonconfigs);
    rShootingMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs rmotorOutputConfigs = new MotorOutputConfigs();
    rmotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rmotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    MotorOutputConfigs lmotorOutputConfigs = new MotorOutputConfigs();
    lmotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    lmotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    lShootingMotor.getConfigurator().apply(rmotorOutputConfigs);
  }

}

 