// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private TalonFX turretMotor = new TalonFX(Constants.turretConstants.TurretMotorID,"4998Canivore");
  private CANcoder turretEncoder = new CANcoder(Constants.turretConstants.TurretEncoderID, "4998Canivore");
  private TalonFX lShootingMotor = new TalonFX(Constants.turretConstants.lShootingMotorID, "4998Canivore");
  private TalonFX rShootingMotor = new TalonFX(Constants.turretConstants.rShootingMotorID,"4998Canivore");
  private TalonFX funnelMotor = new TalonFX(Constants.turretConstants.funnelMotorID, "4998Canivore");

  final DutyCycleOut rotate = new DutyCycleOut(0);

   public Turret() {
    applyTurretMotorConfigs();
    applyShootingMotorConfigs();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Position", getTurretPosition());
    SmartDashboard.putNumber("Shooter Velocity", rShootingMotor.getVelocity().getValueAsDouble());
  }

  public double getTurretPosition(){
    return turretEncoder.getPosition().getValueAsDouble();
  }

  public void setTurretPosition(double position){
    turretEncoder.setPosition(position);
  }

  public void resetTurretPosition(){
    turretEncoder.setPosition(0);
  }
  
  public void shoot(double speed){
    rShootingMotor.set(speed);
    lShootingMotor.set(speed);
    funnelMotor.set(speed);
  }

  public void rotateTurret(double duty){
    rotate.Output = duty;
    rotate.EnableFOC = true;
    turretMotor.setControl(rotate);
  }

  public void rotateToPos(double position){
    final MotionMagicTorqueCurrentFOC request =  new MotionMagicTorqueCurrentFOC(position);
    turretMotor.setControl(request);
  }

  public void rotateToVelocity(double velocity){
    final MotionMagicVelocityTorqueCurrentFOC request =  new MotionMagicVelocityTorqueCurrentFOC(velocity);
    rShootingMotor.setControl(request);
    lShootingMotor.setControl(request);
    funnelMotor.setControl(request);
  }

  public void manualControl(boolean rotate, double tVelocity, BooleanSupplier toPos, double pos,  BooleanSupplier toShoot, double velocity){
    if (rotate){
      rotateTurret(tVelocity);
    } else if (toPos.getAsBoolean()){
      rotateToPos(pos);
    }else {
      rotateTurret(0);
    }
    
    if(toShoot.getAsBoolean()){
      rotateToVelocity(velocity);
    } else {
      shoot(0);
    }
  }


  private void applyTurretMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration(); 

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = Constants.turretConstants.SensorToMechanismRatio;

    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonconfigs.SoftwareLimitSwitch;
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

    talonconfigs.Feedback.FeedbackRemoteSensorID = Constants.turretConstants.TurretEncoderID;
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
   
    turretMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    turretMotor.getConfigurator().apply(motorOutputConfigs);
  }

  private void applyShootingMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration(); 
    
    talonconfigs.Slot0.kP = Constants.turretConstants.SkP;
    talonconfigs.Slot0.kI = Constants.turretConstants.SkI;
    talonconfigs.Slot0.kD = Constants.turretConstants.SkD;
    talonconfigs.Slot0.kV = Constants.turretConstants.Skv;
    talonconfigs.Slot0.kS = Constants.turretConstants.Sks;
    talonconfigs.Slot0.kA = Constants.turretConstants.Ska;

    var motionMagicConfigs = talonconfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.turretConstants.SMotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.turretConstants.SMotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.turretConstants.SMotionMagicJerk;

    talonconfigs.Feedback.FeedbackRemoteSensorID = lShootingMotor.getDeviceID();
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
   
    lShootingMotor.getConfigurator().apply(talonconfigs);
    rShootingMotor.getConfigurator().apply(talonconfigs);
    funnelMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs rmotorOutputConfigs = new MotorOutputConfigs();
    rmotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rmotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    MotorOutputConfigs lmotorOutputConfigs = new MotorOutputConfigs();
    lmotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    lmotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    lShootingMotor.getConfigurator().apply(lmotorOutputConfigs);
    funnelMotor.getConfigurator().apply(lmotorOutputConfigs); 
    rShootingMotor.getConfigurator().apply(rmotorOutputConfigs);
  }

}

 
