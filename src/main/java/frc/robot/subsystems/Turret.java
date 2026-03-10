// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private TalonFX turretMotor = new TalonFX(Constants.turretConstants.TurretMotorID,"4998Canivore");
  private CANcoder turretEncoder = new CANcoder(Constants.turretConstants.TurretEncoderID, "4998Canivore");
  private TalonFX lShootingMotor = new TalonFX(Constants.turretConstants.lShootingMotorID, "4998Canivore");
  private TalonFX rShootingMotor = new TalonFX(Constants.turretConstants.rShootingMotorID,"4998Canivore");
  private TalonFX funnelMotor = new TalonFX(Constants.turretConstants.funnelMotorID, "4998Canivore");

  private final SysIdRoutine shooterSysId;
  private final SysIdRoutine funnelSysId;

  private final MotionMagicTorqueCurrentFOC positionRequest = new MotionMagicTorqueCurrentFOC(0);

  final DutyCycleOut rotate = new DutyCycleOut(0);

  public Turret() {
    applyTurretMotorConfigs();
    applyShootingMotorConfigs();

    lShootingMotor.setControl(new Follower(Constants.turretConstants.rShootingMotorID, MotorAlignmentValue.Opposed));

    funnelSysId = createFunnelSysId();
    shooterSysId = createFunnelSysId();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Position", getTurretPosition());
    SmartDashboard.putNumber("Shooter Velocity", rShootingMotor.getVelocity().getValueAsDouble());
  }

  public double getTurretPosition(){
    return turretMotor.getPosition().getValueAsDouble();
  }

  public void setTurretPosition(double position){
    turretEncoder.setPosition(position);
  }

  public void resetTurretPosition(){
    turretEncoder.setPosition(0);
  }
  
  public void stopShooter(double speed){
    rShootingMotor.set(speed);
  }

  public void stopFunnel(double speed){
    funnelMotor.set(speed);
  }

  public void rotateTurret(double duty){
    rotate.Output = duty;
    rotate.EnableFOC = true;
    turretMotor.setControl(rotate);
  }

  public void stopTurret(){
    turretMotor.set(0);
  }

  public void rotateToPos(double position){
    positionRequest.Position = position;
    turretMotor.setControl(positionRequest);
  }

  public void setTurretAngleDegrees(double degrees) {
    Rotation2d target = Rotation2d.fromDegrees(degrees);
    double rotations = target.getRotations();
    rotateToPos(rotations);
  }

  public void rotateToVelocity(double velocity){
    final MotionMagicVelocityTorqueCurrentFOC request =  new MotionMagicVelocityTorqueCurrentFOC(velocity);
    rShootingMotor.setControl(request);
  }

  public void rotateFunnelToVelocity(double Velocity){
    final MotionMagicVelocityTorqueCurrentFOC request = new MotionMagicVelocityTorqueCurrentFOC(Velocity);
    funnelMotor.setControl(request);
  }

  public void manualControl(boolean rotate, double tVelocity, BooleanSupplier toPos, double pos,  BooleanSupplier toShoot, double sVelocity, double fVelocity){
   if (rotate){
      rotateTurret(tVelocity);
    } else if (toPos.getAsBoolean()){
      rotateToPos(pos);
    } else if (!toPos.getAsBoolean()) {
    stopTurret();
}
    
    if(toShoot.getAsBoolean()){
      rotateToVelocity(sVelocity);
      rotateFunnelToVelocity(fVelocity);
    } else {
      stopShooter(0);
      stopFunnel(0);
    }
  }

  public SysIdRoutine createFunnelSysId(){
    SysIdRoutine.Config config = new SysIdRoutine.Config();

    SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
      volts -> funnelMotor.setControl(new VoltageOut(volts.in(Volts))),
      log -> log.motor("funnelMotor")
          .voltage(Volts.of(funnelMotor.getMotorVoltage().getValueAsDouble()))
          .angularVelocity(funnelMotor.getVelocity().getValue())
          .angularPosition(funnelMotor.getPosition().getValue()),
      this
        );

    return new SysIdRoutine(config, mechanism);
  }

  public SysIdRoutine createShooterSysId(){
    SysIdRoutine.Config config = new SysIdRoutine.Config();

    SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
      volts -> rShootingMotor.setControl(new VoltageOut(volts.in(Volts))),
      log -> log.motor("rShootingMotor")
          .voltage(Volts.of(rShootingMotor.getMotorVoltage().getValueAsDouble()))
          .angularVelocity(rShootingMotor.getVelocity().getValue())
          .angularPosition(rShootingMotor.getPosition().getValue()),
      this
        );

    return new SysIdRoutine(config, mechanism);
  }

  public Command sysIdFunnelQuasistaticForward() {
    return funnelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFunnelQuasistaticReverse() {
    return funnelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFunnelDynamicForward() {
    return funnelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFunnelDynamicReverse() {
    return funnelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  
  public Command sysIdQuasistaticForward() {
    return shooterSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticReverse() {
    return shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    return shooterSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicReverse() {
    return shooterSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  private void applyTurretMotorConfigs(){
    TalonFXConfiguration talonconfigs = new TalonFXConfiguration(); 

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
    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonconfigs.Feedback.SensorToMechanismRatio = Constants.turretConstants.SensorToMechanismRatio;
   
    turretMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

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

    talonconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
   
    lShootingMotor.getConfigurator().apply(talonconfigs);
    rShootingMotor.getConfigurator().apply(talonconfigs);
    funnelMotor.getConfigurator().apply(talonconfigs);

    MotorOutputConfigs rmotorOutputConfigs = new MotorOutputConfigs();
    rmotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rmotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    MotorOutputConfigs lmotorOutputConfigs = new MotorOutputConfigs();
    lmotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    lmotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    lShootingMotor.getConfigurator().apply(lmotorOutputConfigs);
    funnelMotor.getConfigurator().apply(lmotorOutputConfigs); 
    rShootingMotor.getConfigurator().apply(rmotorOutputConfigs);
  }

}

 
