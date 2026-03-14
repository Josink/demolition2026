// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFX intakeMotor = new TalonFX(Constants.intakeConstants.intakeMotorID, "4998Canivore");

  private final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2,PneumaticsModuleType.REVPH, 0, 1);

  public Intake() {
    applyIntakeMotorConfigs();
    compressor.enableDigital();
    //setIntakeSolenoid(true, false);
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

  public void up(){
    intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void down(){
    intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void off(){
    intakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void setIntakeSolenoid(boolean down, boolean up){
    if(down){
      intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (up){
      intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else{
      intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
  }

  public void rotateToVelocity(double velocity){
    final MotionMagicVelocityTorqueCurrentFOC request =  new MotionMagicVelocityTorqueCurrentFOC(velocity);
    intakeMotor.setControl(request);
  }

  public void manualControl(BooleanSupplier down, BooleanSupplier up){
    setIntakeSolenoid(down.getAsBoolean(), up.getAsBoolean());
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
}
