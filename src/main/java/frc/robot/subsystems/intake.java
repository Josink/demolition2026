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

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFX intakeMotor = new TalonFX(Constants.intakeConstants.intakeMotorID);

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  public intake() {
    applyIntakeMotorConfigs();
    compressor.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void suck(double speed){
    intakeMotor.set(speed);
  }

  public void setIntakeSolenoid(boolean state){
    intakeSolenoid.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public void rotateToVelocity(double velocity){
    final MotionMagicVelocityTorqueCurrentFOC request =  new MotionMagicVelocityTorqueCurrentFOC(velocity);
    intakeMotor.setControl(request);
  }

  public void manualControl(BooleanSupplier down, boolean intake, double velocity){
    setIntakeSolenoid(down.getAsBoolean());

    if (intake){
      rotateToVelocity(velocity);
    } else {
      suck(0);
    }

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

    MotorOutputConfigs motorOutputConfigs = talonconfigs.MotorOutput;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(motorOutputConfigs);
  }
}
