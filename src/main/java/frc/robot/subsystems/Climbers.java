// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase {
  /** Creates a new Climb. */
   private TalonFX climbMotor = new TalonFX(6);

  public void runIntake() {
    climbMotor.set(0.);
  }

  public void stopIntake() {
    climbMotor.set(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
