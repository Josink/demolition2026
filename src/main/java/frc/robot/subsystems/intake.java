// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFX intakeMotor = new TalonFX(1);
  public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX TalonFX = new TalonFX(2);

  public void runIntake() {
    TalonFX.set(0.9);
  }

  public void stopIntake() {
    TalonFX.set(0.0);
  }
}
  public intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
