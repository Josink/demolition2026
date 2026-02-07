// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private TalonFX turretMotor = new TalonFX(Constants.turretConstants.TurretMotorID);
  private TalonFX lShootingMotor = new TalonFX(Constants.turretConstants.lShootingMotorID);
  private TalonFX rShootingMotor = new TalonFX(Constants.turretConstants.rShootingMotorID);

  public Turret() {
    
  }

  public void spinTurret() {
    turretMotor.set(.8);
  }

  public void shootTurret() {
    lShootingMotor.set(.8);
    rShootingMotor.set(-0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
