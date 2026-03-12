// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer;

public class ManualShoot extends Command {
  /** Creates a new ManualShoot. */
  private final Indexer indexer = new Indexer();
  private final Turret turret = new Turret();
  private final Indexer indexer = new Indexer();
  private final Intake intake = new Intake();

  private final BooleanSupplier shootButton;
  private final double degrees;
  private final double shootVelocity;
  private final double funnelVelocity;
  private final double intakeVelocity;
  private final double tolerance;
  
  public ManualShoot(BooleanSupplier shootButton, double degrees, double shootVelocity, double funnelVelocity, double indexerVelocity, double intakeVelocity, double tolerance) {
    this.shootButton = shootButton;
    this.degrees = degrees;
    this.shootVelocity = shootVelocity;
    this.funnelVelocity = funnelVelocity;
    this.indexerVelocity = indexerVelocity;
    this.intakeVelocity = intakeVelocity;
    this.tolerance = tolerance;
    
    addRequirements(indexer, turret, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSolenoid(true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shootButton.getAsBoolean()) {
      turret.setTurretAngleDegrees(degrees);

      turret.rotateToVelocity(shootVelocity);
      turret.rotateFunnelToVelocity(funnelVelocity);

      if(turret.shooterAtVelocity(shootVelocity, tolerance) && turret.funnelAtVelocity(funnelVelocity, tolerance)){
        indexer.rotateToVelocity(indexerVelocity);
        intake.rotateToVelocity(intakeVelocity);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
