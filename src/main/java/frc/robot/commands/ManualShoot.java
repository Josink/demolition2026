// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class ManualShoot extends Command {
  /** Creates a new ManualShoot. */
  private final Indexer indexer = new Indexer();
  private final Turret turret = new Turret();
  private final Intake intake = new Intake();

  private final BooleanSupplier shootButton;

  public ManualShoot(BooleanSupplier shootButton) {
    this.shootButton = shootButton;
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
      turret.setTurretAngleDegrees(180);

      turret.rotateToVelocity(100);
      turret.rotateFunnelToVelocity(70);

      if(turret.shooterAtVelocity(100, 0.5) && turret.funnelAtVelocity(70, 0.5)){
        indexer.rotateToVelocity(-100);
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
