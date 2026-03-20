// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new AutoPlay. */
  private final indexer indexer;
  private final Turret turret;
  private final intake intake;
  private final Vision vision;

  private final double tolerance;
  private final double indexerVelocity;
  private final double lowIndexerVelocity;
  private final double bIntakeVelocity;
  
  public Shoot(indexer indexer, Turret turret, intake intake, 
                  Vision vision, double tolerance, double indexerVelocity,
                  double lowIndexerVelocity, double bIntakeVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    this.tolerance = tolerance;
    this.indexerVelocity = indexerVelocity;
    this.lowIndexerVelocity = lowIndexerVelocity;
    this.bIntakeVelocity = bIntakeVelocity;
    
    addRequirements(indexer, turret, intake, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxShooterVelocity = 90;
    double minShooterVelocity = 50; 

    double shooterVelocity = (2 * minShooterVelocity + maxShooterVelocity);
    double funnelVelocity = shooterVelocity * 0.8;

    double turretAngle = vision.getAngleToHub(shooterVelocity).in(Degrees);
    turret.setTurretAngleDegrees(turretAngle);

    turret.rotateToVelocity(shooterVelocity);
    turret.rotateFunnelToVelocity(funnelVelocity);
    indexer.rotateToVelocity(lowIndexerVelocity);
    intake.rotateToVelocity(bIntakeVelocity);

    if(turret.shooterAtVelocity(shooterVelocity, tolerance) && turret.funnelAtVelocity(funnelVelocity, tolerance)){
      indexer.rotateToVelocity(indexerVelocity);
      intake.rotateToVelocity(bIntakeVelocity);
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
