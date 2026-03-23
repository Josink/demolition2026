// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPlay extends Command {
  /** Creates a new AutoPlay. */
  private final Indexer indexer;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;

  private final double tolerance;
  private final double indexerVelocity;
  private final double intakeVelocity;
  private final double lowIndexerVelocity;
  private final double bIntakeVelocity;
  private BooleanSupplier leftTrigger;
  private BooleanSupplier rightTrigger;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;

  private double shooterVelocity;
  private double funnelVelocity;
  
  public AutoPlay(Indexer indexer, Turret turret, Intake intake, 
                  Vision vision, BooleanSupplier leftTrigger, 
                      BooleanSupplier rightTrigger, BooleanSupplier leftBumper, BooleanSupplier rightBumper, 
                  double tolerance, double indexerVelocity, double intakeVelocity,
                  double lowIndexerVelocity, double bIntakeVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    this.tolerance = tolerance;
    this.indexerVelocity = indexerVelocity;
    this.intakeVelocity = intakeVelocity;
    this.lowIndexerVelocity = lowIndexerVelocity;
    this.bIntakeVelocity = bIntakeVelocity;

    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.leftBumper = leftBumper;
    this.rightBumper = rightBumper;
    
    addRequirements(indexer, turret, intake, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double maxShooterVelocity = 90;
    double minShooterVelocity = 55; 

    shooterVelocity = (2 * minShooterVelocity + maxShooterVelocity);
    funnelVelocity = shooterVelocity * 0.8;

    vision.getAngleToHub(shooterVelocity).ifPresent(angle -> {
        double turretAngle = angle.in(Degrees);
        turret.setTurretAngleDegrees(-turretAngle); 
    });

    if(rightTrigger.getAsBoolean()) {
      turret.rotateToVelocity(shooterVelocity);
      indexer.rotateToVelocity(lowIndexerVelocity);
      intake.rotateToVelocity(bIntakeVelocity);

      if(turret.shooterAtVelocity(shooterVelocity, tolerance) && turret.funnelAtVelocity(funnelVelocity, tolerance)){
        indexer.rotateToVelocity(indexerVelocity);
        turret.rotateFunnelToVelocity(funnelVelocity);
        intake.rotateToVelocity(bIntakeVelocity);
      }
    } else if(leftTrigger.getAsBoolean()){
      intake.rotateToVelocity(intakeVelocity);
    } else {
      turret.stopShooter(0);
      turret.stopFunnel(0);
      intake.stopIntake();
      indexer.stopIndexer();
    }

    if(leftBumper.getAsBoolean()){
      intake.down();
    } else if (rightBumper.getAsBoolean()){
      intake.up();
    } else{
      intake.off();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
