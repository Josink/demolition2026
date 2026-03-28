// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private BooleanSupplier leftTrigger;
  private BooleanSupplier rightTrigger;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;

  private double shooterVelocity;
  private double funnelVelocity;

  private double intakeRotateVelocity;
  
  public AutoPlay(Indexer indexer, Turret turret, Intake intake, 
                  Vision vision, BooleanSupplier leftTrigger, 
                  BooleanSupplier rightTrigger, BooleanSupplier leftBumper, BooleanSupplier rightBumper, 
                  double tolerance, double indexerVelocity, double intakeVelocity,
                  double lowIndexerVelocity, double intakeRotateVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    this.tolerance = tolerance;
    this.indexerVelocity = indexerVelocity;
    this.intakeVelocity = intakeVelocity;
    this.lowIndexerVelocity = lowIndexerVelocity;
    this.intakeRotateVelocity = intakeRotateVelocity;

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
    double minShooterVelocity = 55; 

    Distance dist = vision.getDistanceToHub();
    if (dist != null) {
        shooterVelocity = 4.85 * dist.in(Meters) + 35.5;
        funnelVelocity = shooterVelocity * 0.8;
        SmartDashboard.putNumber("DistanceToHub (m)", dist.in(Meters));

    } else {
        shooterVelocity = minShooterVelocity;
        funnelVelocity = shooterVelocity;
    }

    vision.getAngleToHub(shooterVelocity).ifPresent(angle -> {
        turret.setTurretAngleDegrees(angle.in(Degrees)); 
        SmartDashboard.putNumber("AngleTOHub", angle.in(Degrees));
    });

    if(rightTrigger.getAsBoolean()) {
      runShooterSequence(shooterVelocity, funnelVelocity, lowIndexerVelocity,
      indexerVelocity, tolerance, indexer, intake);
    } else if(leftTrigger.getAsBoolean()){
      intake.rotateToVelocity(-intakeVelocity);
    } else {
      turret.stopShooter(0);
      turret.stopFunnel(0);
      intake.stopIntake();
      indexer.stopIndexer();
    }

    if(leftBumper.getAsBoolean()){
      intake.down(intakeRotateVelocity);
    } else if(rightBumper.getAsBoolean()){
      intake.up(-intakeRotateVelocity* 2.5);
    } else{
      intake.off();
    }
  }

  public void runShooterSequence(
    double shooterVel,
    double funnelVel,
    double lowIndexerVel,
    double indexerVel,
    double tolerance,
    Indexer indexer,
    Intake intake
) {
    turret.rotateToVelocity(shooterVel);
    indexer.rotateToVelocity(lowIndexerVel);

    if (turret.shooterAtVelocity(shooterVel, tolerance)) {
        indexer.rotateToVelocity(indexerVel);
        turret.rotateFunnelToVelocity(funnelVel);
    }
}

  @Override
  public void end(boolean interrupted) {
    turret.stopShooter(0);
    turret.stopFunnel(0);
    intake.stopIntake();
    indexer.stopIndexer();
    turret.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
