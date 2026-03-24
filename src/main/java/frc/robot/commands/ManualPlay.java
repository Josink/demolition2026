// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

public class ManualPlay extends Command {
  /** Creates a new ManualShoot. */
  private final Indexer indexer;
  private final Turret turret;
  private final Intake intake;

  private final double shootVelocity;
  private final double funnelVelocity;
  private final double intakeVelocity;
  private final double bIntakeVelocity;
  private final double indexerVelocity;
  private final double lowIndexerVelocity;
  private final double tolerance;

  private BooleanSupplier leftTrigger;
  private BooleanSupplier rightTrigger;
  private BooleanSupplier leftBumper;
  private BooleanSupplier rightBumper;
  private BooleanSupplier x;
  private BooleanSupplier y;
  private BooleanSupplier a;
  private BooleanSupplier b;
  private DoubleSupplier leftX;
  
  public ManualPlay(Indexer indexer, Turret turret, Intake intake, BooleanSupplier leftTrigger, 
                      BooleanSupplier rightTrigger, BooleanSupplier leftBumper, BooleanSupplier rightBumper,
                      BooleanSupplier x, BooleanSupplier y, BooleanSupplier a, BooleanSupplier b, DoubleSupplier leftX,
                      double shootVelocity, double funnelVelocity, 
                      double indexerVelocity, double lowIndexerVelocity, double intakeVelocity,double bIntakeVelocity, 
                      double tolerance) {
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    
    this.shootVelocity = shootVelocity;
    this.funnelVelocity = funnelVelocity;
    this.indexerVelocity = indexerVelocity;
    this.lowIndexerVelocity = lowIndexerVelocity;
    this.intakeVelocity = intakeVelocity;
    this.bIntakeVelocity = bIntakeVelocity;
    this.tolerance = tolerance;

    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.leftBumper = leftBumper;
    this.rightBumper = rightBumper;

    this.leftX = leftX;

    this.x = x;
    this.y = y;
    this.a = a;
    this.b = b;
    
    addRequirements(indexer, turret, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Velocity", turret.getShooterVelocity());
    SmartDashboard.putNumber("Funnel Velocity", turret.getFunnelVelocity());
    SmartDashboard.putNumber("Turret Position", turret.getTurretPosition());

    double joyX = MathUtil.applyDeadband(leftX.getAsDouble(), 0.05);

    if (Math.abs(joyX) > 0.05) {
      turret.rotateTurret(joyX * 0.2);
    } else {
      if (x.getAsBoolean()) {
        turret.rotateToPos(-0.25);
      } else if (y.getAsBoolean()) {
        turret.rotateToPos(0);
      } else if (a.getAsBoolean()) {
        turret.rotateToPos(-0.5);
      } else if(b.getAsBoolean()){
        turret.rotateToPos(0.25);
      }else {
        turret.stopTurret();
      }
    }

    if (rightTrigger.getAsBoolean()) {
      runShooterSequence();
    } else if(leftTrigger.getAsBoolean()){
      intake.rotateToVelocity(intakeVelocity);
    }else{
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

  public void runShooterSequence(
    double shooterVel,
    double funnelVel,
    double lowIndexerVel,
    double indexerVel,
    double intakeVel,
    double tolerance,
    Indexer indexer,
    Intake intake
) {
    rotateToVelocity(shooterVel);
    indexer.rotateToVelocity(lowIndexerVel);
    intake.rotateToVelocity(intakeVel);

    if (shooterAtVelocity(shooterVel, tolerance) &&
        funnelAtVelocity(funnelVel, tolerance)) {

        indexer.rotateToVelocity(indexerVel);
        rotateFunnelToVelocity(funnelVel);
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
