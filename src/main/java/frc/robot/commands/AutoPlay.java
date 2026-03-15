// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private final CommandXboxController operatorJoystick;
  private final double tolerance;
  private final double indexerVelocity;

 
  public AutoPlay(Indexer indexer, Turret turret, Intake intake, Vision vision, CommandXboxController operatorJoystick, double tolerance, double indexerVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    this.operatorJoystick = operatorJoystick;
    this.tolerance = tolerance;
    this.indexerVelocity = indexerVelocity;
    
    addRequirements(indexer, turret, intake, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turretAngle = vision.getAngleToHub().in(Degrees);
    turret.setTurretAngleDegrees(turretAngle);

    double maxShooterVelocity = 90;
    double minShooterVelocity = 50; 

    double shooterVelocity = (2 * minShooterVelocity + maxShooterVelocity);
    double funnelVelocity = shooterVelocity * 0.8;

    if(operatorJoystick.rightTrigger().getAsBoolean()) {
      turret.rotateToVelocity(shooterVelocity);
      turret.rotateFunnelToVelocity(funnelVelocity);

      if(turret.shooterAtVelocity(shooterVelocity, tolerance) && turret.funnelAtVelocity(funnelVelocity, tolerance)){
        indexer.rotateToVelocity(indexerVelocity);
      }

    } else {
      turret.stopShooter(0);
      turret.stopFunnel(0);
      intake.stopIntake();
      indexer.stopIndexer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
