// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ManualShoot extends Command {
  /** Creates a new ManualShoot. */
  private final Indexer indexer;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;

  private final CommandXboxController operatorJoystick;
  private final double degrees;
  private final double shootVelocity;
  private final double funnelVelocity;
  private final double intakeVelocity;
  private final double indexerVelocity;
  private final double tolerance;
  
  public ManualShoot(Indexer indexer, Turret turret, Intake intake, CommandXboxController operatorJoystick, Vision vision, double degrees, double shootVelocity, double funnelVelocity, double indexerVelocity, double intakeVelocity, double tolerance) {
    this.indexer = indexer;
    this.turret = turret;
    this.intake = intake;
    this.vision = vision;
    
    this.operatorJoystick = operatorJoystick;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Velocity", turret.getShooterVelocity());
    SmartDashboard.putNumber("Funnel Velocity", turret.getFunnelVelocity());
    SmartDashboard.putNumber("Turret Position", turret.getTurretPosition());

    if (operatorJoystick.getLeftX() >= 0.15 || operatorJoystick.getLeftX() <= -0.15){
      turret.rotateTurret(MathUtil.applyDeadband(operatorJoystick.getLeftX()*0.2, 0.1));
    }

    if(operatorJoystick.x().getAsBoolean()){
        double turretAngle = vision.getAngleToHub().in(Degrees);
        turret.setTurretAngleDegrees(turretAngle/2);
    }

    if (operatorJoystick.rightTrigger().getAsBoolean()) {
      turret.setTurretAngleDegrees(degrees);

      turret.rotateToVelocity(shootVelocity);
      turret.rotateFunnelToVelocity(funnelVelocity);

      if(turret.shooterAtVelocity(shootVelocity, tolerance) && turret.funnelAtVelocity(funnelVelocity, tolerance)){
        indexer.rotateToVelocity(indexerVelocity);
        intake.rotateToVelocity(intakeVelocity);
      }
    } else{
      turret.stopShooter(0);
      turret.stopFunnel(0);
      intake.stopIntake();
      indexer.stopIndexer();
    }

    if(operatorJoystick.leftBumper().getAsBoolean()){
      intake.down();
    } else if (operatorJoystick.rightBumper().getAsBoolean()){
      intake.up();
    } else{
      intake.off();
    }

    if (operatorJoystick.leftTrigger().getAsBoolean()){
      intake.rotateToVelocity(-intakeVelocity);
    } else {
      intake.stopIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
      return false;
  }
}
