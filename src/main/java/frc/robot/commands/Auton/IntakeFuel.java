// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFuel extends Command {
  /** Creates a new IntakeFuel. */
  private final Intake intake;
  private final Indexer indexer;
  private final double intakeSpeed;
  private final double indexSpeed;

  public IntakeFuel(Intake intake, Indexer indexer, double intakeSpeed, double indexSpeed) {
    this.intake = intake;
    this.indexer = indexer;
    this.intakeSpeed = intakeSpeed;
    this.indexSpeed = indexSpeed;

    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.rotateToVelocity(intakeSpeed);
    indexer.rotateToVelocity(indexSpeed);
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
