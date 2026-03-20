// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoPlay;
import frc.robot.commands.ManualPlay;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.Auton.Shoot;
import frc.robot.commands.Auton.MoveIntake;
import frc.robot.commands.Auton.RotateTurret;
import frc.robot.commands.Auton.IntakeFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DriverJoystick = new CommandXboxController(0); // My DriverJoystick
    private final CommandXboxController OperatorJoystick = new CommandXboxController(1); // My OperatorJoystick

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();

    public final Vision vision = new Vision(drivetrain);

    private boolean isAutoMode = false;
    private Command manualPlay;
    private Command autoPlay;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", new Shoot(indexer, turret, intake, vision, 5, 70, 40, -0.7));
        NamedCommands.registerCommand("Intake Down", new MoveIntake(intake, true));
        NamedCommands.registerCommand("Intake Up", new MoveIntake(intake, false));
        NamedCommands.registerCommand("Intake", new IntakeFuel(intake, indexer, 0.7, 40));
        NamedCommands.registerCommand("Rotate Turret", new RotateTurret(turret, 0.25));

        manualPlay = new ManualPlay(
            indexer, turret, intake, OperatorJoystick.leftTrigger(), OperatorJoystick.rightTrigger(), 
            OperatorJoystick.leftBumper(), OperatorJoystick.rightBumper(),
            OperatorJoystick.x(), OperatorJoystick.y(), OperatorJoystick.a(), OperatorJoystick::getLeftX,
            80, 70, 70, 30, 0.7, -0.5, 2
        );

        autoPlay = new AutoPlay(
            indexer, turret, intake, vision, OperatorJoystick.leftTrigger(), OperatorJoystick.rightTrigger(), 
            OperatorJoystick.leftBumper(), OperatorJoystick.rightBumper(), 
            2, 100, 0.7, 40, -0.7
        );

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putBoolean("Auto Mode Enabled", isAutoMode);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        DriverJoystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        );
        
        DriverJoystick.leftBumper().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );
        
        drivetrain.setDefaultCommand(
            new SwerveDrive(
                drivetrain, 
                DriverJoystick
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);


        //OPERATPOR JOYSTICK BINDINGS
        //Toggle Auto Mode on button 7
        OperatorJoystick.button(7).onTrue(
            Commands.defer(() -> {
                isAutoMode = !isAutoMode;

                if (isAutoMode) {
                    return autoPlay;
                } else {
                    return manualPlay;
                }
            }, Set.of(indexer, turret, intake))
        );
        
        

        // indexer.setDefaultCommand(indexer.run(()->indexer.manualControl(
        //     OperatorJoystick.rightTrigger(), //index
        //     100))); //speed

        // turret.setDefaultCommand(turret.run(()->turret.manualControl(
        //     (OperatorJoystick.getLeftX() >= 0.15 || OperatorJoystick.getLeftX() <= -0.15), //rotate
        //     MathUtil.applyDeadband(OperatorJoystick.getLeftX()*0.2, 0.1), // speed
        //     OperatorJoystick.a(), //rotatetopos
        //     0.25, //position
        //     OperatorJoystick.rightTrigger(), //shoot
        //     100, //shoot velocity
        //     70))); //funnel velocity

        //SysId routines for shooter
        // OperatorJoystick.a().whileTrue(turret.sysIdQuasistaticForward());
        // OperatorJoystick.b().whileTrue(turret.sysIdQuasistaticReverse());
        // OperatorJoystick.x().whileTrue(turret.sysIdDynamicForward());
        // OperatorJoystick.y().whileTrue(turret.sysIdDynamicReverse());

        //SysId routines for Funnel
        // OperatorJoystick.a().whileTrue(turret.sysIdFunnelQuasistaticForward());
        // OperatorJoystick.b().whileTrue(turret.sysIdFunnelQuasistaticReverse());
        // OperatorJoystick.x().whileTrue(turret.sysIdFunnelDynamicForward());
        // OperatorJoystick.y().whileTrue(turret.sysIdFunnelDynamicReverse());

        // Note that each routine should be run exactly once in a single log.
        // DriverJoystick.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // DriverJoystick.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // DriverJoystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // DriverJoystick.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
  }
