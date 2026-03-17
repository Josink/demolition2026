// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoPlay;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.SwerveDrive;
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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public final SendableChooser<Command> turretControlMode;

    private boolean isAutoMode = false;
    private Command currentTurretCommand = null;

    private Command manualCommand;
    
    private Command autoCommand;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        turretControlMode = new SendableChooser<Command>();
        turretControlMode.addOption("Manual Control", new ManualShoot(indexer, turret, intake, OperatorJoystick, vision, 180, 85, 70, 70, -30, 2));
        turretControlMode.addOption("Auto Control", new AutoPlay(indexer, turret, intake, vision, OperatorJoystick, 2, 70));
        turretControlMode.setDefaultOption(
            "Manual Control",
            new ManualShoot(indexer, turret, intake, OperatorJoystick, vision, 180, 85, 70, 70, -30, 2)
        );
        SmartDashboard.putData("Turret Control Mode", turretControlMode);

        manualCommand = new ManualShoot(
            indexer, turret, intake, OperatorJoystick, vision,
            180, 85, 70, 70, -30, 2
        );

        autoCommand = new AutoPlay(
            indexer, turret, intake, vision, OperatorJoystick,
            2, 70
        );


        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            new SwerveDrive(
                drivetrain, 
                DriverJoystick
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //OPERATPOR JOYSTICK BINDINGS
        turret.setDefaultCommand(    
            turret.run(() -> {
            turret.stopShooter(0);
            turret.stopFunnel(0);
        }));

        OperatorJoystick.a().onTrue(
            turret.runOnce(() -> {
                // Flip mode
                isAutoMode = !isAutoMode;
        
                // Cancel current command if running
                if (currentTurretCommand != null) {
                    currentTurretCommand.cancel();
                }
        
                // Pick new command
                currentTurretCommand = isAutoMode ? autoCommand : manualCommand;
        
                // Schedule it
                currentTurretCommand.schedule();
            })
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
