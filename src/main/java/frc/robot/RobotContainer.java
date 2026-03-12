// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController DriverJoystick = new CommandXboxController(0); // My DriverJoystick
    private final CommandXboxController OperatorJoystick = new CommandXboxController(1); // My OperatorJoystick

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();

    //public final Vision vision = new Vision(drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
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

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        DriverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-DriverJoystick.getLeftY(), -DriverJoystick.getLeftX()))
        ));

        DriverJoystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        DriverJoystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverJoystick.back().and(DriverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverJoystick.back().and(DriverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverJoystick.start().and(DriverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverJoystick.start().and(DriverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        DriverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);


        //OPERATPOR JOYSTICK BINDINGS
        intake.setDefaultCommand(intake.run(()->intake.manualControl(
            OperatorJoystick.leftBumper(),  //up
            OperatorJoystick.rightBumper(), //down
            OperatorJoystick.leftTrigger(),  //intake
            100))); //velocity
        
        indexer.setDefaultCommand(indexer.run(()->indexer.manualControl(
            OperatorJoystick.rightTrigger(), //index
            100))); //speed

        // turret.setDefaultCommand(turret.run(()->turret.manualControl(
        //     (OperatorJoystick.getLeftX() >= 0.15 || OperatorJoystick.getLeftX() <= -0.15), //rotate
        //     MathUtil.applyDeadband(OperatorJoystick.getLeftX()*0.2, 0.1), // speed
        //     OperatorJoystick.a(), //rotatetopos
        //     0.25, //position
        //     OperatorJoystick.rightTrigger(), //shoot
        //     100, //shoot velocity
        //     70))); //funnel velocity
        
        OperatorJoystick.rightTrigger()

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
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
  }
