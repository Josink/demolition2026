// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    // Track if we're in SysId test mode
    private boolean m_sysIdMode = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(false);
        SignalLogger.setPath("/home/lvuser/logs/");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        if (m_sysIdMode) {
            SignalLogger.stop();
            m_sysIdMode = false;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        // In test mode, start logging automatically
        // SignalLogger.start();
        // m_sysIdMode = true;
        // System.out.println("SignalLogger started for SysId tests");
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        // if (m_sysIdMode) {
        //     SignalLogger.stop();
        //     m_sysIdMode = false;
        //     System.out.println("SignalLogger stopped");
        // }
    }

    @Override
    public void simulationPeriodic() {}
}
