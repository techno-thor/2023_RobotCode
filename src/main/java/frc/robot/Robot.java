// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.molib.buttons.ButtonManager;
import frc.molib.utilities.Console;
import frc.robot.period.Autonomous;
import frc.robot.period.Disabled;
import frc.robot.period.Teleoperated;
import frc.robot.period.Test;
import frc.robot.subsystem.Chassis;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Manipulator;

/**
 * The root of the Robot.
 * <p>
 * Manages control of the entire Robot, delegating to various Game Periods and Subsystems,
 * as well as handling global components such as NetworkTables, lights, or cameras.
 */
public class Robot extends TimedRobot {
    //Global NetworkTables
    public static final NetworkTable tblMain = NetworkTableInstance.getDefault().getTable("MO Data");
    public static final NetworkTable tblPeriod = tblMain.getSubTable("Period");
    public static final NetworkTable tblSubsystem = tblMain.getSubTable("Subsystem");

    //Driver Camera
    private static UsbCamera camDriver;

    @Override
    public void robotInit() {
        Console.logMsg("Robot initialization starting...");

        //Wait for NetworkTables
        Console.logMsg("Establishing NetworkTable Connection...");
        while(!NetworkTableInstance.getDefault().isConnected());

        //Initialize subsystems
        Console.logMsg("Initializing Subsystems...");
        Chassis.init();
        Elevator.init();
        Manipulator.init();

        //Initialize dashboard
        Console.logMsg("Initializing Dashboard Objects...");
        Chassis.initDashboard();
        Elevator.initDashboard();
        Manipulator.initDashboard();

        Disabled.initDashboard();
        Autonomous.initDashboard();
        Teleoperated.initDashboard();
        Test.initDashboard();

        //Initialize Cameras
        Console.logMsg("Initializing Cameras...");
        camDriver = CameraServer.startAutomaticCapture("Driver Camera", 0);
        camDriver.setResolution(160, 100);
        camDriver.setFPS(10);

        Console.logMsg("Robot initialization complete.");
    }

    @Override
    public void robotPeriodic() {
        //Update button values
        ButtonManager.updateValues();

        //Push new values to dashboard objects
        Chassis.pushDashboardValues();
        Elevator.pushDashboardValues();
        Manipulator.pushDashboardValues();

        Disabled.pushDashboardValues();
        Autonomous.pushDashboardValues();
        Teleoperated.pushDashboardValues();
        Test.pushDashboardValues();
    }

    @Override
    public void disabledInit() { Disabled.init(); }

    @Override
    public void disabledPeriodic() { Disabled.periodic(); }

    @Override
    public void autonomousInit() { Autonomous.init(); }

    @Override
    public void autonomousPeriodic() { Autonomous.periodic(); }

    @Override
    public void teleopInit() { Teleoperated.init(); }

    @Override
    public void teleopPeriodic() { Teleoperated.periodic(); }

    @Override
    public void testInit() { Test.init(); }

    @Override
    public void testPeriodic() { Test.periodic(); }
}
