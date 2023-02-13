package frc.robot.period;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import frc.molib.utilities.Console;
import frc.robot.Robot;
import frc.robot.subsystem.Chassis;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Manipulator;

@SuppressWarnings("all")
public class Disabled {
    private static final NetworkTable tblDisabled = Robot.tblPeriod.getSubTable("Disabled");

    /** Private constructor, cannot instantiate. */
    private Disabled() {}

    /** Initialize all components and subsystems. */
    public static void init() {
        Console.logMsg("Initializing Disabled Period...");
        
        //Set Chassis into Brake Mode
        Console.logMsg("Setting Chassis into Coast mode...");
        Chassis.setDriveNeutralMode(NeutralMode.Coast);

        Console.logMsg("Disabled Period initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {

    }

    /** Update all components and subsystems. */
    public static void periodic() {
        Chassis.disable();
        Elevator.disable();
        Manipulator.disable();

        Chassis.periodic();
        Elevator.periodic();
        Manipulator.periodic();
    }
}
