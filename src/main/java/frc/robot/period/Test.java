package frc.robot.period;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import frc.molib.utilities.Console;
import frc.robot.Robot;
import frc.robot.subsystem.Chassis;

@SuppressWarnings("all")
public class Test {
    private static final NetworkTable tblTest = Robot.tblPeriod.getSubTable("Test");

    /** Private constructor, cannot instantiate. */
    private Test() {}

    /** Initialize all components and subsystems. */
    public static void init() {
        Console.logMsg("Initializing Test Period...");
        
        //Set Chassis into Brake Mode
        Console.logMsg("Setting Chassis into Coast mode...");
        Chassis.setDriveNeutralMode(NeutralMode.Coast);

        Console.logMsg("Test Period initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {

    }

    /** Update all components and subsystems. */
    public static void periodic() {
        
    }
}
