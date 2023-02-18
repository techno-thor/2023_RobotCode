package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.molib.PIDController;
import frc.molib.dashboard.Entry;
import frc.molib.sensors.MagEncoder;
import frc.molib.utilities.Console;
import frc.robot.Robot;

/**
 * The Manipulator Subsystem.
 * <p>
 * Manages control over extending/retracting the Arm and opening/closing the Grip.
 */
@SuppressWarnings("all")
public class Manipulator {
    //Subsystem NetworkTable
    private static final NetworkTable tblManipulator = Robot.tblSubsystem.getSubTable("Manipulator");

    //Solenois
    private static final Solenoid solArm = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private static final Solenoid solGrip = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    //Variables
    private static boolean mIsArmExtended = false;
    private static boolean mIsGripOpen = false;

    /** Private constructor, cannot instantiate. */
    private Manipulator() {}
    
    /** Initialize subsystem and all components. */
    public static void init() {
        Console.logMsg("Initializing Manipulator Subsystem...");

        Console.logMsg("Manipulator Subsystem initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {
        
    }

    /**
     * Read whether the Arm is extended.
     * 
     * @return True if the Arm is set to extended.
     */
    public static boolean isArmExtended() { return mIsArmExtended; }

    /** Extend the Arm. */
    public static void extendArm() { mIsArmExtended = true; }

    /** Retract the Arm. */
    public static void retractArm() { mIsArmExtended = false; }

    /**
     * Read whether the Grip is open.
     * 
     * @return True if the Grip is set to open.
     */
    public static boolean isGripOpen() { return mIsGripOpen; }

    /** Open the Grip. */
    public static void openGrip() { mIsGripOpen = true; }

    /** Close the Grip. */
    public static void closeGrip() { mIsGripOpen = false; }

    /** Placeholder to disable all components. */
    public static void disable() {
        
    }

    /** Update subsystem and all components. */
    public static void periodic() {
        //Apply buffers
        solArm.set(mIsArmExtended);
        solGrip.set(mIsGripOpen);
    }
}
