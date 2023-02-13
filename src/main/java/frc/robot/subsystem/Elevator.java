package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import frc.molib.PIDController;
import frc.molib.dashboard.Entry;
import frc.molib.sensors.DigitalInput;
import frc.molib.sensors.MagEncoder;
import frc.molib.utilities.Console;
import frc.robot.Robot;

/**
 * The Elevator Subsystem.
 * <p>
 * Manages control over the height of the Lift.
 */
@SuppressWarnings("all")
public class Elevator {
    /** Preset Elevator heights. */
    public enum Position {
        LOW(0.0),
        MIDDLE(20.0),
        HIGH(50.0);

        public final double height;
        private Position(double height) {
            this.height = height;
        }
    }

    //Subsystem NetworkTable
    private static final NetworkTable tblElevator = Robot.tblSubsystem.getSubTable("Elevator");

    //Motors
    private static final WPI_TalonFX mtrLift = new WPI_TalonFX(5);

    //Sensors
    private static final MagEncoder encLift = new MagEncoder(mtrLift);
    private static final DigitalInput phoLift_B = new DigitalInput(0, true);
    private static final DigitalInput phoLift_T = new DigitalInput(1, true);

    //PID Controllers
    private static final PIDController pidLift_Height = new PIDController(0.0, 0.0, 0.0);

    //Dashboard Entries
    private static final Entry<Double> entLift_Height = new Entry<Double>(tblElevator, "Lift Height");

    //Variables
    private static double mLiftPower = 0.0;

    /** Private constructor, cannot instantiate. */
    private Elevator() {}

    /** Initialize subsystem and all components. */
    public static void init() {
        Console.logMsg("Initializing Elevator Subsystem...");

        //Configure motor inversions
        Console.logMsg("Configuring Motor inversions...");
        mtrLift.setInverted(false);

        //Configure motor NeutralModes
        Console.logMsg("Configuring Motor NeutralModes...");
        mtrLift.setNeutralMode(NeutralMode.Brake);

        //Configure Sensors FIXME: Determine Lift distancePerPulse
        Console.logMsg("Configuring Sensors...");
        encLift.configDistancePerPulse(1.0);

        //Configure PIDS
        Console.logMsg("Configuring PIDs...");
        pidLift_Height.setTolerance(0.25);
        pidLift_Height.configAtSetpointTime(0.125);
        pidLift_Height.configOutputRange(-1.0, 1.0);

        //Reset sensors
        Console.logMsg("Resetting Sensor values...");
        resetLiftHeight();

        Console.logMsg("Elevator Subsystem initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {
        //Push Sensor values
        entLift_Height.set(getLiftHeight());
    }

    /** Reset Lift encoder distance. */
    public static void resetLiftHeight() { encLift.reset(); }

    /**
     * Read the height of the Lift since last reset.
     * 
     * @return Height in inches
     */
    public static double getLiftHeight() { return encLift.getDistance(); }

    /** 
     * Read whether the Lift is at its bottom limit. 
     * 
     * @return True if the bottom photoeye is tripped.
     */
    public static boolean isLiftAtBottom() { return phoLift_B.get(); }

    /** 
     * Read whether the Lift is at its top limit. 
     * 
     * @return True if the top photoeye is tripped.
     */
    public static boolean isLiftAtTop() { return phoLift_T.get(); }

    /**
     * Assign power to the Lift motor buffers
     * @param power [-1.0, 1.0] Power to the Lift
     */
    public static void setLiftPower(double power) {
        mLiftPower = power;
    }

    /** Turn off the Lift motor. */
    public static void disableLift() { setLiftPower(0.0); }

    /**
     * Enable Lift Height PID control of the Elevator.
     * @param height Target height in inches.
     */
    public static void goToHeight(double height) {
        pidLift_Height.setSetpoint(height);
        pidLift_Height.enable();
    }

    /**
     * Enable Lift Height PID control of the Elevator.
     * @param position Target height via preset positions.
     */
    public static void goToHeight(Position position) {
        goToHeight(position.height);
    }

    /**
     * Read whether the the Lift Height PID has reached its target height.
     * @return True if the Lift height is on target.
     */
    public static boolean isAtHeight() { return pidLift_Height.atSetpoint(); }

    /** Disable Lift Height PID control of the Elevator. */
    public static void disableHeightPID() { pidLift_Height.disable(); }

    /** Disable all PID control of the Elevator. */
    public static void disablePIDs() {
        disableHeightPID();
    }

    /** Disable all PID control of the Elevator and stop the Lift motors. */
    public static void disable() {
        disablePIDs();
        disableLift();
    }

    /** Update subsystem and all components. */
    public static void periodic() {
        //PID Override
        if(pidLift_Height.isEnabled()) 
            setLiftPower(pidLift_Height.calculate(getLiftHeight()));

        //Safety Checks
        if(isLiftAtBottom())
            MathUtil.clamp(mLiftPower, 0.0, Double.POSITIVE_INFINITY);

        if(isLiftAtTop())
            MathUtil.clamp(mLiftPower, Double.NEGATIVE_INFINITY, 0.0);

        //Apply buffers
        mtrLift.set(mLiftPower);
    }
}
