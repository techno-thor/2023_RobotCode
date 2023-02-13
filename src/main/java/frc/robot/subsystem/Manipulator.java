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
    /** Preset Arm angles. */
    public enum Position {
        RETRACTED(0.0),
        EXTENDED(115.0);

        public final double angle;
        private Position(double angle) {
            this.angle = angle;
        }
    }

    //Subsystem NetworkTable
    private static final NetworkTable tblManipulator = Robot.tblSubsystem.getSubTable("Manipulator");

    //Dashboard Entries
    private static final Entry<Double> entArm_Angle = new Entry<Double>(tblManipulator, "Arm Angle");

    //Motors
    private static final WPI_TalonFX mtrArm = new WPI_TalonFX(7);

    //Solenois
    private static final Solenoid solGrip = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    //Sensors TODO: Find out how they plan to limit arm if not pneumatics
    private static final MagEncoder encArm = new MagEncoder(mtrArm);

    //PID Controllers
    private static final PIDController pidArm_Angle = new PIDController(0.0, 0.0, 0.0);

    //Variables
    private static double mArmPower = 0.0;
    private static boolean mIsGripOpen = false;

    /** Private constructor, cannot instantiate. */
    private Manipulator() {}
    
    /** Initialize subsystem and all components. */
    public static void init() {
        Console.logMsg("Initializing Manipulator Subsystem...");

        //Configure motor inversions
        Console.logMsg("Configuring Motor inversions...");
        mtrArm.setInverted(false);

        //Configure motor NeutralModes
        Console.logMsg("Configuring Motor NeutralModes...");
        mtrArm.setNeutralMode(NeutralMode.Brake);

        //Configure Sensors
        Console.logMsg("Configuring Sensors...");
        encArm.configDistancePerPulse(360.0 / 2048.0);

        //Configure PIDs
        Console.logMsg("Configuring PIDs...");
        pidArm_Angle.setTolerance(5.0);
        pidArm_Angle.configAtSetpointTime(0.125);
        pidArm_Angle.configOutputRange(-1.0, 1.0);

        //Reset Sensors
        Console.logMsg("Resetting Sensor values...");
        resetArmAngle();

        Console.logMsg("Manipulator Subsystem initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {
        //Push Sensor values
        entArm_Angle.set(getArmAngle());
    }

    /** Reset Arm encoder distance. */
    public static void resetArmAngle() { encArm.reset(); }

    /**
     * Read the angle of the Arm since last reset.
     * @return Angle in degrees.
     */
    public static double getArmAngle() { return encArm.getDistance(); }

    /**
     * Assign power to the Arm motor buffer.
     * @param power [-1.0, 1.0] Power to the Arm
     */
    public static void setArmPower(double power) {
        mArmPower = power;
    }

    /** Turn off the Arm motor. */
    public static void disableArm() { setArmPower(0.0); }

    /**
     * Enable Arm Angle PID control of the Manipulator.
     * @param angle Target angle in degrees.
     */
    public static void goToAngle(double angle) {
        pidArm_Angle.setSetpoint(angle);
        pidArm_Angle.enable();
    }

    /**
     * Enable Arm Angle PID control of the Manipulator.
     * @param position Target angle via preset positions.
     */
    public static void goToAngle(Position position) {
        goToAngle(position.angle);
    }

    /** Enable Arm Angle PID to extend the Arm. */
    public static void extendArm() { goToAngle(Position.EXTENDED); }

    /** Enable Arm Angle PID to retract the Arm. */
    public static void retractArm() { goToAngle(Position.RETRACTED); }

    /**
     * Read whether the the Arm Angle PID has reached its target angle.
     * @return True if the Arm angle is on target.
     */
    public static boolean isAtAngle() { return pidArm_Angle.atSetpoint(); }

    /** Disable Arm Angle PID control of the Manipulator. */
    public static void disableAnglePID() { pidArm_Angle.disable(); }

    /**
     * Read whether the Grip is open.
     * @return True if the Grip is set to open.
     */
    public static boolean isGripOpen() { return mIsGripOpen; }

    /** Open the Grip. */
    public static void openGrip() { mIsGripOpen = true; }

    /** Close the Grip. */
    public static void closeGrip() { mIsGripOpen = false; }

    /** Disable all PID control of the Manipulator. */
    public static void disablePIDs() {
        disableAnglePID();
    }

    /** Disable all PID control of the Maipulator and stop the Arm motor. */
    public static void disable() {
        disablePIDs();
        disableArm();
    }

    /** Update subsystem and all components. */
    public static void periodic() {
        //PID Override
        if(pidArm_Angle.isEnabled())
            setArmPower(pidArm_Angle.calculate(getArmAngle()));

        //Safety Checks

        //Apply buffers
        mtrArm.set(mArmPower);
        solGrip.set(mIsGripOpen);
    }
}
