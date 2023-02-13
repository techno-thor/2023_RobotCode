package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.molib.PIDController;
import frc.molib.dashboard.Entry;
import frc.molib.sensors.MagEncoder;
import frc.molib.utilities.Console;
import frc.robot.Robot;

/**
 * The Chassis Subsystem.
 * <p>
 * Manages control of the Drive motors.
 */
@SuppressWarnings("all")
public class Chassis {
    //Subsystem NetworkTable
    private static final NetworkTable tblChassis = Robot.tblSubsystem.getSubTable("Chassis");

    //Motors
    private static final WPI_TalonFX mtrDrive_L1 = new WPI_TalonFX(1);
    private static final WPI_TalonFX mtrDrive_L2 = new WPI_TalonFX(2);
    private static final WPI_TalonFX mtrDrive_R1 = new WPI_TalonFX(3);
    private static final WPI_TalonFX mtrDrive_R2 = new WPI_TalonFX(4);

    //Sensors
    private static final MagEncoder encDrive = new MagEncoder(mtrDrive_L1);
    private static final ADXRS450_Gyro gyrDrive = new ADXRS450_Gyro();

    //PID Controllers
    private static final PIDController pidDrive_Distance = new PIDController(0.0, 0.0, 0.0);
    private static final PIDController pidDrive_Angle = new PIDController(0.0, 0.0, 0.0);

    //Dashboard Entries
    private static final Entry<Double> entDrive_Distance = new Entry<Double>(tblChassis, "Drive Distance");
    private static final Entry<Double> entDrive_Speed = new Entry<Double>(tblChassis, "Drive Speed");
    private static final Entry<Double> entDrive_Angle = new Entry<Double>(tblChassis, "Drive Angle");

    //Variables
    private static double mLeftDrivePower = 0.0;
    private static double mRightDrivePower = 0.0;

    /** Private constructor, cannot instantiate. */
    private Chassis() {}

    /** Initialize subsystem and all components. */
    public static void init() {
        Console.logMsg("Initializing Chassis Subsystem...");

        //Confiugure motor inversions
        Console.logMsg("Configuring Motor inversions...");
        mtrDrive_L1.setInverted(true);
        mtrDrive_L2.setInverted(true);
        mtrDrive_R1.setInverted(false);
        mtrDrive_R2.setInverted(false);

        //Configure motor NeutralModes
        Console.logMsg("Configuring Motor NeutralModes...");
        mtrDrive_L1.setNeutralMode(NeutralMode.Coast);
        mtrDrive_L2.setNeutralMode(NeutralMode.Coast);
        mtrDrive_R1.setNeutralMode(NeutralMode.Coast);
        mtrDrive_R2.setNeutralMode(NeutralMode.Coast);
        

        //Configure motor followers
        Console.logMsg("Configuring Motor followers...");
        mtrDrive_L2.follow(mtrDrive_L1);
        mtrDrive_R2.follow(mtrDrive_R1);

        //Confiure sensors FIXME: Determine Drive disantPerPulse
        Console.logMsg("Configuring Sensors...");
        encDrive.configDistancePerPulse(1.0);

        //Confiure PIDs
        Console.logMsg("Configuring PIDs...");
        pidDrive_Distance.setTolerance(0.25);
        pidDrive_Distance.configAtSetpointTime(0.125);
        pidDrive_Distance.configOutputRange(-1.0, 1.0);

        pidDrive_Angle.setTolerance(5.0);
        pidDrive_Angle.configAtSetpointTime(0.125);
        pidDrive_Angle.configOutputRange(-1.0, 1.0);

        //Calibrate sensors
        Console.logMsg("Calibrating Sensors...");
        gyrDrive.calibrate();

        //Reset sensors
        Console.logMsg("Resetting Sensor values...");
        resetDistance();
        resetAngle();

        Console.logMsg("Chassis Subsystem initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {

    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {
        //Push Sensor values
        entDrive_Distance.set(encDrive.getDistance());
        entDrive_Speed.set(encDrive.getRate());
        entDrive_Angle.set(gyrDrive.getAngle());
    }

    /** Reset Drive encoder distance */
    public static void resetDistance() { encDrive.reset(); }

    /** 
     * Read how far the Chassis has driven since last reset. 
     * 
     * @return Distance in inches.
     */
    public static double getDistance() { return encDrive.getDistance(); }

    /**
     * Read how fast the Chassis is driving.
     * 
     * @return Speed in inches per second.
     */
    public static double getSpeed() { return encDrive.getDistance(); }

    /** Reset Drive gyro angle. */
    public static void resetAngle() { gyrDrive.reset(); }

    /**
     * Read how far the Chassis has turned since last reset.
     * 
     * @return Angle in degrees.
     */
    public static double getAngle() { return gyrDrive.getAngle(); }

    /**
     * Set which NeutralMode the Drive motors should be in.
     * @param mode Mode to set the motors to.
     */
    public static void setDriveNeutralMode(NeutralMode mode) {
        mtrDrive_L1.setNeutralMode(mode);
        mtrDrive_L2.setNeutralMode(mode);
        mtrDrive_R1.setNeutralMode(mode);
        mtrDrive_R2.setNeutralMode(mode);
    }

    /**
     * Assign power to the Drive motor buffers.
     * @param leftPower     [-1.0, 1.0] Power to the left side of the Chassis.
     * @param rightPower    [-1.0, 1.0] Power to the right side of the Chassis.
     */
    public static void setDrivePower(double leftPower, double rightPower) {
        mLeftDrivePower = leftPower;
        mRightDrivePower = rightPower;
    }

    /** Turn off the Drive motors. */
    public static void disableDrive() { setDrivePower(0.0, 0.0); }

    /**
     * Enable Drive Distance PID control of the Chassis.
     * @param distance Target distance to drive.
     */
    public static void goToDistance(double distance) {
        pidDrive_Distance.setSetpoint(distance);
        pidDrive_Distance.enable();
    }

    /**
     * Read whether the the Drive Distance PID has reached its target distance.
     * @return True if the Chassis distance is on target.
     */
    public static boolean isAtDistance() { return pidDrive_Distance.atSetpoint(); }

    /** Disable Drive Distance PID control of the Chassis. */
    public static void disableDistancePID() { pidDrive_Distance.disable(); }

    /**
     * Enable Drive Angle PID control of the Chassis.
     * @param angle Target angle to turn to.
     */
    public static void goToAngle(double angle) {
        pidDrive_Angle.setSetpoint(angle);
        pidDrive_Angle.enable();
    }

    /**
     * Read whether the the Drive Angle PID has reached its target distance.
     * @return True if the Chassis angle is on target.
     */
    public static boolean isAtAngle() { return pidDrive_Angle.atSetpoint(); }

    /** Disable Drive Angle PID control of the Chassis. */
    public static void disableAnglePID() { pidDrive_Angle.disable(); }

    /** Disable all PID control of the Chassis. */
    public static void disablePIDs(){
        disableDistancePID();
        disableAnglePID();
    }

    /** Disable all PID control of the Chassis and stop the Drive motors. */
    public static void disable() {
        disablePIDs();
        disableDrive();
    }

    /** Update subsystem and all components. */
    public static void periodic() {
        //PID Override
        if(pidDrive_Distance.isEnabled()) 
            setDrivePower(pidDrive_Distance.calculate(getDistance()), pidDrive_Distance.calculate(getDistance()));
        else if(pidDrive_Angle.isEnabled()) 
            setDrivePower(pidDrive_Distance.calculate(getDistance()), -pidDrive_Distance.calculate(getDistance()));

        //Apply buffers
        mtrDrive_L1.set(mLeftDrivePower);
        mtrDrive_R1.set(mRightDrivePower);
    }
}
