package frc.robot.period;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import frc.molib.buttons.Button;
import frc.molib.dashboard.Option;
import frc.molib.hid.XboxController;
import frc.molib.utilities.Console;
import frc.robot.Robot;
import frc.robot.subsystem.Chassis;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Manipulator;

/**
 * The Teleoperated Period.
 * <p>
 * Reads input from the Driver Station to control the subsystems of the Robot during the match.
 */
@SuppressWarnings("all")
public class Teleoperated {
    private enum DriveControlMode {
        TANK("Tank Drive"),
        ARCADE("Arcade Drive"),
        CHEEZY("Cheezy Drive");

        private final String label;
        private DriveControlMode(String label) {
            this.label = label;
        }

        @Override
        public String toString() { return label; }
    }

    private enum DrivePowerScale {
        FAST("Fast - 80%", 0.30, 0.80, 1.00),
        NORMAL("Normal - 60%", 0.25, 0.60, 0.75),
        SLOW("Slow - 40%", 0.20, 0.40, 0.30),
        TURTLE("Turtle - 20%", 0.15, 0.20, 0.25);


        private final String label;
        public final double precisionMultiplier;
        public final double standardMultiplier;
        public final double boostedMultiplier;
        private DrivePowerScale(String label, double precisionMultiplier, double standardMultiplier, double boostedMultiplier) {
            this.label = label;
            this.precisionMultiplier = precisionMultiplier;
            this.standardMultiplier = standardMultiplier;
            this.boostedMultiplier = boostedMultiplier;
        }

        @Override
        public String toString() { return label; }
    }

    //Period NetworkTable
    private static final NetworkTable tblTeleoperated = Robot.tblPeriod.getSubTable("Teleoperated");

    //Dashboard Options
    private static final Option<DriveControlMode> optDriveControlMode = new Option<DriveControlMode>(tblTeleoperated, "Drive Control Mode", DriveControlMode.TANK);
    private static final Option<DrivePowerScale> optDrivePowerScale = new Option<DrivePowerScale>(tblTeleoperated, "Drive Power Scale", DrivePowerScale.NORMAL);

    //Controllers
    private static final XboxController ctlDriver = new XboxController(0);
    private static final XboxController ctlOperator = new XboxController(1);

    //Buttons
    private static final Button btnDrive_Precise = new Button() {
        @Override public boolean get() { return ctlDriver.getLeftBumper(); }
    };

    private static final Button btnDrive_Boost = new Button() {
        @Override public boolean get() { return ctlDriver.getRightBumper(); }
    };

    private static final Button btnElevator_Down = new Button() {
        @Override public boolean get() { return ctlOperator.getPOV() == 180; }
    };

    private static final Button btnElevator_Up = new Button() {
        @Override public boolean get() { return ctlOperator.getPOV() == 0; }
    };

    private static final Button btnElevator_Low = new Button() {
        @Override public boolean get() { return ctlOperator.getAButton(); }
    };

    private static final Button btnElevator_Middle = new Button() {
        @Override public boolean get() { return ctlOperator.getBButton(); }
    };

    private static final Button btnElevator_High = new Button() {
        @Override public boolean get() { return ctlOperator.getYButton(); }
    };
    
    private static final Button btnManipulator_Extend = new Button() {
        @Override public boolean get() { return ctlOperator.getLeftBumper(); }
    };
    
    private static final Button btnManipulator_Open = new Button() {
        @Override public boolean get() { return ctlOperator.getRightBumper(); }
    };

    //Variables
    private static DriveControlMode mDriveControlMode;
    private static DrivePowerScale mDrivePowerScale;

    /** Private constructor, cannot instantiate. */
    private Teleoperated() {}

    /** Initialize all components and subsystems. */
    public static void init() {
        Console.logMsg("Initializing Teleoperated Period...");

        //Pull selected Options
        Console.logMsg("Pulling selected Dashboard Options...");
        mDriveControlMode = optDriveControlMode.get();
        mDrivePowerScale = optDrivePowerScale.get();

        //Set Chassis into Brake Mode
        Console.logMsg("Setting Chassis into Coast mode...");
        Chassis.setDriveNeutralMode(NeutralMode.Coast);

        Console.logMsg("Teleoperated Period initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {
        //Push Options to the Dashboard
        optDriveControlMode.init();
        optDrivePowerScale.init();
    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {

    }

    /**
     * Drive the Chassis by controlling the left and right sides independently.
     * @param leftPower     [-1.0, 1.0] Power to the left side of the Chassis.
     * @param rightPower    [-1.0, 1.0] Power to the right side of the Chassis.
     */
    private static void tankDrive(double leftPower, double rightPower) {
        Chassis.setDrivePower(leftPower, rightPower);
    }

    /**
     * Drive the Chassis with a forward throttle and steering to turn.
     * @param throttle  [-1.0, 1.0] Forward/Reverse power.
     * @param steering  [-1.0, 1.0] Left/Right power.
     */
    private static void arcadeDrive(double throttle, double steering) {
        Chassis.setDrivePower(throttle + steering, throttle - steering);
    }

    /** Update all components and subsystems. */
    public static void periodic() {
        //Chassis Controls TODO: Square inputs to ramp power?
        //Arcade Drive Mode
        if(mDriveControlMode == DriveControlMode.ARCADE) {
            //Precision Power
            if(btnDrive_Precise.get()) {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.precisionMultiplier, ctlDriver.getLeftX() * mDrivePowerScale.precisionMultiplier);
            //Boosted Power
            } else if(btnDrive_Boost.get()) {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.boostedMultiplier, ctlDriver.getLeftX() * mDrivePowerScale.boostedMultiplier);
            //Standard Power
            } else {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.standardMultiplier, ctlDriver.getLeftX() * mDrivePowerScale.standardMultiplier);
            }
        //Cheezy Drive Mode
        } else if(mDriveControlMode == DriveControlMode.CHEEZY) {
            //Precision Power
            if(btnDrive_Precise.get()) {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.precisionMultiplier, ctlDriver.getRightX() * mDrivePowerScale.precisionMultiplier);
            //Boosted Power
            } else if(btnDrive_Boost.get()) {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.boostedMultiplier, ctlDriver.getRightX() * mDrivePowerScale.boostedMultiplier);
            //Standard Power
            } else {
                arcadeDrive(ctlDriver.getLeftY() * mDrivePowerScale.standardMultiplier, ctlDriver.getRightX() * mDrivePowerScale.standardMultiplier);
            }
        //Tank Drive Mode
        } else if(mDriveControlMode == DriveControlMode.TANK) {
            //Precision Power
            if(btnDrive_Precise.get()) {
                tankDrive(ctlDriver.getLeftY() * mDrivePowerScale.precisionMultiplier, ctlDriver.getRightY() * mDrivePowerScale.precisionMultiplier);
            //Boosted Power
            } else if(btnDrive_Boost.get()) {
                tankDrive(ctlDriver.getLeftY() * mDrivePowerScale.boostedMultiplier, ctlDriver.getRightY() * mDrivePowerScale.boostedMultiplier);
            //Standard Power
            } else {
                tankDrive(ctlDriver.getLeftY() * mDrivePowerScale.standardMultiplier, ctlDriver.getRightY() * mDrivePowerScale.standardMultiplier);
            }
        }
        //Elevator Controls
        if(btnElevator_Down.get()) {
            Elevator.disableHeightPID();
            Elevator.setLiftPower(-0.2);
        } else if(btnElevator_Up.get()) {
            Elevator.disableHeightPID();
            Elevator.setLiftPower(0.2);
        } else if(btnElevator_Low.getPressed()) {
            Elevator.goToHeight(Elevator.Position.LOW);
        } else if(btnElevator_Middle.getPressed()) {
            Elevator.goToHeight(Elevator.Position.MIDDLE);
        } else if(btnElevator_High.getPressed()) {
            Elevator.goToHeight(Elevator.Position.HIGH);
        } else { 
            Elevator.setLiftPower(0.0);
        }

        //Manipulator Controls - Arm
        if(btnManipulator_Extend.get()) {
            Manipulator.extendArm();
        } else { 
            Manipulator.retractArm();
        }

        //Manipulator Controls - Grip
        if(btnManipulator_Open.get()) {
            Manipulator.openGrip();
        } else {
            Manipulator.closeGrip();
        }

        //Update Subsystems
        Chassis.periodic();
        Elevator.periodic();
        Manipulator.periodic();
    }
}
