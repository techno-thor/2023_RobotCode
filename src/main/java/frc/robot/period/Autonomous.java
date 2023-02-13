package frc.robot.period;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import frc.molib.dashboard.Option;
import frc.molib.utilities.Console;
import frc.robot.Robot;
import frc.robot.subsystem.Chassis;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.Manipulator;

/**
 * The Autonomous Period.
 * <p>
 * Reads options from the Dashboard to run various pre-programmed sequences
 * at the start of the match, free of driver input.
 */
@SuppressWarnings("all")
public class Autonomous {
    /** Starting Positions on the field. */
    private enum StartPosition {
        LOADING_ZONE("Loading Zone Side"),
        CHARGE_STATION("Charge Station"),
        WALL("Wall Side");

        private final String label;
        private StartPosition(String label) {
            this.label = label;
        }

        @Override
        public String toString() { return label; }
    }

    /** Pre-programmed sequences. */
    private enum Sequence {
        /** Do nothing. Don't move, don't score, don't even blink. */
        DO_NOTHING("Do Nothing") {
            public void run() {
                switch(mStage) {
                    case 0:
                        Console.logMsg("Starting Sequence \"" + Sequence.DO_NOTHING.toString() + "\"");
                        mStage++;
                        break;
                    case 1:
                        Console.logMsg("Sequence Complete \"" + Sequence.DO_NOTHING.toString() + "\"");
                        mStage++;
                    default:
                        //Disable all Subsystems
                        Chassis.disable();
                        Elevator.disable();
                        Manipulator.disable();
                }
            }
        },
        /** Just drive forward to get the Mobility points. */
        JUST_DRIVE("Just Drive") {
            public void run() {
                switch(mSelecetedStartPosition) {
                    case LOADING_ZONE:
                        switch(mStage) {
                            case 0:
                                Console.logMsg("Starting Sequence \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.LOADING_ZONE.toString());
                                mStage++;
                                break;
                            case 1:
                                Console.logMsg("Starting to drive 5.0ft forward...");
                                Chassis.goToDistance(60.0);
                                tmrStageTimeOut.reset();
                                mStage++;
                                break;
                            case 2:
                                if(Chassis.isAtDistance() || tmrStageTimeOut.get() > 5.0) mStage++;
                                break;
                            case 3:
                                Console.logMsg("Distance reached or timed out. Stopping drive...");
                                Chassis.disable();
                                mStage++;
                                break;
                            case 4:
                                Console.logMsg("Sequence Complete \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.LOADING_ZONE.toString());
                                mStage++;
                            default:
                                //Disable all Subsystems
                                Chassis.disable();
                                Elevator.disable();
                                Manipulator.disable();
                                break;
                        } break;
                    case CHARGE_STATION:
                        switch(mStage) {
                            case 0:
                                Console.logMsg("Starting Sequence \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.CHARGE_STATION.toString());
                                mStage++;
                                break;
                            case 1:
                                Console.logMsg("Starting to drive 10.0ft forward over the Charge Station...");
                                Chassis.goToDistance(120.0);
                                tmrStageTimeOut.reset();
                                mStage++;
                                break;
                            case 2:
                                if(Chassis.isAtDistance() || tmrStageTimeOut.get() > 10.0) mStage++;
                                break;
                            case 3:
                                Console.logMsg("Distance reached or timed out. Stopping drive...");
                                Chassis.disable();
                                mStage++;
                                break;
                            case 4:
                                Console.logMsg("Sequence Complete \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.CHARGE_STATION.toString());
                                mStage++;
                            default:
                                //Disable all Subsystems
                                Chassis.disable();
                                Elevator.disable();
                                Manipulator.disable();
                                break;
                        } break;
                    case WALL:
                        switch(mStage) {
                            case 0:
                                Console.logMsg("Starting Sequence \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.WALL.toString());
                                tmrStageTimeOut.reset();
                                mStage++;
                                break;
                            case 1:
                                Console.logMsg("Starting to drive 10.0ft forward...");
                                Chassis.goToDistance(120.0);
                                tmrStageTimeOut.reset();
                                mStage++;
                                break;
                            case 2:
                                //Wait for Chassis to drive the distance or for the timeout
                                if(Chassis.isAtDistance() || tmrStageTimeOut.get() > 10.0) mStage++;
                                break;
                            case 3:
                                Console.logMsg("Distance reached or timed out. Stopping drive...");
                                Chassis.disable();
                                mStage++;
                                break;
                            case 4:
                                Console.logMsg("Sequence Complete \"" + Sequence.JUST_DRIVE.toString() + "\" - " + StartPosition.WALL.toString());
                                mStage++;
                            default:
                                //Disable all Subsystems
                                Chassis.disable();
                                Elevator.disable();
                                Manipulator.disable();
                                break;
                        } break;
                    default:
                        //Disable all Subsystems
                        Chassis.disable();
                        Elevator.disable();
                        Manipulator.disable();
                        break;
                }
            }
        };
        
        //Timers
        public static final Timer tmrStageTimeOut = new Timer();

        //Static Variables
        private static int mStage = 0;
        
        private final String label;
        private Sequence(String label) {
            this.label = label;
        }

        /** First time initialization for the sequence. Override if additional actions are needed. */
        public static void init() {
            mStage = 0;
            tmrStageTimeOut.start();
            tmrStageTimeOut.reset();
        }

        /** The actual code for running the Autonomous Sequence. */
        public abstract void run();

        @Override
        public String toString() { return label; }
    }

    //Period NetworkTable
    private static final NetworkTable tblAutonomous = Robot.tblPeriod.getSubTable("Autonomous");

    //Dashboard Options
    private static final Option<StartPosition> optStartPosition = new Option<StartPosition>(tblAutonomous, "Start Position", StartPosition.CHARGE_STATION);
    private static final Option<Sequence> optSequence = new Option<Sequence>(tblAutonomous, "Sequence", Sequence.DO_NOTHING);

    //Variables
    private static StartPosition mSelecetedStartPosition;
    private static Sequence mSelectedSequence;

    /** Private constructor, cannot instantiate. */
    private Autonomous() {}

    /** Initialize all components and subsystems. */
    public static void init() {
        Console.logMsg("Initializing Autonomous Period...");

        //Pull selected Options
        Console.logMsg("Pulling selected Dashboard Options...");
        mSelecetedStartPosition = optStartPosition.get();
        mSelectedSequence = optSequence.get();

        //Set Chassis into Brake Mode
        Console.logMsg("Setting Chassis into Brake mode...");
        Chassis.setDriveNeutralMode(NeutralMode.Brake);

        //Sequence initialization
        Console.logMsg("Initializing Autonomous Sequence...");
        Sequence.init();

        Console.logMsg("Autonomous Sequence initialization complete.");

        Console.logMsg("Autonomous Period initialization complete.");
    }

    /** Initialize all dashboard values. */
    public static void initDashboard() {
        optStartPosition.init();
        optSequence.init();
    }

    /** Push new values to dashboard objects. */
    public static void pushDashboardValues() {

    }

    /** Update all components and subsystems. */
    public static void periodic() {
        mSelectedSequence.run();

        Chassis.periodic();
        Elevator.periodic();
        Manipulator.periodic();
    }
}

