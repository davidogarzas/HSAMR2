package parkingRobot.hsamr2;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;
import lejos.robotics.navigation.Pose;

import parkingRobot.hsamr2.ControlRST;
import parkingRobot.hsamr2.HmiPLT;
import parkingRobot.hsamr2.NavigationAT;
import parkingRobot.hsamr2.PerceptionPMP;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.hsamr2.ControlRST;
import parkingRobot.hsamr2.HmiPLT;
import parkingRobot.hsamr2.NavigationAT;
import parkingRobot.hsamr2.PerceptionPMP;
/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking'.
 * This class handles the guidance and state logic for the parking robot.
 */
public class GuidanceAT {

    public enum CurrentStatus {
        SCOUT, // Following line and detecting parking slots
        PARK_THIS, // Performing parking maneuver
        AUSPARKEN, // Exiting a parking slot
        PAUSE, // Pausing all operations
        EXIT // Shutting down the system
    }

    private enum ScoutSubState {
        FOLLOW_LINE,
        DETECT_PARKING,
        LOG_PARKING
    }

    protected static CurrentStatus currentStatus = CurrentStatus.PAUSE;
    protected static CurrentStatus lastStatus = CurrentStatus.EXIT;
    private static ScoutSubState scoutSubState = ScoutSubState.FOLLOW_LINE;

    // Map definition
    static Line line0 = new Line(0, 0, 180, 0);
    static Line line1 = new Line(180, 0, 180, 60);
    static Line line2 = new Line(180, 60, 150, 60);
    static Line line3 = new Line(150, 60, 150, 30);
    static Line line4 = new Line(150, 30, 30, 30);
    static Line line5 = new Line(30, 30, 30, 60);
    static Line line6 = new Line(30, 60, 0, 60);
    static Line line7 = new Line(0, 60, 0, 0);

    static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};

    private IControl control;
    private INavigation navigation;
    private INxtHmi hmi;
    private IMonitor monitor;

    public GuidanceAT(IControl control, INavigation navigation, INxtHmi hmi, IMonitor monitor) {
        this.control = control;
        this.navigation = navigation;
        this.hmi = hmi;
        this.monitor = monitor;
    }

    public void execute() {
        switch (currentStatus) {
            case SCOUT:
                handleScout();
                break;

            case PARK_THIS:
                handleParkThis();
                break;

            case AUSPARKEN:
                handleAusparken();
                break;

            case PAUSE:
                handlePause();
                break;

            case EXIT:
                handleExit();
                break;
        }
    }

    private void handleScout() {
        if (lastStatus != CurrentStatus.SCOUT) {
            control.setCtrlMode(ControlMode.LINE_CTRL);
        }

        navigation.updateNavigation();

        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();
        if (slots != null) {
            for (INavigation.ParkingSlot slot : slots) {
                monitor.writeGuidanceComment("Detected parking slot ID: " + slot.getID());
            }
        }

        checkForStateTransition();
        lastStatus = currentStatus;
    }

    private void handleParkThis() {
        if (lastStatus != CurrentStatus.PARK_THIS) {
            control.setCtrlMode(ControlMode.PARK_CTRL);
        }

        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();
        int selectedSlot = hmi.getSelectedParkingSlot();
        for (INavigation.ParkingSlot slot : slots) {
            if (slot.getID() == selectedSlot) {
                Pose currentPose = navigation.getPose();
                Pose targetPose = computeParkingTrajectory(currentPose, slot);
                control.setDestination(targetPose.getHeading(), targetPose.getX(), targetPose.getY());
                break;
            }
        }

        checkForStateTransition();
        lastStatus = currentStatus;
    }

    private void handleAusparken() {
        if (lastStatus != CurrentStatus.AUSPARKEN) {
            control.setCtrlMode(ControlMode.PARK_CTRL);
        }

        Pose currentPose = navigation.getPose();
        Pose returnToLine = computeUnparkingTrajectory(currentPose);
        control.setDestination(returnToLine.getHeading(), returnToLine.getX(), returnToLine.getY());

        checkForStateTransition();
        lastStatus = currentStatus;
    }

    private void handlePause() {
        if (lastStatus != CurrentStatus.PAUSE) {
            control.setCtrlMode(ControlMode.INACTIVE);
        }

        checkForStateTransition();
        lastStatus = currentStatus;
    }

    private void handleExit() {
        monitor.writeGuidanceComment("Exiting guidance module.");
        System.exit(0);
    }

    private void checkForStateTransition() {
        INxtHmi.Mode mode = hmi.getMode();

        switch (mode) {
            case SCOUT:
                currentStatus = CurrentStatus.SCOUT;
                break;
            case PARK_THIS:
                currentStatus = CurrentStatus.PARK_THIS;
                break;
            case DISCONNECT:
                currentStatus = CurrentStatus.EXIT;
                break;
            case PAUSE:
                currentStatus = CurrentStatus.PAUSE;
                break;
            default:
                break;
        }
    }

    private Pose computeParkingTrajectory(Pose currentPose, INavigation.ParkingSlot slot) {
        float x2 = (float) slot.getBackBoundaryPosition().getX();
        float y2 = (float) slot.getBackBoundaryPosition().getY();

        float theta2 = (float) Math.PI / 2;

        Pose targetPose = new Pose(x2, y2, theta2);
        monitor.writeGuidanceComment("Generated parking trajectory to: " + targetPose.toString());

        return targetPose;
    }

    private Pose computeUnparkingTrajectory(Pose currentPose) {
        float x = currentPose.getX() - 20;
        float y = currentPose.getY();
        float theta = 0;

        Pose returnPose = new Pose(x, y, theta);
        monitor.writeGuidanceComment("Generated unparking trajectory to: " + returnPose.toString());

        return returnPose;
    }

    public static CurrentStatus getCurrentStatus() {
        return currentStatus;
    }

    public static void main(String[] args) throws Exception {
        while (Button.ENTER.isDown()) {}

        currentStatus = CurrentStatus.PAUSE;
        lastStatus = CurrentStatus.EXIT;

        NXTMotor leftMotor = new NXTMotor(MotorPort.B);
        NXTMotor rightMotor = new NXTMotor(MotorPort.A);

        IMonitor monitor = new Monitor();

        IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);

        LCD.clear();
        LCD.drawString("Calibrating Sensors...", 0, 0);
        perception.calibrateLineSensors();
        LCD.clear();
        LCD.drawString("Calibration Done", 0, 0);
        Thread.sleep(1000);

        INavigation navigation = new NavigationAT(perception, monitor);
        IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);
        INxtHmi hmi = new HmiPLT(perception, navigation, control, monitor);

        GuidanceAT guidance = new GuidanceAT(control, navigation, hmi, monitor);

        while (!Button.ENTER.isDown()) {
            LCD.clear();
            guidance.showData(navigation, perception);
            LCD.drawString("Press ENTER to Start", 0, 6);
            Thread.sleep(100);
        }
        while (Button.ENTER.isDown()) {}

        monitor.startLogging();

        while (true) {
            guidance.execute();
            Thread.sleep(100);
        }
    }

    protected static void showData(INavigation navigation, IPerception perception) {
        LCD.clear();
        LCD.drawString("X (in cm): " + (navigation.getPose().getX() * 100), 0, 0);
        LCD.drawString("Y (in cm): " + (navigation.getPose().getY() * 100), 0, 1);
        LCD.drawString("Phi (deg): " + (navigation.getPose().getHeading() * 180 / Math.PI), 0, 2);
    }
}