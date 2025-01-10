package parkingRobot.hsamr2;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;
import lejos.robotics.navigation.Pose;

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 */
public class GuidanceAT {

    /**
     * States for the main finite state machine.
     */
    public enum CurrentStatus {
        SCOUT,
        PAUSE,
        PARK,
        EXIT
    }

    // Current status of the robot
    protected static CurrentStatus currentStatus = CurrentStatus.PAUSE;

    // Last known status of the robot
    protected static CurrentStatus lastStatus = CurrentStatus.EXIT;

    // Lap counter
    private static int lapCounter = 0;

    // Cooldown for lap detection
    private static long lastLapTime = 0;

    // Map lines for the robot's environment
    static Line line0 = new Line(0, 0, 180, 0);
    static Line line1 = new Line(180, 0, 180, 60);
    static Line line2 = new Line(180, 60, 150, 60);
    static Line line3 = new Line(150, 60, 150, 30);
    static Line line4 = new Line(150, 30, 30, 30);
    static Line line5 = new Line(30, 30, 30, 60);
    static Line line6 = new Line(30, 60, 0, 60);
    static Line line7 = new Line(0, 60, 0, 0);

    static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};

    public static void main(String[] args) throws Exception {

        while (Button.ENTER.isDown()) {}

        // Initialize robot status
        currentStatus = CurrentStatus.PAUSE;
        lastStatus = CurrentStatus.EXIT;

        // Initialize robot components
        NXTMotor leftMotor = new NXTMotor(MotorPort.B);
        NXTMotor rightMotor = new NXTMotor(MotorPort.A);

        IMonitor monitor = new Monitor();

        IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
        perception.calibrateLineSensors();

        INavigation navigation = new NavigationAT(perception, monitor);
        IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);

        monitor.startLogging();

        // Main loop for handling robot states
        while (true) {
            showData(navigation);

            switch (currentStatus) {
                case SCOUT:
                    // Handle SCOUT mode
                    if (lastStatus != CurrentStatus.SCOUT) {
                        control.setCtrlMode(ControlMode.LINE_CTRL);
                    }

                    // Update navigation and check for lap completion
                    navigation.updateNavigation();
                    checkLapCompletion(navigation);

                    lastStatus = currentStatus;

                    // Handle state transitions
                    if (Button.ENTER.isDown()) {
                        currentStatus = CurrentStatus.PAUSE;
                        while (Button.ENTER.isDown()) {
                            Thread.sleep(1); // Wait for button release
                        }
                    } else if (Button.ESCAPE.isDown()) {
                        currentStatus = CurrentStatus.EXIT;
                        while (Button.ESCAPE.isDown()) {
                            Thread.sleep(1); // Wait for button release
                        }
                    } else if (Button.LEFT.isDown()) {
                        currentStatus = CurrentStatus.PARK;
                        while (Button.LEFT.isDown()) {
                            Thread.sleep(1); // Wait for button release
                        }
                    }
                    break;

                case PAUSE:
                    // Handle PAUSE mode
                    if (lastStatus != CurrentStatus.PAUSE) {
                        control.setCtrlMode(ControlMode.INACTIVE);
                    }

                    lastStatus = currentStatus;

                    // Handle state transitions
                    if (Button.ENTER.isDown()) {
                        currentStatus = CurrentStatus.SCOUT;
                        while (Button.ENTER.isDown()) {
                            Thread.sleep(1); // Wait for button release
                        }
                    } else if (Button.ESCAPE.isDown()) {
                        currentStatus = CurrentStatus.EXIT;
                        while (Button.ESCAPE.isDown()) {
                            Thread.sleep(1); // Wait for button release
                        }
                    }
                    break;

                case PARK:
                    // Handle PARK mode
                    if (lapCounter >= 1) {
                        parkInSlot(navigation, control);
                    } else {
                        Sound.playTone(500, 500); // Beep to indicate parking is unavailable
                        LCD.drawString("Complete 1 lap", 0, 4);
                    }
                    currentStatus = CurrentStatus.PAUSE;
                    break;

                case EXIT:
                    // Handle EXIT mode
                    monitor.stopLogging();
                    System.exit(0);
                    break;

                default:
                    break;
            }

            Thread.sleep(100);
        }
    }

    /**
     * Returns the current robot status.
     */
    public static CurrentStatus getCurrentStatus() {
        return currentStatus;
    }

    /**
     * Displays the robot's current pose on the LCD screen.
     */
    protected static void showData(INavigation navigation) {
        Pose pose = navigation.getPose();
        LCD.clear();

        LCD.drawString("X (cm): " + (pose.getX() * 100), 0, 0);
        LCD.drawString("Y (cm): " + (pose.getY() * 100), 0, 1);
        LCD.drawString("Phi: " + (pose.getHeading() / Math.PI * 180), 0, 2);
        LCD.drawString("Lap: " + lapCounter, 0, 3);
    }

    /**
     * Checks for lap completion by monitoring robot position.
     */
    private static void checkLapCompletion(INavigation navigation) {
        Pose pose = navigation.getPose();
        long currentTime = System.currentTimeMillis();

        if (pose.getX() < 5 && pose.getY() < 5 && (currentTime - lastLapTime) > 5000) {
            lastLapTime = currentTime; // Update last lap time
            lapCounter++;
            Sound.playTone(1000, 500); // Beep to indicate a new lap
        }
    }

    /**
     * Parks in the first suitable parking slot.
     */
    private static void parkInSlot(INavigation navigation, IControl control) {
        ParkingSlot[] slots = navigation.getParkingSlots();
        if (slots != null && slots.length > 0) {
            for (ParkingSlot slot : slots) {
                if (slot.getStatus() == ParkingSlot.ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                    float targetX = (slot.getBackBoundaryPosition().x + slot.getFrontBoundaryPosition().x) / 2;
                    float targetY = (slot.getBackBoundaryPosition().y + slot.getFrontBoundaryPosition().y) / 2;

                    control.setDestination(0, targetX, targetY);
                    Sound.playTone(2000, 500); // Signal parking start

                    while (true) {
                        Pose pose = navigation.getPose();
                        double distanceToTarget = Math.sqrt(
                            Math.pow(targetX - pose.getX(), 2) + Math.pow(targetY - pose.getY(), 2)
                        );

                        if (distanceToTarget < 0.1) { // Threshold to consider parking complete
                            break;
                        }

                        try {
                            Thread.sleep(50); // Small delay to avoid busy waiting
                        } catch (InterruptedException e) {
                            System.out.println("Interrupted during parking");
                        }
                    }

                    Sound.playTone(3000, 500); // Signal parking end
                    return;
                }
            }
        }
        Sound.playTone(500, 500); // No suitable parking slot found
    }
}
