package parkingRobot.hsamr2;

// Guidance module imports
import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

public class GuidanceAT {

    public enum CurrentStatus {
        SCOUT,
        PAUSE,
        PARK,
        EXIT
    }

    protected static CurrentStatus currentStatus = CurrentStatus.PAUSE;
    protected static CurrentStatus lastStatus = CurrentStatus.EXIT;

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

        currentStatus = CurrentStatus.PAUSE;
        lastStatus = CurrentStatus.EXIT;

        NXTMotor leftMotor = new NXTMotor(MotorPort.B);
        NXTMotor rightMotor = new NXTMotor(MotorPort.A);

        IMonitor monitor = new Monitor();

        IPerception perception = new PerceptionPMP(leftMotor, rightMotor, monitor);
        perception.calibrateLineSensors();

        INavigation navigation = new NavigationAT(perception, monitor);
        navigation.setMap(map);

        IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);

        monitor.startLogging();

        while (true) {
            showData(navigation);

            switch (currentStatus) {
                case SCOUT:
                    handleScoutMode(navigation, control);
                    break;

                case PAUSE:
                    handlePauseMode(control);
                    break;

                case PARK:
                    handleParkMode(navigation, control); // Pass perception here
                    break;

                case EXIT:
                    monitor.stopLogging();
                    System.exit(0);
                    break;

                default:
                    break;
            }

            Thread.sleep(100);
        }
    }

    private static void handleScoutMode(INavigation navigation, IControl control) throws InterruptedException {
        if (lastStatus != CurrentStatus.SCOUT) {
            control.setCtrlMode(ControlMode.LINE_CTRL);
            navigation.setDetectionState(true);
        }

        navigation.updateNavigation();

        lastStatus = currentStatus;

        if (Button.ENTER.isDown()) {
            currentStatus = CurrentStatus.PAUSE;
            while (Button.ENTER.isDown()) {
                Thread.sleep(1);
            }
        } else if (Button.ESCAPE.isDown()) {
            currentStatus = CurrentStatus.EXIT;
            while (Button.ESCAPE.isDown()) {
                Thread.sleep(1);
            }
        } else if (Button.RIGHT.isDown()) {
            currentStatus = CurrentStatus.PARK;
            while (Button.RIGHT.isDown()) {
                Thread.sleep(1);
            }
        }
    }

    private static void handlePauseMode(IControl control) throws InterruptedException {
        if (lastStatus != CurrentStatus.PAUSE) {
            control.setCtrlMode(ControlMode.INACTIVE);
        }

        lastStatus = currentStatus;

        if (Button.ENTER.isDown()) {
            currentStatus = CurrentStatus.SCOUT;
            while (Button.ENTER.isDown()) {
                Thread.sleep(1);
            }
        } else if (Button.ESCAPE.isDown()) {
            currentStatus = CurrentStatus.EXIT;
            while (Button.ESCAPE.isDown()) {
                Thread.sleep(1);
            }
        }
    }

    private static void handleParkMode(INavigation navigation, IControl control) throws InterruptedException {
        // Get all available parking slots
        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();

        if (slots != null && slots.length > 0) {
            // Select the first slot as the target (or implement a selection mechanism if needed)
            INavigation.ParkingSlot targetSlot = slots[0];
            
            // Define the target coordinates for the front boundary
            double frontBoundaryX = targetSlot.getFrontBoundaryPosition().x;
            double frontBoundaryY = targetSlot.getFrontBoundaryPosition().y;

            // Display target slot details on LCD
            LCD.clear();
            LCD.drawString("Parking Slot ID: " + targetSlot.getID(), 0, 3);
            LCD.drawString("Front X: " + (frontBoundaryX * 100) + " cm", 0, 4);
            LCD.drawString("Front Y: " + (frontBoundaryY * 100) + " cm", 0, 5);

            // Step 1: Drive to the front boundary position
            control.setDestination(0, frontBoundaryX, frontBoundaryY);

            // Wait until the robot reaches the front boundary position
            while (Math.abs(navigation.getPose().getX() - frontBoundaryX) > 0.1
                    || Math.abs(navigation.getPose().getY() - frontBoundaryY) > 0.1) {
                navigation.updateNavigation();
                Thread.sleep(100); // Allow time for movement and updates
            }

            // Stop at the front boundary
            control.setCtrlMode(IControl.ControlMode.INACTIVE); // Pause robot motion
            Sound.beepSequenceUp(); // Provide feedback

            // Step 2: Calculate the middle point
            double middleX = (targetSlot.getFrontBoundaryPosition().x + targetSlot.getBackBoundaryPosition().x) / 2;
            double middleY = (targetSlot.getFrontBoundaryPosition().y + targetSlot.getBackBoundaryPosition().y) / 2;

            // Display middle point details on LCD
            LCD.clear();
            LCD.drawString("Middle X: " + (middleX * 100) + " cm", 0, 3);
            LCD.drawString("Middle Y: " + (middleY * 100) + " cm", 0, 4);

            // Step 3: Drive backward to the middle point
            control.setDestination(0, middleX, middleY);

            // Wait until the robot reaches the middle point
            while (Math.abs(navigation.getPose().getX() - middleX) > 0.1
                    || Math.abs(navigation.getPose().getY() - middleY) > 0.1) {
                navigation.updateNavigation();
                Thread.sleep(100); // Allow time for movement and updates
            }

            // Parking sequence complete
            Sound.beepSequence(); // Feedback to indicate parking completion
            currentStatus = CurrentStatus.PAUSE; // Transition to PAUSE mode

        } else {
            // No parking slots available
            LCD.drawString("No parking slots detected!", 0, 3);
        }

        // Allow the user to exit PARK mode
        if (Button.ESCAPE.isDown()) {
            currentStatus = CurrentStatus.EXIT;
            while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); // Wait for button release
            }
        }
    }

    public static CurrentStatus getCurrentStatus() {
        return currentStatus;
    }

    public static void setCurrentStatus(CurrentStatus status) {
        currentStatus = status;
    }

    protected static void showData(INavigation navigation) {
        LCD.clear();
        LCD.drawString("X (cm): " + (navigation.getPose().getX() * 100), 0, 0);
        LCD.drawString("Y (cm): " + (navigation.getPose().getY() * 100), 0, 1);
        LCD.drawString("Phi: " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
    }

    /**
     * Generates a 3rd-degree trajectory for parking.
     */
    static class TrajectoryGenerator {
        private double a, b, c, d;
        private double startX, startY, endX, endY;

        public TrajectoryGenerator(double startX, double startY, double endX, double endY) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;

        }

        public double[] getNextPoint(double currentX) {
            double nextX = currentX - 0.05; // Move backward
            double nextY = a * Math.pow((nextX - startX), 3) + b * Math.pow((nextX - startX), 2) + c * (nextX - startX) + d;

            return new double[]{nextX, nextY};
        }
    }
}