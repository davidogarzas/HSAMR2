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
                    handleParkMode(navigation, perception, control); // Pass perception here
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

    private static void handleParkMode(INavigation navigation, IPerception perception, IControl control) throws InterruptedException {
        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();

        if (slots != null && slots.length > 0) {
            // Select the first parking slot for demonstration
            INavigation.ParkingSlot targetSlot = slots[0];
            double targetX = targetSlot.getFrontBoundaryPosition().x + 0.1; // Move 10 cm forward from the slot
            double targetY = targetSlot.getFrontBoundaryPosition().y;

            // Display parking slot details
            LCD.drawString("Slot ID: " + targetSlot.getID(), 0, 3);
            LCD.drawString("BX: " + targetSlot.getBackBoundaryPosition().x * 100, 0, 4);
            LCD.drawString("BY: " + targetSlot.getBackBoundaryPosition().y * 100, 0, 5);
            LCD.drawString("FX: " + targetSlot.getFrontBoundaryPosition().x * 100, 0, 6);
            LCD.drawString("FY: " + targetSlot.getFrontBoundaryPosition().y * 100, 0, 7);

            // Move forward 10 cm (always)
            control.setDestination(0, targetX, targetY);
            while (Math.abs(navigation.getPose().getX() - targetX) > 0.01 || Math.abs(navigation.getPose().getY() - targetY) > 0.01) {
                // Check for collisions
                if (perception.getBackSensorDistance() < 0.1 || perception.getFrontSensorDistance() < 0.1) { // 10 cm
                    Sound.twoBeeps(); // Collision warning
                    currentStatus = CurrentStatus.PAUSE;
                    return; // Exit the parking logic if a collision is imminent
                }
                Thread.sleep(50); // Allow time for the robot to adjust position
            }

            // Beep to signal reaching 10 cm forward
            Sound.beep();

            // Now execute the parking logic using TrajectoryGenerator
            double startX = navigation.getPose().getX();
            double startY = navigation.getPose().getY();
            double endX = targetSlot.getBackBoundaryPosition().x;
            double endY = targetSlot.getBackBoundaryPosition().y;

            TrajectoryGenerator trajectory = new TrajectoryGenerator(startX, startY, endX, endY);

            double currentX = startX;
            while (Math.abs(currentX - endX) > 0.01) { // Loop until close to the parking end position
                double[] nextPoint = trajectory.getNextPoint(currentX);
                control.setDestination(0, nextPoint[0], nextPoint[1]); // Set the next trajectory point
                currentX = nextPoint[0];

                // Check for collisions
                if (perception.getBackSensorDistance() < 0.1 || perception.getFrontSensorDistance() < 0.1) { // 10 cm
                    Sound.twoBeeps(); // Collision warning
                    currentStatus = CurrentStatus.PAUSE;
                    return;
                }
                Thread.sleep(50); // Small delay for smooth motion
            }

            // Signal that parking is complete
            currentStatus = CurrentStatus.PAUSE;
            Sound.beep();
        } else {
            LCD.drawString("No slots available", 0, 3);
        }

        // Allow user to exit the parking mode
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