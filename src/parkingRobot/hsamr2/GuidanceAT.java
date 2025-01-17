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
            navigation.setUseOnlyOdometry(true);
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
    	//disable odometry to avoid conflicts
    	 navigation.setUseOnlyOdometry(false);
        // Get all available parking slots
        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();

       /* if (slots != null && slots.length > 0) {
            // Select the first slot as the target (or implement a selection mechanism if needed)
            INavigation.ParkingSlot targetSlot = slots[0];

            // Step 1: Calculate the middle point
            double middleX = (targetSlot.getFrontBoundaryPosition().x + targetSlot.getBackBoundaryPosition().x) / 2;
            double middleY = (targetSlot.getFrontBoundaryPosition().y + targetSlot.getBackBoundaryPosition().y) / 2;

            // Display middle point details on LCD
            LCD.clear();
            LCD.drawString("Middle X: " + (middleX * 100) + " cm", 0, 3);
            LCD.drawString("Middle Y: " + (middleY * 100) + " cm", 0, 4);

            // Step 2: Drive directly towards the middle point
            control.setDestination(0, middleX, middleY);

            // Wait until the robot reaches the middle point
            while (Math.abs(navigation.getPose().getX() - middleX) > 0.1
                    || Math.abs(navigation.getPose().getY() - middleY) > 0.1) {
                navigation.updateNavigation();
                Thread.sleep(100); // Allow time for movement and updates
            }*/

            // Parking sequence complete: Robot reaches middle point
            //Sound.beepSequenceUp(); // Feedback to indicate reaching the middle point

         // Step 3: Perform a 90-degree turn
            LCD.clear();
            LCD.drawString("Turning 90 degrees", 0, 3);

            // Get the current heading
            double initialHeading = navigation.getPose().getHeading();

            // Desired heading after a 90 degree turn (in radians)
            double targetHeading = initialHeading + Math.PI / 2;
            /*if (targetHeading > 2 * Math.PI) {
                targetHeading -= 2 * Math.PI; // Ensure heading stays within [0, 2pi]
            }*/

            // Stop any ongoing control to take direct control of the motors
            control.setCtrlMode(IControl.ControlMode.INACTIVE); 

            // Access motors directly
            NXTMotor leftMotor = ((ControlRST) control).leftMotor; 
            NXTMotor rightMotor = ((ControlRST) control).rightMotor;

            // Set motors for turning: Keep the left motor stationary and rotate the right motor
            leftMotor.setPower(0); // stop the left motor
            rightMotor.setPower(40); // Set power to the right motor
            rightMotor.forward(); // Start turning the robot

            // Keep turning until the robot reaches the target heading or timeout
            /*while ((Math.abs(navigation.getPose().getHeading() - targetHeading) > 0.05)) {
                navigation.updateNavigation(); // Update the navigation to get the latest pose
                Thread.sleep(50); // Allow time for pose updates
            }*/

            // Stop the motors after turning
           // leftMotor.stop();
            //rightMotor.stop();

 
           // Sound.beepSequence(); // Feedback to indicate turn completion
            //LCD.drawString("Turn Complete", 0, 6);
     /*   } else {
            // No parking slots available
            LCD.drawString("No parking slots detected!", 0, 3);
        }

        // Allow the user to exit PARK mode
        if (Button.ESCAPE.isDown()) {
            currentStatus = CurrentStatus.EXIT;
            while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); // Wait for button release
            }
        }*/
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
}