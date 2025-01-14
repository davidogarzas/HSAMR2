package parkingRobot.hsamr2;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

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
        navigation.setMap(map);

        IControl control = new ControlRST(perception, navigation, leftMotor, rightMotor, monitor);

        monitor.startLogging();

        // Main loop for handling robot states
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
                    handleParkMode(navigation, control);
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

    /**
     * Handles the SCOUT mode logic, including line following and parking slot detection.
     */
    private static void handleScoutMode(INavigation navigation, IControl control) throws InterruptedException {
        if (lastStatus != CurrentStatus.SCOUT) {
            control.setCtrlMode(ControlMode.LINE_CTRL); // Follow the line
            navigation.setDetectionState(true); // Enable parking slot detection
        }

        // Update navigation to process slot detection
        navigation.updateNavigation();
        
        lastStatus = currentStatus;

        // Handle button transitions
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
        } else if (Button.RIGHT.isDown()) {
            currentStatus = CurrentStatus.PARK;
            while (Button.RIGHT.isDown()) {
                Thread.sleep(1); // Wait for button release
            }
        }
    }

    /**
     * Handles the PAUSE mode logic.
     */
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
    
    /**
     * Handles the PARK mode logic.
     */
    private static void handleParkMode(INavigation navigation, IControl control) throws InterruptedException {
        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();
        	
        	INavigation.ParkingSlot targetSlot = slots[0]; // Use the first mock slot for testing
            control.setCtrlMode(ControlMode.LINE_CTRL); 
            /*
            control.setDestination(
                0, // Heading (angle)
                targetSlot.getFrontBoundaryPosition().x,
                targetSlot.getFrontBoundaryPosition().y
            );
*/
            LCD.drawString("Slot ID: " + targetSlot.getID(), 0, 3);
            LCD.drawString("BX: " + targetSlot.getBackBoundaryPosition().x * 100, 0, 4);
            LCD.drawString("BY: " + targetSlot.getBackBoundaryPosition().y * 100, 0, 5);
            LCD.drawString("FX: " + targetSlot.getFrontBoundaryPosition().x * 100, 0, 6);
            LCD.drawString("FY: " + targetSlot.getFrontBoundaryPosition().y * 100, 0, 7);
            
            if (Math.abs(navigation.getPose().getX() - (targetSlot.getFrontBoundaryPosition().x + 0.1)) <= 0.01
            		&& Math.abs(navigation.getPose().getY() - targetSlot.getFrontBoundaryPosition().y) <= 0.1){
            	
            	currentStatus = CurrentStatus.PAUSE;	
            
            }
            //control.setCtrlMode(ControlMode.PARK_CTRL);
	        else {
	            LCD.drawString("No slots found", 0, 3);
	            Sound.buzz(); // Indicate failure
	            currentStatus = CurrentStatus.SCOUT;
	        }
	
	        if (Button.ESCAPE.isDown()) {
	            currentStatus = CurrentStatus.EXIT;
	            while (Button.ESCAPE.isDown()) {
	                Thread.sleep(1); // Wait for button release
	            }
	        }
    }

    /**
     * Returns the current robot status.
     */
    public static CurrentStatus getCurrentStatus() {
        return currentStatus;
    }

    /**
     * Sets the current robot status.
     */
    public static void setCurrentStatus(CurrentStatus status) {
        currentStatus = status;
    }

    /**
     * Displays the robot's current pose on the LCD screen.
     */
    protected static void showData(INavigation navigation) {
        LCD.clear();

        LCD.drawString("X (cm): " + (navigation.getPose().getX() * 100), 0, 0);
        LCD.drawString("Y (cm): " + (navigation.getPose().getY() * 100), 0, 1);
        LCD.drawString("Phi: " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
        
    }
}
