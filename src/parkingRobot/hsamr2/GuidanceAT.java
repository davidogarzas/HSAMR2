package parkingRobot.hsamr2;

// Guidance module imports
import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import lejos.geom.Line;
import lejos.nxt.LCD;

import parkingRobot.INxtHmi;
import parkingRobot.IControl;
import parkingRobot.IControl.ControlMode;
import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

public class GuidanceAT {

    public enum CurrentStatus {
        SCOUT,
        PAUSE,
        PARK,
        AUSPARKEN,
        EXIT,
        //DEMO1,
        //DEMO2
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
    
    //local Mode variable to set hmi mode
    private static INxtHmi.Mode currentMode = INxtHmi.Mode.PAUSE;

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
        INxtHmi  	hmi        = new HmiPLT(perception, navigation, control, monitor);

        monitor.startLogging();

        while (true) {
            showData(navigation, perception);
            
         // Update the HMI mode based on the current state
            updateHmiMode(monitor);

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
                    
                case AUSPARKEN:
                	handleAusparkenMode(navigation, control);
                	break;
                    
                 // --ADDED FOR DEMO--
                /*case DEMO1:
                    handleDemo1Mode(control);
                    break;

                // --ADDED FOR DEMO--
                case DEMO2:
                    handleDemo2Mode(control);
                    break;*/

                default:
                    break;
            }

            Thread.sleep(100);
        }
    }
    
    // --ADDED FOR DEMO--
    /*private static void handleDemo1Mode(IControl control) throws InterruptedException {
        // Enter action
        if (lastStatus != CurrentStatus.DEMO1) {
            control.setCtrlMode(ControlMode.DEMO_PRG1); // or however you name it
        }

        lastStatus = currentStatus;

        // Check transitions
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
        }
    }
    
    // --ADDED FOR DEMO--
    private static void handleDemo2Mode(IControl control) throws InterruptedException {
        // Enter action
        if (lastStatus != CurrentStatus.DEMO2) {
            control.setCtrlMode(ControlMode.DEMO_PRG2);
        }

        lastStatus = currentStatus;

        // Check transitions
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
        }
    }*/
    
 // Update HMI mode
    private static void updateHmiMode(IMonitor monitor) {
        // Determine the mode based on the current status
        switch (currentStatus) {
            case SCOUT:
                currentMode = INxtHmi.Mode.SCOUT;
                break;
            case PARK:
                currentMode = INxtHmi.Mode.PARK_NOW;
                break;
            case AUSPARKEN:
            case PAUSE:
                currentMode = INxtHmi.Mode.PAUSE;
                break;
            case EXIT:
                currentMode = INxtHmi.Mode.DISCONNECT;
                break;
            default:
                currentMode = INxtHmi.Mode.PAUSE;
                break;
        }

        // Log the updated mode for monitoring
        monitor.writeGuidanceComment("Updated HMI Mode: " + currentMode.name());
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
            currentStatus = CurrentStatus.PARK; //se cambia a .park o a .demo1 dependiendo de la prueba
            while (Button.RIGHT.isDown()) {
                Thread.sleep(1);
            }
        }
        //ausparken is not accessible from scout, doesn't make sense
        /*else if (Button.LEFT.isDown()) {
            currentStatus = CurrentStatus.DEMO2; //este se comenta o descomenta dependiendo de si es prueba de control o guidance
            while (Button.LEFT.isDown()) { Thread.sleep(1); }
        }*/
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
        //checks for demo1
        else if (Button.RIGHT.isDown()) {
        currentStatus = CurrentStatus.PARK; //cambiar dependiendo de si es park o demo1
        while (Button.RIGHT.isDown()) Thread.sleep(1);

    // --- NEW: Jump to DEMO2 with left button ---
    } else if (Button.LEFT.isDown()) {
        currentStatus = CurrentStatus.AUSPARKEN; //cambiar a demo2 o a ausparken
        while (Button.LEFT.isDown()) Thread.sleep(1);
    }
        
    }

    private static void handleParkMode(INavigation navigation, IControl control) throws InterruptedException {
        // Disable odometry-based corrections to avoid conflicts
        navigation.setUseOnlyOdometry(false);

        // Get all available parking slots
        INavigation.ParkingSlot[] slots = navigation.getParkingSlots();

        if (slots != null && slots.length > 0) {
            // Select the first suitable parking slot
            INavigation.ParkingSlot targetSlot = null;
            for (INavigation.ParkingSlot slot : slots) {
                if (slot.getStatus() == ParkingSlotStatus.SUITABLE_FOR_PARKING) {
                    targetSlot = slot;
                    break;
                }
            }

            if (targetSlot != null) {
                double middleX = (targetSlot.getFrontBoundaryPosition().x + targetSlot.getBackBoundaryPosition().x) / 2;
                double middleY = (targetSlot.getFrontBoundaryPosition().y + targetSlot.getBackBoundaryPosition().y) / 2;

                LCD.clear();
                LCD.drawString("Parking...", 0, 3);

                control.setDestination(0, middleX, middleY);

                while (Math.abs(navigation.getPose().getX() - middleX) > 0.1
                        || Math.abs(navigation.getPose().getY() - middleY) > 0.1) {
                    navigation.updateNavigation();
                    Thread.sleep(100);
                }
            // Stop the robot at the middle point
            control.setCtrlMode(IControl.ControlMode.INACTIVE);

            // Display "Parking done" and beep once
            LCD.clear();
            LCD.drawString("Parking done", 0, 3);
            Sound.beepSequence(); // Plays the beep sequence exactly once
            Thread.sleep(1000); // Ensure the beeping completes
        } else {
            // Handle case where no parking slots are detected
            LCD.clear();
            LCD.drawString("No parking slots!", 0, 3);
        }

        // Allow the user to transition to "ausparken" mode using the LEFT button
        if (Button.LEFT.isDown()) {
            currentStatus = CurrentStatus.AUSPARKEN;
            while (Button.LEFT.isDown()) {
                Thread.sleep(1); // Wait for button release
            }
        }
        // Allow the user to exit using the ESCAPE button
        else if (Button.ESCAPE.isDown()) {
            currentStatus = CurrentStatus.EXIT;
            while (Button.ESCAPE.isDown()) {
                Thread.sleep(1); // Wait for button release
            }
        }
    }
    }

    private static void handleAusparkenMode(INavigation navigation, IControl control) throws InterruptedException {
        // Display "Ausparken" for 3 seconds
        LCD.clear();
        LCD.drawString("Ausparken...", 0, 3);
        Thread.sleep(3000); // Wait for 3 seconds

        // Transition to SCOUT mode immediately
        LCD.clear();
        LCD.drawString("Scout Mode", 0, 3);

        // Directly initialize SCOUT mode logic
        currentStatus = CurrentStatus.SCOUT;
        lastStatus = CurrentStatus.AUSPARKEN; // Update lastStatus for consistency
        Thread.sleep(1000); // Allow transition message to display

        // Automatically start SCOUT mode
        control.setCtrlMode(IControl.ControlMode.LINE_CTRL);
        navigation.setDetectionState(true);
        navigation.setUseOnlyOdometry(true);
    }

    public static CurrentStatus getCurrentStatus() {
        return currentStatus;
    }

    public static void setCurrentStatus(CurrentStatus status) {
        currentStatus = status;
    }

    protected static void showData(INavigation navigation, IPerception perception) {
        LCD.clear();
        LCD.drawString("X (cm): " + (navigation.getPose().getX() * 100), 0, 0);
        LCD.drawString("Y (cm): " + (navigation.getPose().getY() * 100), 0, 1);
        LCD.drawString("Phi: " + (navigation.getPose().getHeading() / Math.PI * 180), 0, 2);
        
        perception.showSensorData();
    }
}