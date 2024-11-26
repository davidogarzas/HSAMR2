package parkingRobot.hsamr2;

import java.util.LinkedList;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.hsamr2.NavigationThread;

/**
 * A executable basic example implementation of the corresponding interface provided by the 
 * Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are 
 * used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). 
 * Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, 
 * although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed 
 * and invented by the students.  
 * 
 * @author IfA
 */

public class NavigationAT implements INavigation{
	
	// Custom Variables
	int currentLine = 0;
	int lapNumber = 0;
	double currentLineAngle = 0;
	double nextLineAngle = 0;
	double lastAngleResult = 0;
	boolean initialize = false;
	LinkedList<ParkingSlot> parkingSlotsList = new LinkedList<ParkingSlot>();
	int parallelCounter = 0;
	
	// For parkingSlots
	float beginX = 0;
	float beginY = 0;
	Point parkingSlotBegin = new Point(0,0);
	Point parkingSlotEnd = new Point(0,0);
	float finalX = 0;
	float finalY = 0;
	int state = 0;
	double sizeRobot = 30/100; // Check size
	int parkingSlotID = 1;
	
	/**
	 * line information measured by light sensor: 0 - beside line, 1 - on line border or 
	 * gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	int lineSensorLeft	=	0;
		
	/**
	 * reference to {@link IPerception.EncoderSensor} class for robot wheels which 
	 * measures the wheels angle difference between actual and last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the wheels angle difference between actual 
	 * and last request and the corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure 
	 * the ground displacement between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the 
	 * ground displacement between actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensors pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm 
	 * (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm 
	 * (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;

	/**
	 * robot specific constant: radius of wheels
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.028; 
	static final double RIGHT_WHEEL_RADIUS	= 	0.028; 
	// only rough guess, to be measured exactly and maybe refined by experiments
	
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.138; 
	// only rough guess, to be measured exactly and maybe refined by experiments

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain 
	 * and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception 
	 * object (to obtain the measurement information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		// MONITOR VARIABLES
		monitor.addNavigationVar("X");
		monitor.addNavigationVar("Y");
		monitor.addNavigationVar("Phi");
		monitor.addNavigationVar("PhiError");
		
		// INIT MAP
		setMap(GuidanceAT.map);
		this.currentLineAngle = 0;
		
		
		this.nextLineAngle = this.map[1].getP1().angleTo(this.map[1].getP2());
				
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); 
		// background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
	}
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		
		return this.parkingSlotsList.toArray(new ParkingSlot[this.parkingSlotsList.size()]);
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes 
	 * the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		
		// Variable Declarations
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		
		double xMap			= 0;
		double yMap			= 0;
		
		double angleResult 	= 0;	
				
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		// Calculate Odometry with Encoders
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
		} else {			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= Math.toDegrees(this.pose.getHeading() + w * deltaT);
		}
		
		// Correction of XY Coordinates according to Map
		
		// Detects change of line (may change method of detection)
		if (Math.abs(angleResult - this.nextLineAngle) <= 20){
			
			// Updates Current Line Index
			// Resets line number when finishing the lap
			if (currentLine < this.map.length - 1){
				currentLine++;
				
				// Increases lap number when it is on the last line
				// (Not when it reaches the first line)
				if (currentLine == this.map.length - 1){this.lapNumber++;}
			}
			else {
				currentLine = 0;
			}
			
			// Saves Current Line Angle
			this.currentLineAngle = this.nextLineAngle;
			
			// Calculates Next Line Angle
			if (currentLine == this.map.length - 1){this.nextLineAngle = this.map[0].getP1().angleTo(this.map[0].getP2());} 
			else {this.nextLineAngle = this.map[currentLine+1].getP1().angleTo(this.map[currentLine+1].getP2());}
			
			// Turns all angles positive and can go above 360°
			if (this.nextLineAngle < 0){this.nextLineAngle += 360;}
			this.nextLineAngle = this.nextLineAngle + 360*this.lapNumber;
			
			// Gets X Y from Map Coordinates
			xMap = this.map[currentLine].getX1() / 100;  // Turns to cm
			yMap = this.map[currentLine].getY1() / 100;  // Turns to cm
			
			// Prints info
			monitor.writeNavigationComment("Now on Line " + currentLine);
			monitor.writeNavigationComment("Estimated X: " + String.valueOf(xResult * 100) + " Y: " + String.valueOf(yResult * 100));
			monitor.writeNavigationComment("Map X: " + String.valueOf(this.map[currentLine].getX1()) + " Y: " + String.valueOf(this.map[currentLine].getY1()));
			
			// Updates X Y to Map Coordinates
			xResult = xMap;
			yResult = yMap;
		}
		
		// Correction of angle using SideDistanceSensors
		
		// If both line sensors are on white (going straight)
		if (this.lineSensorLeft == 0 && this.lineSensorRight == 0){
			
			// If sensors are measuring something
			if (this.frontSideSensorDistance != 30 && this.backSideSensorDistance != 30){
				
				// If sensors are parallel to wall (Measure same distance)
				if  (Math.abs(this.frontSideSensorDistance - this.backSideSensorDistance) <= 0){
					
					// If they have measured the same distance at least 3 times
					if (this.parallelCounter < 3){this.parallelCounter++;}	
					else if (this.parallelCounter >= 3){
						
						// Resets counter
						this.parallelCounter = 0;

						// If angle error is big enough
						if (Math.abs(this.currentLineAngle - angleResult) >= 5){
							
							// Updates angleResult
							monitor.writeNavigationComment("Corrected Angle from: " + String.valueOf(angleResult) + " to: " + String.valueOf(this.currentLineAngle));
							angleResult = this.currentLineAngle;
						}
					}
				
				// Resets counter if they have not been consecutive equal measurements
				} else {this.parallelCounter = 0;}	
			}
		}
	
		// MONITOR (example)
		monitor.writeNavigationVar("X", "" + (xResult * 100));
		monitor.writeNavigationVar("Y", "" + (yResult * 100));
		monitor.writeNavigationVar("Phi", "" + angleResult);	
		monitor.writeNavigationVar("PhiError", "" + (angleResult - this.currentLineAngle));	
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);
		
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		/*
		// Variables

		boolean newSlot = true;
		
		// state 0 = Looking for Possible Slot
		// state 1 = May have found possible slot
		
		// Looking for Possible Slot
		if (this.state == 0){
				if (this.frontSideSensorDistance >= this.sizeRobot*100){
					this.state = 1;
					
					// Saves Back Coordinates of Parking Slot
					this.parkingSlotBegin.setLocation(this.pose.getX(),this.pose.getY());
				}
				
		// May Have Found Possible Slot
		} else if (this.state == 1){
			
			// Did not find possibleParkingSlot
			// Did not cover enough distance before FrontSideSensor stopped detecting enough 
			// depth for robot
			if (Math.abs(this.pose.distanceTo(this.parkingSlotBegin)) < this.sizeRobot*100 
				&& this.frontSideSensorDistance < this.sizeRobot*100){
				
				this.state = 0;
				// Look through array, see if first coordinate was saved and update slot
			}
			
			// Found possible parkingSlot
			// Covered enough distance in X or Y so that frontSideSensor and BackSideSensor
			// both detect enough depth for robot
			else if (this.pose.distanceTo(this.parkingSlotBegin) >= this.sizeRobot*100 
					&& this.frontSideSensorDistance >= this.sizeRobot*100 
					&& this.backSideSensorDistance >= this.sizeRobot*100){
				
				this.state = 0;
				
				// Saves Front Coordinates of ParkingSlot
				this.parkingSlotEnd.setLocation(this.pose.getX(),this.pose.getY());
				
				// Check existing array
				for (int i = 0; i < this.parkingSlotArray.length; i++) {
					
					// Parking Slot already exists
					if (this.parkingSlotArray[i].getFrontBoundaryPosition().distance(this.parkingSlotEnd) < 5
						&& this.parkingSlotArray[i].getBackBoundaryPosition().distance(this.parkingSlotBegin) < 5){	
						// Update parking slot
						newSlot = false;
						break;
					}
				}
				
				// If Parking Slot didnt already exist
				if (newSlot){
					ParkingSlot parkingSlotObj = new ParkingSlot(
							this.parkingSlotID, 
							this.parkingSlotBegin, 
							this.parkingSlotEnd, 
							ParkingSlotStatus.SUITABLE_FOR_PARKING, 
							10);
					// Add to array
				}
			}
		}
		*/
		return; // has to be implemented by students
	}
}
