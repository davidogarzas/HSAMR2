package parkingRobot.hsamr2;

import java.util.LinkedList;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
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
	int lapNumberAngleCalculation = 0;
	int lapNumber = 0;
	double currentLineAngle = 0;
	double nextLineAngle = 0;
	double lastAngleResult = 0;
	boolean initialize = false;
	LinkedList<ParkingSlot> parkingSlotsList = new LinkedList<ParkingSlot>();
	int parallelCounter = 0;
	int distanceCounter = 0;
	
	float[] lineAngles = {0,0,0,0,0,0,0,0};
	
	double prevAngle = 0;
	int line_state = 0;
	int line_state_coords = 0;
	long ref_time_coords = 0;
	long ref_time = 0;
	boolean turn_corner = false;
	// 0 = both white
	// 1 = different
	
	// For parkingSlots
	float measurementQuality = 100;
	double measurementQualityDistanceFactor = 0.001;
	float measurementQualityBegin = 100;
	float measurementQualityEnd = 100;
	float beginX = 0;
	float beginY = 0;
	Point parkingSlotBegin = new Point(0,0);
	Point parkingSlotEnd = new Point(0,0);
	float finalX = 0;
	float finalY = 0;
	int state = 0;
	double sizeRobot = 25; // Check size
	double sizeParkingSpace = 45;
	int limitDistanceSensors = 15;
	int parkingSlotID = 0;
	
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
	boolean parkingSlotDetectionIsOn		= true;
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
		monitor.addNavigationVar("Lap");
		monitor.addNavigationVar("Line");
		monitor.addNavigationVar("X");
		monitor.addNavigationVar("Y");
		monitor.addNavigationVar("Phi");
		monitor.addNavigationVar("PhiError");
		
		// INIT MAP
		setMap(GuidanceAT.map);
		calculateLineAngles();
		
		
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
	
	public IPerception.AngleDifferenceMeasurement getAngleMeasuremntLeft(){
		return this.angleMeasurementLeft;
	}
	
	public IPerception.AngleDifferenceMeasurement getAngleMeasuremntRight(){
		return this.angleMeasurementRight;
	}
	
	// Private methods
	
	private void calculateLineAngles(){
		for (int i=0; i<this.map.length; i++){
			this.lineAngles[i] = this.map[i].getP1().angleTo(this.map[i].getP2());
			
			// Turns all angles positive (from 0° to 360°)
			if (this.lineAngles[i] < 0){this.lineAngles[i] += 360;}
		}
		
		// Initiates currentLineAngle
		this.currentLineAngle = this.lineAngles[0];

	}
	
	private void detectCorner(int method, double angleResult){
		
		// Method 1
		// Detect corner when the change in angle is big enough while a line sensor detects black
		if (method == 1){
			if (this.line_state == 0){
				if ((this.lineSensorLeft == 2 && this.lineSensorRight == 0)
					|| (this.lineSensorLeft == 0 && this.lineSensorRight == 2)){
					this.line_state = 1;
					this.prevAngle = angleResult;
				}
			} 
			
			else if (this.line_state == 1){
				if (this.lineSensorLeft == 0 && this.lineSensorRight == 0){
					this.line_state = 0;
					if (Math.abs((angleResult - this.prevAngle)) >= 70){
						this.turn_corner = true; 
					}
					this.prevAngle = angleResult;
				}
			}
		}
		
		// Method 2
		// Detects corner when a lineSensor has detected black for a certain time period
		else if (method == 2){
			if (this.line_state == 0){
				if ((this.lineSensorLeft == 2 && this.lineSensorRight == 0)
					|| (this.lineSensorLeft == 0 && this.lineSensorRight == 2)){
					this.line_state = 1;
					this.ref_time = System.currentTimeMillis();
				}
			} 
			
			else if (this.line_state == 1){
				if (this.lineSensorLeft == 0 && this.lineSensorRight == 0){
					this.line_state = 0;
					if (Math.abs((System.currentTimeMillis() - this.ref_time)) >= 200){
						this.turn_corner = true; 
					}
					this.ref_time = System.currentTimeMillis();
				}
			}
		}
		
		// Method 3
		// Detects change of line/ corner turn 
		// when it is 20° from reaching the desired angle of the next line		
		else if (method == 3){
			if (Math.abs(angleResult - this.nextLineAngle) <= 20){
				this.turn_corner = true;
			}
		}
	}
	
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

		// Converts to cm
		this.frontSensorDistance	= perception.getFrontSensorDistance()/10;
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance()/10;
		this.backSensorDistance		= perception.getBackSensorDistance()/10;
		this.backSideSensorDistance	= perception.getBackSideSensorDistance()/10;
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
		double angleResult  = 0;
						
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
			angleResult 	= this.pose.getHeading() + w * deltaT;
		}
		
		// Updates measurementQuality
		this.measurementQuality -= this.measurementQualityDistanceFactor;
		
		// Turns angle to degrees
		angleResult = Math.toDegrees(angleResult);
		
		// Correction of XY coordinates according to lineSensors
		if (this.line_state_coords == 0){
			
			// Both Line Sensors Detect White
			if (this.lineSensorLeft == 0 && this.lineSensorRight == 0){
				this.line_state_coords = 1;
				this.ref_time_coords = System.currentTimeMillis();
			}
		}
		
		else if (this.line_state_coords == 1){
			
			// One of the Line Sensors doesnt detect White (Robot is not going straight)
			if (this.lineSensorLeft != 0 || this.lineSensorRight != 0){
				this.line_state_coords = 0;
				this.ref_time_coords = System.currentTimeMillis();
			}
			
			// 2 seconds have passed with robot going straight
			else if (Math.abs(System.currentTimeMillis() - this.ref_time_coords) >= 2000){
				this.line_state_coords = 0;
				this.ref_time_coords = System.currentTimeMillis();
				Sound.playTone(260,100); // C4
				
				// Corrects X if line is horizontal
				if (this.map[this.currentLine].getX1() == this.map[this.currentLine].getX2()){
					xResult = this.map[this.currentLine].getX1()/100; //m
					
				// Corrects Y if line is vertical
				} else if (this.map[this.currentLine].getY1() == this.map[this.currentLine].getY2()){
					yResult = this.map[this.currentLine].getY1()/100;
				}
				
				// Corrects angle to currentLineAngle
				angleResult = this.currentLineAngle;
			}
		}
		
		// Detects Corner Turns
		// updates this.turn_corner
		detectCorner(3, angleResult);
		
		// Correction of XY Coordinates After Corner Turn
		if (this.turn_corner){
			
			this.turn_corner = false;
			
			// Updates angle of finished line (on next lap, it will need +360°)
			this.lineAngles[this.currentLine] += 360;
			
			// Updates Current Line Index of the Map
			// Increases counter by one as long as the robot wasn't on the last line
			if (this.currentLine < this.map.length - 1){this.currentLine++;}
			
			// Resets line number and increases lap number if the robot finished the last line
			else {
				this.currentLine = 0;
				this.lapNumber++;
			}
			
			// Updates Current Line Angle (the angle of the current line)
			this.currentLineAngle = this.lineAngles[this.currentLine];
			
			// Updates Next Line Angle (the angle of the next line)
			if (this.currentLine < this.map.length - 1){
				this.nextLineAngle = this.lineAngles[this.currentLine + 1];
			} 
			
			else {
				this.nextLineAngle = this.lineAngles[0];
			}
			
			
			// Gets X Y from Map Coordinates
			xMap = this.map[currentLine].getX1();  // cm
			yMap = this.map[currentLine].getY1();  // cm
			
			// Considers the turning radius of robot for coordinate correction
			if (this.currentLineAngle == 0 + 360*this.lapNumber) {xMap += 5;}
			else if (this.currentLineAngle == 90 + 360*this.lapNumber) {yMap += 5;}
			else if (this.currentLineAngle == 180 + 360*this.lapNumber) {xMap -= 5;}
			else if (this.currentLineAngle == 270 + 360*this.lapNumber) {yMap -= 5;}
			
			// Plays sound
			Sound.playTone(130,100); // C3
			
			// Prints info
			monitor.writeNavigationComment("Now on Line " + currentLine);
			monitor.writeNavigationComment("Lap: " + this.lapNumber);
			monitor.writeNavigationComment("Estimated X: " + String.valueOf(xResult * 100) + " Y: " + String.valueOf(yResult * 100));
			monitor.writeNavigationComment("Map X: " + String.valueOf(xMap) + " Y: " + String.valueOf(yMap));
			monitor.writeNavigationComment("Current Line Angle: " + this.currentLineAngle);
			monitor.writeNavigationComment("Next Line Angle: " + this.nextLineAngle);
			
			// Updates X Y to Map Coordinates
			xResult = xMap / 100; // m
			yResult = yMap / 100; // m
			
			// Resets MeasurementQuality
			this.measurementQuality = 10;
		}
		
		// Correction of angle using SideDistanceSensors
		
		// If both line sensors are on white (going straight)
		if (this.lineSensorLeft == 0 && this.lineSensorRight == 0
			
			// if sensors are detecting a wall
			&& this.frontSideSensorDistance < 20 && this.backSideSensorDistance < 20
			&& this.frontSideSensorDistance > 0 && this.backSideSensorDistance > 0
			
			// If both sensors are parallel to wall //mm
			&& Math.abs(this.frontSideSensorDistance - this.backSideSensorDistance) <= 1
			
			// If difference between angleResult and currentLineAngle (desired angle) is too big
			&& Math.abs(this.currentLineAngle - angleResult) >= 5){
					
				// Plays sound
				Sound.playTone(523,100); //C5
				
				// Updates angleResult
				monitor.writeNavigationComment("Corrected Angle from: " + String.valueOf(angleResult) + " to: " + String.valueOf(this.currentLineAngle));
				angleResult = this.currentLineAngle;
				
		}
	
		// MONITOR (example)
		monitor.writeNavigationVar("Lap", "" + this.lapNumber);
		monitor.writeNavigationVar("Line", "" + this.currentLine);
		monitor.writeNavigationVar("X", "" + (xResult * 100));
		monitor.writeNavigationVar("Y", "" + (yResult * 100));
		monitor.writeNavigationVar("Phi", "" + angleResult);	
		monitor.writeNavigationVar("PhiError", "" + (angleResult - this.currentLineAngle));	
		
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)(angleResult*Math.PI/180));
		
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		// Variables
		boolean newSlot = true;
		double sizeMeasured = 0;

		// state 0 = Looking for Beginning of Possible Slot
		// state 1 = Measuring Possible Slot
		
		// Looking for Beginning of Possible Slot
		if (this.state == 0){
			
			// Sensor detects enough depth for parking space
			if (this.frontSideSensorDistance >= this.limitDistanceSensors){
				
				// Resets distanceCounter and goes to next state
				this.distanceCounter = 0;
				this.state = 1;
				
				// Plays sound
				// Sound.playTone(260,100); // DO4, 0.5s
				
				// Saves Back Coordinate of Parking Slot
				this.parkingSlotBegin.setLocation(this.pose.getX(),this.pose.getY()); // m
				
				// Saves Measurement Quality of Begin Point
				this.measurementQualityBegin = this.measurementQuality;
			}
				
		// Measuring Possible Slot
		} else if (this.state == 1){
			
			// Sensor stops detecting enough space for parking space
			if (this.frontSideSensorDistance < this.limitDistanceSensors){
				
				// Saves Front Coordinates of Parking Slot
				this.parkingSlotEnd.setLocation(this.pose.getX(), this.pose.getY());
				sizeMeasured = this.parkingSlotEnd.distance(this.parkingSlotBegin)*100; // cm
				this.measurementQualityEnd = this.measurementQuality;
				this.state = 0;
				
				// Space is too small
				if (sizeMeasured < this.sizeParkingSpace){
					
					// Plays sound
					// Sound.playTone(130,100); // DO3, 0.5s
					
					monitor.writeNavigationComment("Slot not possible");
					monitor.writeNavigationComment("Begin X: " + this.parkingSlotBegin.getX()*100 + " Y: " + this.parkingSlotBegin.getY()*100);
					monitor.writeNavigationComment("End X: " + this.parkingSlotEnd.getX()*100 + " Y: " + this.parkingSlotEnd.getY()*100);
					monitor.writeNavigationComment("Size Measured: " + sizeMeasured);					
					/*
					// Look through array, see if first coordinate was saved and update slot
					if (!this.parkingSlotsList.isEmpty()){
						
						// Check existing array
						for (int i = 0; i < this.parkingSlotsList.size(); i++) {
							
							// Parking Slot already exists
							if (this.parkingSlotsList.get(i).getBackBoundaryPosition().distance(this.parkingSlotBegin)*100 < 10){	
								
								// Update parking slot
								
								this.parkingSlotsList.get(i).setFrontBoundaryPosition(this.parkingSlotEnd.clone());
								this.parkingSlotsList.get(i).setStatus(ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING);
	
								monitor.writeNavigationComment("Changed Slot " + this.parkingSlotsList.get(i).getID());
								monitor.writeNavigationComment("Begin X: " + this.parkingSlotBegin.getX()*100 + " and Y: " + this.parkingSlotBegin.getY()*100);
								monitor.writeNavigationComment("End X: " + this.parkingSlotEnd.getX()*100 + " and Y: " + this.parkingSlotEnd.getY()*100);
					
								break;
							}
						}
					} */
				} else if (sizeMeasured >= this.sizeParkingSpace){
					
					/*
					// Checks current array of parkingSlots
					if (!this.parkingSlotsList.isEmpty()){
						for (int i = 0; i < this.parkingSlotsList.size(); i++) {
							
							// Parking Slot already exists (same coordinates)
							if (this.parkingSlotsList.get(i).getFrontBoundaryPosition().distance(this.parkingSlotEnd)*100 < 10
								&& this.parkingSlotsList.get(i).getBackBoundaryPosition().distance(this.parkingSlotBegin)*100 < 10){	
								
								// Plays sound
								Sound.playTone(260,100); // DO4, 0.5s
								
								// Update parking slot
								monitor.writeNavigationComment("Parking Slot Exists");
								monitor.writeNavigationComment("Begin X: " + this.parkingSlotsList.get(i).getBackBoundaryPosition().getX()*100 + " Y: " + this.parkingSlotsList.get(i).getBackBoundaryPosition().getY()*100);
								monitor.writeNavigationComment("Front X: " + this.parkingSlotsList.get(i).getFrontBoundaryPosition().getX()*100 + " Y: " + this.parkingSlotsList.get(i).getFrontBoundaryPosition().getY()*100);
								newSlot = false;
								break;
							}
						}
					} */
					
					// Adding New Parking Slot
					if (newSlot){
						this.parkingSlotID++;
						this.parkingSlotsList.add(
								new ParkingSlot(
										this.parkingSlotID, 
										this.parkingSlotBegin.clone(), 
										this.parkingSlotEnd.clone(), 
										ParkingSlotStatus.SUITABLE_FOR_PARKING, 
										Math.round((this.measurementQualityBegin + this.measurementQualityEnd)/2))
								);
						
						// Plays sound
						// Sound.playTone(520,100); // DO4, 0.5s
						
						monitor.writeNavigationComment("Added New Slot");
						monitor.writeNavigationComment("Begin X: " + this.parkingSlotBegin.getX()*100 + " and Y: " + this.parkingSlotBegin.getY()*100);
						monitor.writeNavigationComment("End X: " + this.parkingSlotEnd.getX()*100 + " and Y: " + this.parkingSlotEnd.getY()*100);
						monitor.writeNavigationComment("Size Measured: " + sizeMeasured);
						monitor.writeNavigationComment("Quality: " + Math.round((this.measurementQualityBegin + this.measurementQualityEnd)/2));
					}
				}
			}
		}
		
		return;
	}
}
