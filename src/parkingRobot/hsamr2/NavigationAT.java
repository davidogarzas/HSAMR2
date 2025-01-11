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
	boolean horizontalLine = false;
	int lapNumber = 0;
	double currentLineAngle = 0;
	double nextLineAngle = 0;
	double currentLineX = 0;
	double currentLineY = 0;
	
	
	LinkedList<ParkingSlot> parkingSlotsList = new LinkedList<ParkingSlot>();
	
	float[] mapLineAngles = {0,0,0,0,0,0,0,0};
	
	int stateCornerTurn = 0;
	int angleCorrectionState = 0;
	long ref_time_coords = 0;
	long ref_time = 0;
	double cornerTurnAngleResult = 0;
	double cornerTurnxResult = 0;
	double cornerTurnyResult = 0;
	double cornerTurnxDistance = 0;
	double cornerTurnyDistance = 0;
	// 0 = both white
	// 1 = different
	
	// For parkingSlots
	float measurementQualityEncoders = 100;
	double measurementQualityDistanceFactor = 0.001;
	float measurementQualityBegin = 100;
	float measurementQualityEnd = 100;
	float beginX = 0;
	float beginY = 0;
	Point parkingSlotBegin = new Point(0,0);
	Point parkingSlotEnd = new Point(0,0);
	float finalX = 0;
	float finalY = 0;
	int parking_slot_state = 0;
	int parkingSlotID = 0;
	
	/**
	 * line information measured by light sensor: 0 - beside line, 1 - on line border or 
	 * gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	int lineSensorLeft	=	0;
	boolean onWhite 	= false;
		
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
		calculateMapLineAngles();
		this.currentLine = 0;
		this.updateMapVariables();
				
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
	
	private void calculateMapLineAngles(){
		for (int i=0; i<this.map.length; i++){
			this.mapLineAngles[i] = this.map[i].getP1().angleTo(this.map[i].getP2());
			
			// Turns all angles positive (from 0� to 360�)
			if (this.mapLineAngles[i] < 0){this.mapLineAngles[i] += 360;}
		}
	}
	
	private void updateMapVariables(){
		
		// Updates Current Line Angle (the angle of the current line)
		this.currentLineAngle = this.mapLineAngles[this.currentLine];
		
		// Updates Next Line Angle (the angle of the next line)
		if (this.currentLine < this.map.length - 1){this.nextLineAngle = this.mapLineAngles[this.currentLine + 1];} 
		else {this.nextLineAngle = this.mapLineAngles[0];}
		
		// Updates type of line
		this.horizontalLine = this.map[this.currentLine].getX1() == this.map[this.currentLine].getX2();
		if (this.horizontalLine) {this.currentLineX = this.map[this.currentLine].getX1();} //cm
		else if (!this.horizontalLine) {this.currentLineY = this.map[this.currentLine].getY1();} //cm
	}
	
	private boolean detectCorner(double xResult, double yResult, double angleResult, int method){
		
		boolean turn_corner = false;
		
		// Method 1
		// Detect corner when the change in angle is big enough while a line sensor detects black
		if (method == 1){
			if (this.stateCornerTurn == 0){
				if (!this.onWhite){
					this.stateCornerTurn = 1;
					this.cornerTurnAngleResult = angleResult;
				}
			} 
			
			else if (this.stateCornerTurn == 1){
				if (this.onWhite){
					this.stateCornerTurn = 0;
					if (Math.abs((angleResult - this.cornerTurnAngleResult)) >= 70){
						turn_corner = true; 
					}
					this.cornerTurnAngleResult = angleResult;
				}
			}
		}
		
		// Method 2
		// Detects corner when a lineSensor has detected black for a certain time period
		else if (method == 2){
			if (this.stateCornerTurn == 0){
				if (!this.onWhite){
					this.stateCornerTurn = 1;
					this.ref_time = System.currentTimeMillis();
				}
			} 
			
			else if (this.stateCornerTurn == 1){
				if (this.onWhite){
					this.stateCornerTurn = 0;
					if (Math.abs((System.currentTimeMillis() - this.ref_time)) >= 200){
						turn_corner = true; 
					}
					this.ref_time = System.currentTimeMillis();
				}
			}
		}
		
		// Method 3
		// Detects change of line/ corner turn 
		// when it is 20� from reaching the desired angle of the next line	
		else if (method == 3){
			if (Math.abs(angleResult - this.nextLineAngle) <= 20){
				turn_corner = true;
			}
		}
		
		// Method 4
		// Combines method 1, 2 and 3
		// Allows for each of the 'components' to be less strict on their own
		else if (method == 4){
			if (this.stateCornerTurn == 0){
				if (!this.onWhite){
					this.stateCornerTurn = 1;
					this.ref_time = System.currentTimeMillis();
					this.cornerTurnAngleResult = angleResult;
					this.cornerTurnxResult = xResult;
					this.cornerTurnyResult = yResult;
				}
			} 
			
			else if (this.stateCornerTurn == 1){
				if (this.onWhite){
					this.stateCornerTurn = 0;
					if (System.currentTimeMillis() - this.ref_time >=150
						&& Math.abs((angleResult - this.cornerTurnAngleResult)) >= 60
						&& Math.abs(angleResult - this.nextLineAngle) <= 25){
						
						turn_corner = true;
						this.cornerTurnxDistance = xResult - this.cornerTurnxResult;
						this.cornerTurnyDistance = yResult - this.cornerTurnyResult;
					}
					
					this.ref_time = System.currentTimeMillis();
					this.cornerTurnAngleResult = angleResult;
					this.cornerTurnxResult = xResult;
					this.cornerTurnyResult = yResult;
				}
			}
		}

		return turn_corner;
	}
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes 
	 * the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		this.onWhite				= this.lineSensorRight == 0 && this.lineSensorLeft == 0;
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		// Converts to cm
		this.frontSensorDistance	= perception.getFrontSensorDistance()/10;
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance()/10;
		this.backSensorDistance		= perception.getBackSensorDistance()/10;
		this.backSideSensorDistance	= perception.getBackSideSensorDistance()/10;
	}		 		
	
	// Calculates robot pose from odometry/ encoders
	private double[] calculateOdometry(){
		
		double[] pose_results = new double[3];
		
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
		this.measurementQualityEncoders -= this.measurementQualityDistanceFactor;
		
		// Turns angle to degrees
		angleResult = Math.toDegrees(angleResult);
		
		// Packs into array
		pose_results[0] = xResult;
		pose_results[1] = yResult;
		pose_results[2] = angleResult;
		
		return pose_results;
	}
	
	private double[] poseCorrectionLineSensors(double xResult, double yResult, double angleResult){
		
		double[] pose_results = new double[3];
		
		// Corrects X or Y every time both line sensors detect white
		if (this.onWhite){
			
			// Corrects X if line is horizontal
			if (this.horizontalLine){xResult = this.currentLineX/100;} //m
				
			// Corrects Y if line is vertical
			else if (!this.horizontalLine){yResult = this.currentLineY/100;} //m
		}
		
		// Correction for angle
		if (this.angleCorrectionState == 0){
			
			// Both Line Sensors Detect White
			if (this.onWhite){
				this.angleCorrectionState = 1;
				this.ref_time_coords = System.currentTimeMillis();
			}
		}
		
		else if (this.angleCorrectionState == 1){
			
			// One of the Line Sensors doesnt detect White (Robot is not going straight)
			if (!this.onWhite){
				this.angleCorrectionState = 0;
				this.ref_time_coords = System.currentTimeMillis();
			}
			
			// 2 seconds have passed with robot going straight
			else if (Math.abs(System.currentTimeMillis() - this.ref_time_coords) >= 2000){
				this.angleCorrectionState = 0;
				this.ref_time_coords = System.currentTimeMillis();
				
				// Corrects angle to currentLineAngle
				angleResult = this.currentLineAngle;
				// Sound.playTone(260,100); // C4
			}
		}
		
		// Packs into array
		pose_results[0] = xResult;
		pose_results[1] = yResult;
		pose_results[2] = angleResult;
		
		return pose_results;
	}
	
	private double[] poseCorrectionCornerTurn(double xResult, double yResult, double angleResult){
		
		double[] pose_results = new double[3];
		double xMap	= 0;
		double yMap	= 0;
		
		boolean turn_corner = this.detectCorner(xResult,yResult,angleResult,4);
		
		if (turn_corner){
			
			// Updates angle of finished line (on next lap, it will need +360�)
			this.mapLineAngles[this.currentLine] += 360;
			
			// Updates Current Line Index of the Map
			// Increases counter by one as long as the robot wasn't on the last line
			if (this.currentLine < this.map.length - 1){this.currentLine++;}
			
			// Resets line number and increases lap number if the robot finished the last line
			else {
				this.currentLine = 0;
				this.lapNumber++;
			}
			
			this.updateMapVariables();
			
			// Gets X Y from Map Coordinates
			xMap = this.map[currentLine].getX1();  // cm
			yMap = this.map[currentLine].getY1();  // cm
			
			// Considers the distance covered on X or Y while making the corner turn
			if (this.horizontalLine) {xMap += this.cornerTurnxDistance*100;} //cm
			else if (!this.horizontalLine) {yMap += this.cornerTurnyDistance*100;} //cm
			
			// Considers the turning radius of robot for coordinate correction
			//xMap += 5*Math.cos(this.currentLineAngle * Math.PI/180);
			//yMap += 5*Math.sin(this.currentLineAngle * Math.PI/180);
			
			// Plays sound
			// Sound.playTone(130,100); // C3
			
			// Prints info
			monitor.writeNavigationComment("Now on Line " + currentLine);
			monitor.writeNavigationComment("Lap: " + this.lapNumber);
			monitor.writeNavigationComment("Estimated X: " + String.valueOf(xResult * 100) + " Y: " + String.valueOf(yResult * 100));
			monitor.writeNavigationComment("Map X: " + String.valueOf(xMap) + " Y: " + String.valueOf(yMap));
			monitor.writeNavigationComment("Current Line Angle: " + this.currentLineAngle);
			monitor.writeNavigationComment("Next Line Angle: " + this.nextLineAngle);
			
			// Updates X Y to Map Coordinates
			//xResult = xMap / 100; // m
			//yResult = yMap / 100; // m
			
			// Resets MeasurementQuality
			this.measurementQualityEncoders = 100;
		}
		
		// Packs into array
		pose_results[0] = xResult;
		pose_results[1] = yResult;
		pose_results[2] = angleResult;
		
		return pose_results;
	}
	
	private double[] angleCorrectionSideDistanceSensors(double xResult, double yResult, double angleResult){
		
		double[] pose_results = new double[3];
		
		// If both line sensors are on white (going straight)
		if (this.onWhite
			
			// if sensors are detecting a wall
			&& this.frontSideSensorDistance < 20 && this.backSideSensorDistance < 20
			&& this.frontSideSensorDistance > 0 && this.backSideSensorDistance > 0
			
			// If both sensors are parallel to wall //mm
			&& Math.abs(this.frontSideSensorDistance - this.backSideSensorDistance) <= 1
			
			// If difference between angleResult and currentLineAngle (desired angle) is too big
			&& Math.abs(this.currentLineAngle - angleResult) >= 5){
					
				// Plays sound
				//Sound.playTone(523,100); //C5
				
				// Updates angleResult
				monitor.writeNavigationComment("Corrected Angle from: " + String.valueOf(angleResult) + " to: " + String.valueOf(this.currentLineAngle));
				angleResult = this.currentLineAngle;		
		}

		// Packs into array
		pose_results[0] = xResult;
		pose_results[1] = yResult;
		pose_results[2] = angleResult;
		return pose_results;
	}
	
	private float calculateMeasurementQuality(){
		
		double measurementQuality = 0;
		double measurementQualityAngle = 0; // 90� is max error
		double measurementQualityPose = 0;  // 10 cm is max error	
		
		measurementQualityAngle = 100 - Math.abs((Math.toDegrees(this.pose.getHeading())-this.currentLineAngle))*100/90;
		
		if (this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getX()*100 - this.currentLineX)*100/10;}
		else if (!this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getY()*100 - this.currentLineY)*100/10;}
		
		measurementQuality = this.measurementQualityEncoders*0.5 + measurementQualityAngle*0.2 + measurementQualityPose*0.3;
		
		return (float) measurementQuality;
	}
	
	/**
	 * calculates the robot pose
	 */
	private void calculateLocation(){
		
		double[] pose_results = new double[3];
		// pose_results[0] = xResult [m]
		// pose_results[1] = yResult [m]
		// pose_results[2] = angleResult [�]
		
		// Calculates pose using odometry/ encoders 
		pose_results = this.calculateOdometry();
		
		// Correction of pose after corner turn
		pose_results = this.poseCorrectionCornerTurn(pose_results[0],pose_results[1],pose_results[2]);
		
		// Correction of pose according to lineSensors
		pose_results = this.poseCorrectionLineSensors(pose_results[0],pose_results[1],pose_results[2]);
		
		// Correction of angle using SideDistanceSensors
		pose_results = this.angleCorrectionSideDistanceSensors(pose_results[0],pose_results[1],pose_results[2]);
		
		// MONITOR VARIABLES
		monitor.writeNavigationVar("Lap", "" + this.lapNumber);
		monitor.writeNavigationVar("Line", "" + this.currentLine);
		monitor.writeNavigationVar("X", "" + (pose_results[0] * 100)); // [cm]
		monitor.writeNavigationVar("Y", "" + (pose_results[1] * 100)); // [cm]
		monitor.writeNavigationVar("Phi", "" + pose_results[2]); // [�]
		monitor.writeNavigationVar("PhiError", "" + (pose_results[2] - this.currentLineAngle));	// [�]
		
		// Updates global pose (used by other methods/ modules)
		this.pose.setLocation((float)pose_results[0], (float)pose_results[1]); // [m]
		this.pose.setHeading((float)(pose_results[2]*Math.PI/180)); // [rad]
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		// Variables
		boolean newSlot = true;
		double sizeMeasured = 0;
		double sizeParkingSpace = 30;
		double allowedError = 10;
		double limitDistanceSensors = 11;
	

		// state 0 = Looking for Beginning of Possible Slot
		// state 1 = Measuring Possible Slot
		
		// Looking for Beginning of Possible Slot
		if (this.parking_slot_state == 0){

				// Sensor detects enough depth for parking space
				if (this.frontSideSensorDistance >= limitDistanceSensors){
					
					// Goes to next state
					this.parking_slot_state = 1;
					
					// Plays sound
					//Sound.playTone(260,100); // C4
					
					// Saves Back Coordinate of Parking Slot
					this.parkingSlotBegin.setLocation(this.pose.getX(),this.pose.getY()); // m
					
					// Saves Measurement Quality of Begin Point
					this.measurementQualityBegin = calculateMeasurementQuality();
				}
			
				
		// Measuring Possible Slot
		} else if (this.parking_slot_state == 1){
			
				// Sensor stops detecting enough space for parking space
				if (this.frontSideSensorDistance < limitDistanceSensors){
					
					// Saves Front Coordinates of Parking Slot
					this.parkingSlotEnd.setLocation(this.pose.getX(), this.pose.getY());
					sizeMeasured = this.parkingSlotEnd.distance(this.parkingSlotBegin)*100; // cm
					this.measurementQualityEnd = calculateMeasurementQuality();
					this.parking_slot_state = 0;
					
					// Space is too small
					if (sizeMeasured < sizeParkingSpace){
						
						boolean slotExists = false;
						
						monitor.writeNavigationComment("Slot not possible");
						monitor.writeNavigationComment("Begin X: " + this.parkingSlotBegin.getX()*100 + " Y: " + this.parkingSlotBegin.getY()*100);
						monitor.writeNavigationComment("End X: " + this.parkingSlotEnd.getX()*100 + " Y: " + this.parkingSlotEnd.getY()*100);
						monitor.writeNavigationComment("Size Measured: " + sizeMeasured);					
						
						// Look through array, see if first coordinate was saved and update slot
						if (!this.parkingSlotsList.isEmpty()){
							
							// Check existing array
							for (int i = 0; i < this.parkingSlotsList.size(); i++) {
								
								// ADD CASES (PENDING) (ex first point exists, second doesnt
								// first point doesnt exist, second does)
								
								// Parking Slot already exists
								if (this.parkingSlotsList.get(i).getBackBoundaryPosition().distance(this.parkingSlotBegin)*100 < allowedError){	
									
									slotExists = true;
									
									// Plays sound
									Sound.playTone(130,100); // C3
									
									// Update parking slot
									
									this.parkingSlotsList.get(i).setFrontBoundaryPosition(this.parkingSlotEnd.clone());
									this.parkingSlotsList.get(i).setStatus(ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING);
		
									monitor.writeNavigationComment("Changed Slot " + this.parkingSlotsList.get(i).getID());
									monitor.writeNavigationComment("Begin X: " + this.parkingSlotBegin.getX()*100 + " and Y: " + this.parkingSlotBegin.getY()*100);
									monitor.writeNavigationComment("End X: " + this.parkingSlotEnd.getX()*100 + " and Y: " + this.parkingSlotEnd.getY()*100);
						
									break;
								}
							}
						}
						if (!slotExists){
							// Plays sound
							Sound.playTone(260,100); // C4
						}
					} 
					
					else if (sizeMeasured >= sizeParkingSpace){
						
						
						// Checks current array of parkingSlots
						if (!this.parkingSlotsList.isEmpty()){
							for (int i = 0; i < this.parkingSlotsList.size(); i++) {
								
								// Parking Slot already exists (same coordinates)
								if (this.parkingSlotsList.get(i).getFrontBoundaryPosition().distance(this.parkingSlotEnd)*100 < allowedError
									&& this.parkingSlotsList.get(i).getBackBoundaryPosition().distance(this.parkingSlotBegin)*100 < allowedError){
									
									// Update parking slot (PENDING)
									// Compare measurement qualities for what coordinates to keep
									if (this.parkingSlotsList.get(i).getStatus() == ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING){
										this.parkingSlotsList.get(i).setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);
									}
									
									// Plays sound
									Sound.playTone(2093,100); // C7
									
									/*
									// Update parking slot
									monitor.writeNavigationComment("Parking Slot Exists");
									monitor.writeNavigationComment("Begin X: " + this.parkingSlotsList.get(i).getBackBoundaryPosition().getX()*100 + " Y: " + this.parkingSlotsList.get(i).getBackBoundaryPosition().getY()*100);
									monitor.writeNavigationComment("Front X: " + this.parkingSlotsList.get(i).getFrontBoundaryPosition().getX()*100 + " Y: " + this.parkingSlotsList.get(i).getFrontBoundaryPosition().getY()*100);
									*/
									
									newSlot = false;
									break;
								}
							}
						}
						
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
							Sound.playTone(1046,100); // C6
							
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
