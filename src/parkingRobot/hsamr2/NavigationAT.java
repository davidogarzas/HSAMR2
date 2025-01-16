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
	
	boolean guidance_testing = true;
	
	// Global Odometry Variables
	double w = 0; // angular velocity [rad/s]
	
	// Custom Variables
	int currentLine = 0;
	boolean horizontalLine = false;
	int lapNumber = 0;
	double currentLineAngle = 0;
	double nextLineAngle = 0;
	double currentLineX = 0;
	double currentLineY = 0;
	long parkingSlotTime = 0;
	int lineAtMeasurement = 0;
	
	
	LinkedList<ParkingSlot> parkingSlotsList = new LinkedList<ParkingSlot>();
	LinkedList<ParkingSlot> parkingSlotsListTest = new LinkedList<ParkingSlot>();

	float[] mapLineAngles = {0,0,0,0,0,0,0,0};
	
	int stateCornerTurn = 0;
	int angleCorrectionState = 0;
	long ref_time_coords = 0;
	long ref_time = 0;
	long time_turning = 0;
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
	float measurementQualityBack = 100;
	float measurementQualityFront = 100;

	Point parkingSlotBackPoint = new Point(0,0);
	Point parkingSlotFrontPoint = new Point(0,0);

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
		//monitor.addNavigationVar("Lap");
		monitor.addNavigationVar("Line");
		monitor.addNavigationVar("X");
		monitor.addNavigationVar("Y");
		//monitor.addNavigationVar("Phi");
		//monitor.addNavigationVar("PhiError");
		monitor.addNavigationVar("wallDistanceFrontSide");
		monitor.addNavigationVar("wallDistanceBackSide");
		
		// INIT MAP
		setMap(GuidanceAT.map);
		calculateMapLineAngles();
		this.currentLine = 0;
		this.updateMapVariables();
		
		// IF GUIDANCE TESTING
		if (guidance_testing){
			this.parkingSlotsListTest.add(
					new ParkingSlot(
							0, 
							new Point((float) 0.25,0), 
							new Point((float) 0.75,0), 
							ParkingSlotStatus.SUITABLE_FOR_PARKING, 
							100)
					);
			this.parkingSlotsListTest.add(
					new ParkingSlot(
							1, 
							new Point((float) 1.30,0), 
							new Point((float) 1.60,0), 
							ParkingSlotStatus.SUITABLE_FOR_PARKING, 
							100)
					);
			this.parkingSlotsListTest.add(
					new ParkingSlot(
							2, 
							new Point((float) 1.8,(float) 0.02), 
							new Point((float) 1.8,(float) 0.50), 
							ParkingSlotStatus.SUITABLE_FOR_PARKING, 
							100)
					);
			this.parkingSlotsListTest.add(
					new ParkingSlot(
							3, 
							new Point((float) 1.15,(float) 0.30), 
							new Point((float) 0.65,(float) 0.30), 
							ParkingSlotStatus.SUITABLE_FOR_PARKING, 
							100)
					);
		}
		
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
		
		if (this.guidance_testing){return this.parkingSlotsListTest.toArray(new ParkingSlot[this.parkingSlotsListTest.size()]);}
		
		else {return this.parkingSlotsList.toArray(new ParkingSlot[this.parkingSlotsList.size()]);}
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
			
			// Turns all angles positive (from 0° to 360°)
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
		this.horizontalLine = this.map[this.currentLine].getY1() == this.map[this.currentLine].getY2();
		if (this.horizontalLine) {this.currentLineY = this.map[this.currentLine].getY1();} //cm
		else if (!this.horizontalLine) {this.currentLineX = this.map[this.currentLine].getX1();} //cm
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
		// when it is 20° from reaching the desired angle of the next line	
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
				
				this.time_turning = System.currentTimeMillis() - this.ref_time;
				
				if (this.onWhite){
					if (this.time_turning >=150
						&& Math.abs((angleResult - this.cornerTurnAngleResult)) >= 60
						&& Math.abs(angleResult - this.nextLineAngle) <= 25){
						
						turn_corner = true;
						this.cornerTurnxDistance = xResult - this.cornerTurnxResult;
						this.cornerTurnyDistance = yResult - this.cornerTurnyResult;
					}
					
					this.stateCornerTurn = 0;
					this.ref_time = System.currentTimeMillis();
					this.time_turning = 0;
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
		this.w 				= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
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
			if (this.horizontalLine){yResult = this.currentLineY/100;} //m
				
			// Corrects Y if line is vertical
			else if (!this.horizontalLine){xResult = this.currentLineX/100;} //m
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
			
			// Updates angle of finished line (on next lap, it will need +360°)
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
			//Sound.playTone(130,100); // C3
			
			/*
			// Prints info
			monitor.writeNavigationComment("Now on Line " + currentLine);
			monitor.writeNavigationComment("Lap: " + this.lapNumber);
			monitor.writeNavigationComment("Estimated X: " + String.valueOf(xResult * 100) + " Y: " + String.valueOf(yResult * 100));
			monitor.writeNavigationComment("Map X: " + String.valueOf(xMap) + " Y: " + String.valueOf(yMap));
			monitor.writeNavigationComment("Current Line Angle: " + this.currentLineAngle);
			monitor.writeNavigationComment("Next Line Angle: " + this.nextLineAngle);
			*/
			
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
				//monitor.writeNavigationComment("Corrected Angle from: " + String.valueOf(angleResult) + " to: " + String.valueOf(this.currentLineAngle));
				angleResult = this.currentLineAngle;		
		}

		// Packs into array
		pose_results[0] = xResult;
		pose_results[1] = yResult;
		pose_results[2] = angleResult;
		return pose_results;
	}
	
	/**
	 * calculates the robot pose
	 */
	private void calculateLocation(){
		
		double[] pose_results = new double[3];
		// pose_results[0] = xResult [m]
		// pose_results[1] = yResult [m]
		// pose_results[2] = angleResult [°]
		
		// Calculates pose using odometry/ encoders 
		pose_results = this.calculateOdometry();
		
		// Correction of pose after corner turn
		pose_results = this.poseCorrectionCornerTurn(pose_results[0],pose_results[1],pose_results[2]);
		
		// Correction of pose according to lineSensors
		pose_results = this.poseCorrectionLineSensors(pose_results[0],pose_results[1],pose_results[2]);
		
		// Correction of angle using SideDistanceSensors
		pose_results = this.angleCorrectionSideDistanceSensors(pose_results[0],pose_results[1],pose_results[2]);
		
		// Updates global pose (used by other methods/ modules)
		this.pose.setLocation((float)pose_results[0], (float)pose_results[1]); // [m]
		this.pose.setHeading((float)(pose_results[2]*Math.PI/180)); // [rad]
		
		// MONITOR VARIABLES
		//monitor.writeNavigationVar("Lap", "" + this.lapNumber);
		monitor.writeNavigationVar("Line", "" + this.currentLine);
		monitor.writeNavigationVar("X", "" + (pose_results[0] * 100)); // [cm]
		monitor.writeNavigationVar("Y", "" + (pose_results[1] * 100)); // [cm]
		//monitor.writeNavigationVar("Phi", "" + pose_results[2]); // [°]
		//monitor.writeNavigationVar("PhiError", "" + (pose_results[2] - this.currentLineAngle));	// [°]
		monitor.writeNavigationVar("wallDistanceFrontSide", "" + this.frontSideSensorDistance);
		monitor.writeNavigationVar("wallDistanceBackSide", "" + this.backSideSensorDistance);
		
	}

	private void saveParkingSlotCoordinate(boolean backPoint, double wallDistance, double wallMeasurement){
		
		
		if (backPoint){
			this.parkingSlotBackPoint.setLocation(
					this.pose.getX(), //m
					this.pose.getY()//m
					);
		}
		
		else{
			this.parkingSlotFrontPoint.setLocation(
					this.pose.getX(), //m
					this.pose.getY() //m
					);
		}
	}
	
	private float calculateMeasurementQuality(double angularVelocityAtMeasurement){
		
		double measurementQuality = 0;
		double measurementQualityAngle = 0; // 90° is max error
		double measurementQualityPose = 0;  // 10 cm is max error	
		double measurementQualityAngularVelocity = 0; // 90 °/s is max error
		
		// Quality based on error from expected angle
		measurementQualityAngle = 100 - Math.abs((Math.toDegrees(this.pose.getHeading())-this.currentLineAngle))*100/90;
		
		// Quality based on error from expected X or Y coordinate
		if (this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getY()*100 - this.currentLineY)*100/10;}
		else if (!this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getX()*100 - this.currentLineX)*100/10;}
		
		// Quality based on how long the robot had been making a turn (200 ms is max)
		measurementQualityAngularVelocity = 100 - angularVelocityAtMeasurement*100/90;
		
		// Sum of quality factors
		measurementQuality = this.measurementQualityEncoders*0.4 
							+ measurementQualityAngle*0.2 
							+ measurementQualityPose*0.2 
							+ measurementQualityAngularVelocity*0.2;
		
		return (float) measurementQuality;
	}
	
	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		// Variables
		double sizeMeasured = 0; //cm
		double sizeParkingSpace = 30; //cm
		double allowedError = 10; //cm
		double actualWallDistance = 10; //cm
		double wallDistanceLowerThreshhold = 12; //cm
		double wallDistanceUpperThreshhold = 17; //cm
		double wallMeasurement = 0; //cm
		double angularVelocityAtMeasurement = 0; // [°/s]
		
		int indexBackPoint = 0;
		int indexFrontPoint = 0;
		boolean backPointExists = false;
		boolean frontPointExists = false;
	

		// state 0 = Looking for Beginning of Possible Slot
		// state 1 = Measuring Possible Slot
		
		// Looking for Beginning of Possible Slot
		if (this.parking_slot_state == 0){
			
			// Sensor detects enough depth for parking space
			if (this.frontSideSensorDistance >= wallDistanceUpperThreshhold
				&& this.backSideSensorDistance <= wallDistanceLowerThreshhold){
				
				// Goes to next state
				this.parking_slot_state = 1;
				
				//Saves line it was found on
				this.lineAtMeasurement = this.currentLine;
				
				// Saves measurement of wall
				wallMeasurement = this.frontSideSensorDistance;
				
				// Saves time robot had been turning
				angularVelocityAtMeasurement = this.w*180/Math.PI; //[°/s]
				
				// Plays sound
				//Sound.playTone(520,25); // C5
				
				// Saves Back Coordinate of Parking Slot
				this.saveParkingSlotCoordinate(true,actualWallDistance,wallMeasurement);
				
				// Saves Measurement Quality of Back Point
				this.measurementQualityBack = calculateMeasurementQuality(angularVelocityAtMeasurement);
			}		
		} 
		
		else if (this.parking_slot_state == 1){
			
			// Avoids corners
			if (this.lineAtMeasurement != this.currentLine
				|| (this.frontSideSensorDistance < wallDistanceUpperThreshhold 
					&& this.backSideSensorDistance < wallDistanceUpperThreshhold)){
				
				this.parking_slot_state = 0;
				
				// Plays sound
				//Sound.playTone(260,25); // C4
			}

			
			// Stops detecting wall
			if (this.frontSideSensorDistance <= wallDistanceLowerThreshhold
				&& this.backSideSensorDistance >= wallDistanceUpperThreshhold){
				
				// Resets state
				this.parking_slot_state = 0;
		
				// Plays sound
				//Sound.playTone(1040,25); // C6

				// Saves measurement of frontSideSensor
				wallMeasurement = this.frontSideSensorDistance;
				
				// Saves angular velocity at measurement
				angularVelocityAtMeasurement = this.w*180/Math.PI; //[°/s]
				
				// Saves Front Coordinates of Parking Slot
				this.saveParkingSlotCoordinate(false, actualWallDistance, wallMeasurement);

				// Calculates size of space
				sizeMeasured = this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100; //cm
				
				// Calculates Quality
				this.measurementQualityFront = calculateMeasurementQuality(angularVelocityAtMeasurement);
				
				monitor.writeNavigationComment("Slot Found: " + this.parkingSlotID);
				monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
				monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
				monitor.writeNavigationComment("Size: " + this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100);

				// Look through array of saved slots
				if (!this.parkingSlotsList.isEmpty()){
					
					// Check existing array
					for (int i = 0; i < this.parkingSlotsList.size(); i++) {
						
						// 1st Point Already Exists
						if (this.parkingSlotsList.get(i).getBackBoundaryPosition().distance(this.parkingSlotBackPoint)*100 < allowedError){	
							backPointExists = true;
							indexBackPoint = this.parkingSlotsList.get(i).getID();
						}
						
						// 2nd Point Already Exists
						if (this.parkingSlotsList.get(i).getFrontBoundaryPosition().distance(this.parkingSlotFrontPoint)*100 < allowedError){	
							frontPointExists = true;
							indexFrontPoint = this.parkingSlotsList.get(i).getID();
						}
					}
				}
						
				// Slot does not exist
				if (!backPointExists && !frontPointExists){
					
					// Must add slot (quality max 70)
					if (sizeMeasured >= sizeParkingSpace){
						Sound.playTone(520,100); // C5
						this.parkingSlotsList.add(
								new ParkingSlot(
										this.parkingSlotID, 
										this.parkingSlotBackPoint.clone(), 
										this.parkingSlotFrontPoint.clone(), 
										ParkingSlotStatus.SUITABLE_FOR_PARKING, 
										(int) (0.7*(this.measurementQualityBack + this.measurementQualityFront)/2))
								);
						this.parkingSlotID++;
					}
					
					// Can ignore points measured
					//else {}
				}
				
				// Only Front point exists (an added slot has been changed, must update)
				// Quality max 50
				else if (!backPointExists && frontPointExists){
					
					// Must make slot suitable for parking and update backPoint
					if (sizeMeasured >= sizeParkingSpace){
						Sound.playTone(1040,100); // C6
						this.parkingSlotsList.get(indexFrontPoint).setBackBoundaryPosition(this.parkingSlotBackPoint.clone());
						this.parkingSlotsList.get(indexFrontPoint).setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);
						this.parkingSlotsList.get(indexFrontPoint).setMeasurementQuality((int) (0.5*(this.measurementQualityBack + this.measurementQualityFront)/2));
					}
					
					// Deletes slot
					else {
						Sound.playTone(130,100); // C3
						this.parkingSlotsList.remove(indexFrontPoint);}
				}
				
				// Only Back point exists (an added slot has been changed, must update)
				// Quality max 50
				else if (backPointExists && !frontPointExists){
					
					// Must make slot suitable for parking and update frontPoint
					if (sizeMeasured >= sizeParkingSpace){
						Sound.playTone(1040,100); // C6
						this.parkingSlotsList.get(indexBackPoint).setFrontBoundaryPosition(this.parkingSlotFrontPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);
						this.parkingSlotsList.get(indexBackPoint).setMeasurementQuality((int) (0.5*(this.measurementQualityBack + this.measurementQualityFront)/2));
					}
					
					// Deletes slot
					else {
						Sound.playTone(130,100); // C3
						this.parkingSlotsList.remove(indexBackPoint);}	
				}
				
				// Slot already exists (can leave slot the same or update)
				// Quality Max 100
				else if (backPointExists && frontPointExists && indexBackPoint == indexFrontPoint){
					
					// Compare quality of measurement and make suitable
					if (sizeMeasured >= sizeParkingSpace){
						Sound.twoBeeps();
						//Sound.playTone(2080,100); // C7
						this.parkingSlotsList.get(indexBackPoint).setBackBoundaryPosition(this.parkingSlotBackPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setFrontBoundaryPosition(this.parkingSlotFrontPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setStatus(ParkingSlotStatus.SUITABLE_FOR_PARKING);
						this.parkingSlotsList.get(indexBackPoint).setMeasurementQuality((int) (this.measurementQualityBack + this.measurementQualityFront)/2);
					}
					
					// Shouldnt be possible but Deletes slot just in case
					else {
						Sound.playTone(130,100); // C3
						//this.parkingSlotsList.remove(indexBackPoint);
					}
				}
				
				// Points exist but belong to different slots
				// Quality max 70
				else if (backPointExists && frontPointExists && indexBackPoint != indexFrontPoint){
					
					// Must merge slots
					if (sizeMeasured >= sizeParkingSpace){
						
						Sound.twoBeeps();
						// Adds new parking Slot
						this.parkingSlotsList.add(
								new ParkingSlot(
										this.parkingSlotID, 
										this.parkingSlotBackPoint.clone(), 
										this.parkingSlotFrontPoint.clone(), 
										ParkingSlotStatus.SUITABLE_FOR_PARKING, 
										(int) (0.7*(this.measurementQualityBack + this.measurementQualityFront)/2))
								);
						this.parkingSlotID++;
						
						// Deletes old parking slots
						this.parkingSlotsList.remove(indexBackPoint);
						this.parkingSlotsList.remove(indexFrontPoint);
					}
					
					// Shouldnt be possible
					//else {}	
				}
			}
		}
		
		return;
	}
}
