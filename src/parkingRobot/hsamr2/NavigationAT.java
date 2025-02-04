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
	
	// For use by other modules
	boolean guidance_testing = true;
	boolean useOnlyOdometry = false;
	
	// Map Variables
	int lapNumber = 0;
	int currentLine = 0;
	
	float[] mapLineAngles = {0,0,0,0,0,0,0,0};
	double currentLineAngle = 0;
	double nextLineAngle = 0;
	
	double constantX = 0;
	double constantY = 0;
	double lineFinalX = 0;
	double lineFinalY = 0;

	boolean horizontalLine = false;

	// Corner Detection Variables
	int stateCornerTurn = 0;
	long turnTimeRef = 0;
	long turnTime = 0;
	double cornerTurnAngleResult = 0;
	boolean onWhite 	= false;
	boolean goingStraight = false;

	// Angle Correction Variables
	int angleCorrectionState = 0;
	long angleCorrectionRefTime = 0;
	
	// Parking Slots Variables
	LinkedList<ParkingSlot> parkingSlotsList = new LinkedList<ParkingSlot>();
	LinkedList<ParkingSlot> parkingSlotsListTest = new LinkedList<ParkingSlot>();
	int freeSpaceCounter = 0;
	int wallCounter = 0;
	int lineAtMeasurement = 0;

	float measurementQualityEncoders = 100;
	double measurementQualityDistanceFactor = 0.25;
	float measurementQualityBack = 100;
	float measurementQualityFront = 100;
	
	Point temporaryPoint = new Point(0,0);
	Point parkingSlotBackPoint = new Point(0,0);
	Point parkingSlotFrontPoint = new Point(0,0);

	int parking_slot_state = 0;
	int parkingSlotID = 0;
	
	double wallDistanceAverageSum = 0;
	int wallDistanceAverageCounter = 0;
	double wallDistanceAverage = 0;
	
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
	
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.138; 

	
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
		monitor.addNavigationVar("wallDistanceFrontSide");
		monitor.addNavigationVar("wallDistanceBackSide");
		monitor.addNavigationVar("onWhite");
		
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
	
	public void setUseOnlyOdometry(boolean isOn){
		this.useOnlyOdometry = isOn;
	}
	
	// For control demo
	public void setPose(float x, float y, float phi){
		this.pose.setLocation(x, y); // [m]
		this.pose.setHeading(phi); // [rad]
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
	
	private boolean detectCorner(double xResult, double yResult, double angleResult){
		
		boolean turn_corner = false;
					
		if (this.stateCornerTurn == 0){
			if (!this.onWhite){
				this.stateCornerTurn = 1;
				this.turnTimeRef = System.currentTimeMillis();
				this.cornerTurnAngleResult = angleResult;
			}
		} 
		
		else if (this.stateCornerTurn == 1){
			
			this.turnTime = System.currentTimeMillis() - this.turnTimeRef;
			
			if (this.onWhite){
				if (this.turnTime >= 50
					&& Math.abs((angleResult - this.cornerTurnAngleResult)) >= 40
					&& Math.abs(angleResult - this.nextLineAngle) <= 35
					&& Math.abs(xResult*100 - this.lineFinalX) <= 25
					&& Math.abs(yResult*100 - this.lineFinalY) <= 25){
					
					turn_corner = true;
					this.wallDistanceAverageSum = 0;
					this.wallDistanceAverageCounter = 0;
					this.wallDistanceAverage = 0;
				}
				
				this.stateCornerTurn = 0;
				this.turnTimeRef = System.currentTimeMillis();
				this.turnTime = 0;
				this.cornerTurnAngleResult = angleResult;
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
		
		if (this.onWhite && this.frontSideSensorDistance > 0 && this.frontSideSensorDistance < 12
				&& this.goingStraight){
			this.wallDistanceAverageSum += this.frontSideSensorDistance;
			this.wallDistanceAverageCounter++;
			this.wallDistanceAverage = this.wallDistanceAverageSum/this.wallDistanceAverageCounter;
		}
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
			if (this.horizontalLine){yResult = this.constantY/100;} //m
				
			// Corrects Y if line is vertical
			else if (!this.horizontalLine){xResult = this.constantX/100;} //m
		}
		
		// Correction for angle
		if (this.angleCorrectionState == 0){
			
			// Both Line Sensors Detect White
			if (this.onWhite){
				this.angleCorrectionState = 1;
				this.angleCorrectionRefTime = System.currentTimeMillis();
			}
		}
		
		else if (this.angleCorrectionState == 1){
			
			// One of the Line Sensors doesnt detect White (Robot is not going straight)
			if (!this.onWhite){
				this.angleCorrectionState = 0;
				this.angleCorrectionRefTime = System.currentTimeMillis();
			}
			
			// 2 seconds have passed with robot going straight
			else if (Math.abs(System.currentTimeMillis() - this.angleCorrectionRefTime) >= 2000){
				this.angleCorrectionState = 0;
				this.angleCorrectionRefTime = System.currentTimeMillis();
				
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
	
	private void onNewLine(){
		
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
	}
	
	private void updateMapVariables(){
		
		// Updates Current Line Angle (the angle of the current line)
		this.currentLineAngle = this.mapLineAngles[this.currentLine];
		
		// Updates Next Line Angle (the angle of the next line)
		if (this.currentLine < this.map.length - 1){this.nextLineAngle = this.mapLineAngles[this.currentLine + 1];} 
		else {this.nextLineAngle = this.mapLineAngles[0];}
		
		// Updates type of line
		this.horizontalLine = this.map[this.currentLine].getY1() == this.map[this.currentLine].getY2();
		if (this.horizontalLine) {this.constantY = this.map[this.currentLine].getY1();} //cm
		else if (!this.horizontalLine) {this.constantX = this.map[this.currentLine].getX1();} //cm
		
		// Updates lineFinalX and lineFinalY of line
		this.lineFinalX = this.map[this.currentLine].getX2(); //cm
		this.lineFinalY = this.map[this.currentLine].getY2(); //cm
		
		// Resets MeasurementQuality
		this.measurementQualityEncoders = 100;
	}
	
	private void mapLocalization(double xResult, double yResult, double angleResult){
				
		// Detects corner turn
		boolean turn_corner = this.detectCorner(xResult, yResult, angleResult);
		
		if (turn_corner){
			
			// Updates CurrentLine, Lap Number and lineAngle of finished line
			this.onNewLine();
			
			// Updates the rest of the map variables
			this.updateMapVariables();
		}
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
			&& Math.abs(this.currentLineAngle - angleResult) >= 15){
					
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
		
		if (!useOnlyOdometry){
			// Localization of robot in map
			this.mapLocalization(pose_results[0], pose_results[1], pose_results[2]);
			
			// Correction of pose according to lineSensors
			pose_results = this.poseCorrectionLineSensors(pose_results[0],pose_results[1],pose_results[2]);
			
			// Correction of angle using SideDistanceSensors
			pose_results = this.angleCorrectionSideDistanceSensors(pose_results[0],pose_results[1],pose_results[2]);
		}
		
		this.goingStraight = Math.abs(pose_results[2] - this.currentLineAngle) <= 10 ;
		
		// Updates global pose (used by other methods/ modules)
		this.pose.setLocation((float)pose_results[0], (float)pose_results[1]); // [m]
		this.pose.setHeading((float)(pose_results[2]*Math.PI/180)); // [rad]
				
		// MONITOR VARIABLES
		monitor.writeNavigationVar("Lap", "" + this.lapNumber);
		monitor.writeNavigationVar("Line", "" + this.currentLine);
		monitor.writeNavigationVar("X", "" + (pose_results[0] * 100)); // [cm]
		monitor.writeNavigationVar("Y", "" + (pose_results[1] * 100)); // [cm]
		monitor.writeNavigationVar("Phi", "" + pose_results[2]); // [°]
		monitor.writeNavigationVar("PhiError", "" + (pose_results[2] - this.currentLineAngle));	// [°]
		monitor.writeNavigationVar("wallDistanceFrontSide", "" + this.frontSideSensorDistance);
		monitor.writeNavigationVar("wallDistanceBackSide", "" + this.backSideSensorDistance);
		
		if (this.onWhite){
			monitor.writeNavigationVar("onWhite", "" + 1);
		}
		
		else {
			monitor.writeNavigationVar("onWhite", "" + 0);
		}
	}

	private boolean isPointValid(double sensorDistanceFromCenter){
		
		Line projectedLine = new Line(  0,  0, 0,  0); // [cm]
		
		double displacementProjectedLine = 10;   //cm
		double lengthProjectedLine = 45;	//cm
		boolean validPoint = true;

		projectedLine.setLine(
				// X1
				this.pose.getX()*100 
					+ (sensorDistanceFromCenter + lengthProjectedLine/2)*Math.cos(Math.toRadians(this.currentLineAngle))
					+ displacementProjectedLine*Math.sin(Math.toRadians(this.currentLineAngle)), 
				//Y1
				this.pose.getY()*100 
					+ (sensorDistanceFromCenter + lengthProjectedLine/2)*Math.sin(Math.toRadians(this.currentLineAngle))
					- displacementProjectedLine*Math.cos(Math.toRadians(this.currentLineAngle)), 
				//X2
				this.pose.getX()*100 
					+ (sensorDistanceFromCenter - lengthProjectedLine/2)*Math.cos(Math.toRadians(this.currentLineAngle))
					+ displacementProjectedLine*Math.sin(Math.toRadians(this.currentLineAngle)), 
				//Y2
				this.pose.getY()*100 
					+ (sensorDistanceFromCenter - lengthProjectedLine/2)*Math.sin(Math.toRadians(this.currentLineAngle))
					- displacementProjectedLine*Math.cos(Math.toRadians(this.currentLineAngle)) 
				);
		
		// Checks that projected line doesnt intersect any course line
		for (int i=0; i<this.map.length; i++){
			validPoint = !this.map[i].intersectsLine(projectedLine);
			if (!validPoint){break;}
		}
		
		return validPoint;
	}
	
	private void saveParkingSlotCoordinate(int savePoint, double sensorDistanceFromCenter, double wallMeasurement, double angleAtMeasurement){
		
		// Points in [m]
		
		double xDistWithAngle = 0;
		double yDistWithAngle = 0;
		
		if (this.horizontalLine){
			xDistWithAngle = (wallMeasurement/100)*Math.sin(Math.toRadians(angleAtMeasurement)); //m
			yDistWithAngle = 0;
		} 
		
		else {
			xDistWithAngle = 0;
			yDistWithAngle = -(wallMeasurement/100)*Math.cos(Math.toRadians(angleAtMeasurement)); //m
		}
		
		double X = this.pose.getX() 
					+ (sensorDistanceFromCenter/100)*Math.cos(Math.toRadians(this.currentLineAngle)) //m
					+ xDistWithAngle;
		
		double Y = this.pose.getY() 
					+ (sensorDistanceFromCenter/100)*Math.sin(Math.toRadians(this.currentLineAngle)) //m
					+ yDistWithAngle;

		switch (savePoint){

		// Saving Temporary Point
		case 0:
			this.temporaryPoint.setLocation(X,Y);	
			break;
		
		// Saving Back Point
		case 1:
			this.parkingSlotBackPoint.setLocation(X,Y);				  
			break;
		
		// Saving Front Point
		case 2:
			this.parkingSlotFrontPoint.setLocation(X,Y);				  
			break;
		}
	}
	
	private float calculateMeasurementQuality(double angleAtMeasurement){
		
		double measurementQuality = 0;
		double measurementQualityAngle = 0; // 30° is max error
		double measurementQualityPose = 0;  // 10 cm is max error	
		
		// Quality based on error from expected X or Y coordinate
		if (this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getY()*100 - this.constantY)*100/10;}
		else if (!this.horizontalLine){measurementQualityPose = 100 - Math.abs(this.pose.getX()*100 - this.constantX)*100/10;}
		
		// Quality based on angle at time of measurement (30° is max error)
		measurementQualityAngle = 100 - (Math.abs(angleAtMeasurement-this.currentLineAngle))*100/30;
		
		// Sum of quality factors
		measurementQuality = this.measurementQualityEncoders*0.4 
							+ measurementQualityAngle*0.4 
							+ measurementQualityPose*0.2;
		
		return (float) measurementQuality;
	}
	
	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		
		// Variables
		double sizeMeasured = 0; //cm
		double minSizeParkingSpace = 10; //cm
		double sizeParkingSpace = 45; //cm
		double allowedError = 10; //cm
		int quality = 0;
		
		double wallDistanceLowerThreshhold = 0; //cm
		double wallDistanceUpperThreshhold = 12; //cm
		
		double freeSpaceThreshhold = 20;
		
		double angleAtMeasurement = 0; // [°]

		int indexBackPoint = 0;
		int indexFrontPoint = 0;
		
		double BSSensorFromCenter = -8; //cm
		double FSSensorFromCenter = 7;  //cm
		double wallMeasurement = 0;
		
		int saveTemporaryPoint = 0;
		int saveBackPoint = 1;
		int saveFrontPoint = 2;
		
		boolean validPoint = false;
		boolean backPointExists = false;
		boolean frontPointExists = false;
		
		ParkingSlotStatus parkingSlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
		
		boolean FSSensorInWallThreshhold = this.frontSideSensorDistance >= wallDistanceLowerThreshhold && this.frontSideSensorDistance <= wallDistanceUpperThreshhold;
		boolean FSSensorDetectsFreeSpace = this.frontSideSensorDistance >= freeSpaceThreshhold;

		//boolean BSSensorInWallThreshhold = this.backSideSensorDistance >= wallDistanceLowerThreshhold && this.backSideSensorDistance <= wallDistanceUpperThreshhold;
		boolean BSSensorDetectsFreeSpace = this.backSideSensorDistance >= freeSpaceThreshhold;
		
		// Avoids corners
		if (this.lineAtMeasurement != this.currentLine){
			this.parking_slot_state = 0;
			//monitor.writeNavigationComment("To state: " + this.parking_slot_state);

			this.freeSpaceCounter = 0;
			this.wallCounter = 0;

		}
		
		switch(this.parking_slot_state){
		
		// Looking for Beginning of Slot with FS Sensor
		case 0:
			
			// Avoids Finding Spots while turning
			if (this.onWhite){
				
				// FS Sensor detects start of parking slot
				if (FSSensorDetectsFreeSpace){
					
					validPoint = this.isPointValid(FSSensorFromCenter);
					
					if (validPoint){
						
						this.parking_slot_state = 1;
						monitor.writeNavigationComment("To state: " + this.parking_slot_state);

						this.lineAtMeasurement = this.currentLine;

						angleAtMeasurement = Math.toDegrees(this.pose.getHeading()); //[°]
						wallMeasurement = 10; //cm
						
						this.saveParkingSlotCoordinate(saveBackPoint,FSSensorFromCenter,wallMeasurement,angleAtMeasurement);
						this.measurementQualityBack = calculateMeasurementQuality(angleAtMeasurement);
						
						monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
						monitor.writeNavigationComment("Angle at Measurement: " + angleAtMeasurement);
						monitor.writeNavigationComment(" ");
						
					}
					else {
						this.parking_slot_state = 0;
						monitor.writeNavigationComment("To state: " + this.parking_slot_state);

						freeSpaceCounter = 0;
						wallCounter = 0;
					}
				}
			}
			break;
		
		// Ensures consecutive readings of free space
		case 1:
			
			if (this.onWhite){
				if (FSSensorDetectsFreeSpace){
					
					freeSpaceCounter++;
					
					// Saves measurement of BS Sensor in case both sensors measure freeSpace
					if (BSSensorDetectsFreeSpace){
						
						// Searches to State Look for Front Point
						this.parking_slot_state = 3;
						monitor.writeNavigationComment("To state: " + this.parking_slot_state);

						freeSpaceCounter = 0;
						wallCounter = 0;

						// Sees Validity of Point
						validPoint = this.isPointValid(BSSensorFromCenter);
						
						if (validPoint){
							
							angleAtMeasurement = Math.toDegrees(this.pose.getHeading()); //[°]
							wallMeasurement = 10; //cm
							
							this.saveParkingSlotCoordinate(saveTemporaryPoint,BSSensorFromCenter,wallMeasurement,angleAtMeasurement);
							
							// Points are different (1st Point measured late -> BS is more reliable)
							if (this.parkingSlotBackPoint.distance(this.temporaryPoint) >= 0.075){
								
								// Compares BackPoint and TemporaryPoint
								monitor.writeNavigationComment("Back Point Compared");
								monitor.writeNavigationComment("FS BX: " + this.parkingSlotBackPoint.getX()*100 + " FS BY: " + this.parkingSlotBackPoint.getY()*100);
								monitor.writeNavigationComment("BS BX: " + this.temporaryPoint.getX()*100 + " BS BY: " + this.temporaryPoint.getY()*100);
								monitor.writeNavigationComment("Angle at Measurement: " + angleAtMeasurement);
								monitor.writeNavigationComment(" ");
								
								this.saveParkingSlotCoordinate(saveBackPoint,BSSensorFromCenter,wallMeasurement,angleAtMeasurement);
								this.measurementQualityBack = calculateMeasurementQuality(angleAtMeasurement);
							}
						}
						else {
							this.parking_slot_state = 0;
							monitor.writeNavigationComment("To state: " + this.parking_slot_state);

							freeSpaceCounter = 0;
							wallCounter = 0;
						}
					}
				}
				else {wallCounter++;}
			}
							
			if (freeSpaceCounter == 20){
				this.parking_slot_state = 2;
				monitor.writeNavigationComment("To state: " + this.parking_slot_state);

				freeSpaceCounter = 0;
				wallCounter = 0;
			}
			
			if (wallCounter == 3){
				this.parking_slot_state = 0;
				monitor.writeNavigationComment("To state: " + this.parking_slot_state);

				freeSpaceCounter = 0;
				wallCounter = 0;
			}
			break;
		
		
		// Looks for Back Point of Parking Slot With BS Sensor
		case 2:
			
			// Avoids Finding Spots while turning
			if (this.onWhite){
				
				// BS Sensor detects start of parking slot
				if (BSSensorDetectsFreeSpace){
					
					// Searches to State Look for Front Point
					this.parking_slot_state = 3;
					monitor.writeNavigationComment("To state: " + this.parking_slot_state);

					freeSpaceCounter = 0;
					wallCounter = 0;

					// Sees Validity of Point
					validPoint = this.isPointValid(BSSensorFromCenter);
					
					if (validPoint){
						
						angleAtMeasurement = Math.toDegrees(this.pose.getHeading()); //[°]
						wallMeasurement = 10; //cm
						
						this.saveParkingSlotCoordinate(saveTemporaryPoint,BSSensorFromCenter,wallMeasurement,angleAtMeasurement);
						
						// Points are different (1st Point measured late -> BS is more reliable)
						if (this.parkingSlotBackPoint.distance(this.temporaryPoint) >= 0.075){
							
							// Compares BackPoint and TemporaryPoint
							monitor.writeNavigationComment("Back Point Compared");
							monitor.writeNavigationComment("FS BX: " + this.parkingSlotBackPoint.getX()*100 + " FS BY: " + this.parkingSlotBackPoint.getY()*100);
							monitor.writeNavigationComment("BS BX: " + this.temporaryPoint.getX()*100 + " BS BY: " + this.temporaryPoint.getY()*100);
							monitor.writeNavigationComment("Angle at Measurement: " + angleAtMeasurement);
							monitor.writeNavigationComment(" ");
							
							this.saveParkingSlotCoordinate(saveBackPoint,BSSensorFromCenter,wallMeasurement,angleAtMeasurement);
							this.measurementQualityBack = calculateMeasurementQuality(angleAtMeasurement);
						}
					}
				}		
			}
			break;
		
		
		// Looking for Front Point with FS Sensor
		case 3:
			
			// Avoids Finding Spots while turning
			if (this.onWhite){
				
				// FS Sensor detects start of parking slot
				if (FSSensorInWallThreshhold){
					
					validPoint = this.isPointValid(FSSensorFromCenter);
					
					if (validPoint){
						this.parking_slot_state = 4;
						monitor.writeNavigationComment("To state: " + this.parking_slot_state);
						
						angleAtMeasurement = Math.toDegrees(this.pose.getHeading()); //[°]
						wallMeasurement = this.frontSideSensorDistance; //cm
						
						this.saveParkingSlotCoordinate(saveFrontPoint,FSSensorFromCenter,wallMeasurement,angleAtMeasurement);
						this.measurementQualityFront = calculateMeasurementQuality(angleAtMeasurement);

						monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
						monitor.writeNavigationComment("Angle at Measurement: " + angleAtMeasurement);
						monitor.writeNavigationComment(" ");
						
					}
					else {
						this.parking_slot_state = 0;
						monitor.writeNavigationComment("To state: " + this.parking_slot_state);

						freeSpaceCounter = 0;
						wallCounter = 0;
					}
				}
			}
			break;
		

		// Ensures consecutive readings of wall
		case 4:
			
			if (this.onWhite){
				if (FSSensorInWallThreshhold){wallCounter++;}
				else {freeSpaceCounter++;}
			}
				
			
			if (wallCounter == 3){
				this.parking_slot_state = 5;
				monitor.writeNavigationComment("To state: " + this.parking_slot_state);

				freeSpaceCounter = 0;
				wallCounter = 0;
			}
			
			if (freeSpaceCounter == 3){
				this.parking_slot_state = 3;
				monitor.writeNavigationComment("To state: " + this.parking_slot_state);

				freeSpaceCounter = 0;
				wallCounter = 0;
			}
			break;
		
		// Saves Data
		case 5:
			
			this.parking_slot_state = 0;
			monitor.writeNavigationComment("To state: " + this.parking_slot_state);

			freeSpaceCounter = 0;
			wallCounter = 0;
			
			// Calculates size of space
			sizeMeasured = this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100; //cm
			monitor.writeNavigationComment("Exact Size: " + sizeMeasured);

			quality = (int) ((this.measurementQualityBack + this.measurementQualityFront)/2);
			monitor.writeNavigationComment("Quality: " + quality);
			
			// Quality 0 = +/- 15 cm max error
			sizeMeasured += (15 - quality*0.1);
			monitor.writeNavigationComment("Size with quality: " + sizeMeasured);
			
			if (sizeMeasured >= minSizeParkingSpace){
		
				// Plays sound
				//Sound.playTone(1040,25); // C6

				if (sizeMeasured >= sizeParkingSpace){parkingSlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;} 
				else {parkingSlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;}
				
				monitor.writeNavigationComment("Quality: " + quality);

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
						
				// Slot does not exist yet
				if (!backPointExists && !frontPointExists){
					
					// Adds slot (quality max 80)
					Sound.playTone(520,100); // C5
					monitor.writeNavigationComment("Added Slot ID: " + this.parkingSlotID);
					monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
					monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
					monitor.writeNavigationComment("Size: " + this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100);
					monitor.writeNavigationComment("Quality: " + quality);
					monitor.writeNavigationComment(" ");
					
					this.parkingSlotsList.add(
							new ParkingSlot(
									this.parkingSlotID, 
									this.parkingSlotBackPoint.clone(), 
									this.parkingSlotFrontPoint.clone(), 
									parkingSlotStatus, 
									(int) (this.measurementQualityBack + this.measurementQualityFront)/2)
							);
					this.parkingSlotID++;
				}
				
				// Only Front point exists
				// Quality max 50
				else if (!backPointExists && frontPointExists){
					
					Sound.playTone(130,100); // C3

					if (quality > this.parkingSlotsList.get(indexFrontPoint).getMeasurementQuality()){
						monitor.writeNavigationComment("Updated BP of Slot ID: " + this.parkingSlotsList.get(indexFrontPoint).getID());
						monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
						monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
						monitor.writeNavigationComment("Size: " + this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100);
						monitor.writeNavigationComment("Quality: " + quality);
						monitor.writeNavigationComment(" ");
						
						// Updates backPoint
						this.parkingSlotsList.get(indexFrontPoint).setBackBoundaryPosition(this.parkingSlotBackPoint.clone());
						this.parkingSlotsList.get(indexFrontPoint).setStatus(parkingSlotStatus);
						this.parkingSlotsList.get(indexFrontPoint).setMeasurementQuality(quality);
					}
				}
				
				// Only Back point exists
				// Quality max 50
				else if (backPointExists && !frontPointExists){
					
					Sound.playTone(130,100); // C3

					if (quality > this.parkingSlotsList.get(indexBackPoint).getMeasurementQuality()){
					
						monitor.writeNavigationComment("Updated FP of  ID: " + this.parkingSlotsList.get(indexFrontPoint).getID());
						monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
						monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
						monitor.writeNavigationComment("Size: " + this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100);
						monitor.writeNavigationComment("Quality: " + quality);
						monitor.writeNavigationComment(" ");
						
						// Updates frontPoint
						this.parkingSlotsList.get(indexBackPoint).setFrontBoundaryPosition(this.parkingSlotFrontPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setStatus(parkingSlotStatus);
						this.parkingSlotsList.get(indexBackPoint).setMeasurementQuality(quality);
					}
				}
				
				// Slot already exists
				// Quality Max 100
				else if (backPointExists && frontPointExists && indexBackPoint == indexFrontPoint){

					Sound.twoBeeps();

					if (quality > this.parkingSlotsList.get(indexBackPoint).getMeasurementQuality()){

						monitor.writeNavigationComment("Found Same Slot with ID: " + this.parkingSlotsList.get(indexFrontPoint).getID());
						monitor.writeNavigationComment("BX: " + this.parkingSlotBackPoint.getX()*100 + " BY: " + this.parkingSlotBackPoint.getY()*100);
						monitor.writeNavigationComment("FX: " + this.parkingSlotFrontPoint.getX()*100 + " FY: " + this.parkingSlotFrontPoint.getY()*100);
						monitor.writeNavigationComment("Size: " + this.parkingSlotFrontPoint.distance(this.parkingSlotBackPoint)*100);
						monitor.writeNavigationComment("Quality: " + quality);
						monitor.writeNavigationComment(" ");
		
						this.parkingSlotsList.get(indexBackPoint).setBackBoundaryPosition(this.parkingSlotBackPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setFrontBoundaryPosition(this.parkingSlotFrontPoint.clone());
						this.parkingSlotsList.get(indexBackPoint).setStatus(parkingSlotStatus);
						this.parkingSlotsList.get(indexBackPoint).setMeasurementQuality(quality);
					}
				}
				// Points exist but belong to different slots
				// Quality max 70
				else if (backPointExists && frontPointExists && indexBackPoint != indexFrontPoint){
					
					// Merges slots		
					Sound.beep();
					
					if (quality > this.parkingSlotsList.get(indexBackPoint).getMeasurementQuality()
						&& quality > this.parkingSlotsList.get(indexFrontPoint).getMeasurementQuality()){

						// Adds new parking Slot
						this.parkingSlotsList.add(
								new ParkingSlot(
										this.parkingSlotID, 
										this.parkingSlotBackPoint.clone(), 
										this.parkingSlotFrontPoint.clone(), 
										parkingSlotStatus, 
										(int) (this.measurementQualityBack + this.measurementQualityFront)/2)
								);
						this.parkingSlotID++;
						
						// Deletes old parking slots
						this.parkingSlotsList.remove(indexBackPoint);
						this.parkingSlotsList.remove(indexFrontPoint);	
					}
				}
			}
			break;
		}
		
		return;
	}
}
