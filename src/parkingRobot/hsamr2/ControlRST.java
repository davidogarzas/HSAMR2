package parkingRobot.hsamr2;


import lejos.robotics.navigation.Pose;
import lejos.util.Delay;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.Motor;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	IMonitor monitor = null;
	ControlThread ctrlThread = null;

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
  
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param monitor corresponding main module Monitor class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor, IMonitor monitor){
		this.perception = perception;
        this.navigation = navigation;
        this.monitor = monitor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		// MONITOR (example)
		monitor.addControlVar("RightSensor");
		monitor.addControlVar("LeftSensor");
		monitor.addControlVar("RightWheelPower");
		monitor.addControlVar("LeftWheelPower");
		monitor.addControlVar("Error");
		monitor.addControlVar("Correction");
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1);
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
		
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      exec_LINECTRL_ALGO();
		                      break;
		  case VW_CTRL		: update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();		
	}
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity);
	}
    
	//Aufgabe 3.3
    private void exec_SETPOSE_ALGO() {
        // Ziel: Implementierung der Regelung für die Geradeausfahrt
        // Zielposition
        double targetX = 100.0; // Ziel in X-Richtung
        double targetY = 0.0;   // Ziel in Y-Richtung

        // Regelparameter
        double v0 = 30.0;  // Zielgeschwindigkeit
        double kp = 2.0;    // Proportionalverstärker
        double kd = 1.0;    // Differenzialverstärker

        // Fehlerinitialisierung
        double lastError = 0.0;

        // Schleife zur Regelung der Geradeausfahrt
        while (true) {
            // Aktuelle Pose abrufen
            Pose currentPose = getCurrentPose(); // Methode zur Abfrage der Fahrzeugpose

            // Aktuelle Querabweichung berechnen
            double error = computeLateralError(currentPose, targetX, targetY);

            // Differenzialanteil (Änderung des Fehlers)
            double derivative = error - lastError;

            // Regelgröße berechnen
            double correction = kp * error + kd * derivative;

            // Motorengeschwindigkeiten anpassen
            double u1 = v0 + correction; // Geschwindigkeit linkes Rad
            double u2 = v0 - correction; // Geschwindigkeit rechtes Rad

            // Setze die Motoren
            Motor.A.setSpeed((int) Math.max(0, u1)); // Begrenzung auf 0 oder mehr
            Motor.B.setSpeed((int) Math.max(0, u2));
            Motor.A.forward();
            Motor.B.forward();

            // Fehler aktualisieren
            lastError = error;

            // Abbruchbedingung (optional, z. B. Ziel erreicht oder Taste gedrückt)
            if (hasReachedTarget(currentPose, targetX, targetY)) {
                break;
            }

            // Kleine Pause für Stabilität
            Delay.msDelay(50);
        }

        // Stoppe Motoren nach Abschluss
        Motor.A.stop();
        Motor.B.stop();
    }

    /**
     * Berechnet die Querabweichung e zur Zielgeraden.
     * @param pose Aktuelle Pose des Fahrzeugs.
     * @param targetX Ziel-X-Koordinate.
     * @param targetY Ziel-Y-Koordinate.
     * @return Querabweichung e.
     */
    private double computeLateralError(Pose pose, double targetX, double targetY) {
        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getHeading();

        // Transformiere Koordinaten
        double dx = targetX - x;
        double dy = targetY - y;

        // Berechne Querabweichung in lokaler Fahrzeugkoordinate
        return -dy * Math.cos(theta) + dx * Math.sin(theta);
    }

    /**
     * Überprüft, ob das Fahrzeug das Ziel erreicht hat.
     * @param pose Aktuelle Pose des Fahrzeugs.
     * @param targetX Ziel-X-Koordinate.
     * @param targetY Ziel-Y-Koordinate.
     * @return true, wenn Ziel erreicht ist.
     */
    private boolean hasReachedTarget(Pose pose, double targetX, double targetY) {
        double distance = Math.sqrt(Math.pow(pose.getX() - targetX, 2) + Math.pow(pose.getY() - targetY, 2));
        return distance < 5.0; // Schwelle von 5 Einheiten
    }

    /**
     * Beispielmethode zur Abfrage der aktuellen Pose.
     * Implementierung ist abhängig vom spezifischen Framework (LeJOS, etc.).
     */
    private Pose getCurrentPose() {
        // Beispiel: Die Pose wird hier simuliert oder von einem Sensor gelesen.
        // Ersetzen Sie diesen Teil mit der spezifischen Implementierung.
        return new Pose(0, 0, 0);
    }

	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		
		// PID-Parameter  
		
		/*VARIANTE 2 OPTION A
		
	    double Kp = 1.0;  // Proportionalfaktor
	    double Ki = 0.1;  // Integralfaktor
	    double Kd = 0.5;  // Differenzialfaktor
	    double dt = 0.5;   //Abtastzeitraum in Sekunden

	    // PID-Zustandsvariablen
	    double integral = 0.0;     // Akkumulierter Fehler
	    double lastError = 0.0;    // Fehler der vorherigen Iteration

	    // Lichtintensität von den Sensoren holen (angenommen: Wertebereich 0-100)
        int lightLeft = perception.getLeftLineSensorValue();
        int lightRight = perception.getRightLineSensorValue();

        // Fehlerberechnung: Differenz der Lichtintensitäten
        double error = lightLeft - lightRight;

        // Proportionalanteil
        double proportional = error;

        // Integralanteil: Fehler akkumulieren
        integral += error * dt;

        // Differenzialanteil: Änderung des Fehlers
        double derivative = (error - lastError) / dt;

        // PID-Berechnung
        double correction = Kp * proportional + Ki * integral + Kd * derivative;

        // Speichern des Fehlers für die nächste Iteration
        lastError = error;

        // Basisgeschwindigkeit für die Motoren
        int basePower = 30;

        // Motorleistung berechnen
        int leftPower = (int) Math.max(0, Math.min(100, basePower - correction));
        int rightPower = (int) Math.max(0, Math.min(100, basePower + correction));

        // Motorleistung setzen
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        
        VARIANTE 2 OPTION A*/
		
	    double Kp = 1.0;   // Proportionalfaktor
	    double Ki = 10.0;  // Integralfaktor
	    double Kd = 0.5;   // Differenzialfaktor
	    double dt = 0.5;   //Abtastzeitraum in Sekunden

	    // Inkrementelle PID-Variablen
	    double PID1 = Kp * (1 + (dt / Ki) + (Kd / dt)); //Coeficiente asociado al error actual (combinación de términos proporcional, integral y derivativo)
	    double PID2 = Kp * (-1 - (2 * Kd / dt));  //Coeficiente asociado al error anterior (influencia negativa proporcional y derivativa)
	    double PID3 = Kp * (Kd / dt); //Coeficiente asociado al error dos pasos atrás (influencia derivativa pura)

	    double correction = 0;
	    double lastCorrection = 0;
	    double error = 0;
	    double lastError = 0;
	    double lastError2 = 0;

	    // Lichtintensität von den Sensoren holen (angenommen: Wertebereich 0-100)
	    int lightLeft = perception.getLeftLineSensorValue();
	    int lightRight = perception.getRightLineSensorValue();

	    // Fehlerberechnung: Differenz der Lichtintensitäten
	    error = lightLeft - lightRight;

	    //PID-Berechnung
	    correction = lastCorrection + (PID1 * error) + (PID2 * lastError) + (PID3 * lastError2);

	    // Speichern des Wertes für die nächste Iteration
	    lastError2 = lastError;
	    lastError = error;
	    lastCorrection = correction;

	    // Basisgeschwindigkeit für die Motoren
	    int basePower = 30;

	    // Motorleistung berechnen
	    int leftPower = (int) Math.max(0, Math.min(100, basePower - correction));
	    int rightPower = (int) Math.max(0, Math.min(100, basePower + correction));

	    // Motorleistung setzen
	    leftMotor.setPower(leftPower);
	    rightMotor.setPower(rightPower);


        // MONITOR: Werte ausgeben
        monitor.writeControlVar("LeftLight", "" + lightLeft);
        monitor.writeControlVar("RightLight", "" + lightRight);
        monitor.writeControlVar("RightWheelPower", "" + rightPower);
        monitor.writeControlVar("LeftWheelPower", "" + leftPower);
        monitor.writeControlComment("PID Correction: " + correction);
        monitor.writeControlVar("Error", "" + error);
        monitor.writeControlComment("Left Line Sensor Value" + perception.getLeftLineSensorValue());
        monitor.writeControlComment("Right Line Sensor Value" + perception.getRightLineSensorValue());

		/*VARIANTE 1
		 // Farbwerte: 0 = weiß, 1 = grau, 2 = schwarz
        int sensorLeftValue = this.lineSensorLeft;
        int sensorRightValue = this.lineSensorRight;

        // Leistungsparameter
        int basePower = 30;  // Basisgeschwindigkeit
        int maxCorrection = 50;  // Maximale Abweichungskorrektur
        float p = 25;

        // Berechnung des Fehlers
        int error = sensorLeftValue - sensorRightValue;

        // Proportionalsteuerung
        float correction = error * p;

        // Begrenzung der Korrekturwerte
        if (correction < -maxCorrection) {correction = -maxCorrection;}
        else if (correction > maxCorrection) {correction = maxCorrection;}

        // Anpassung der Motorleistung
        int leftPower = Math.round(basePower - correction);
        int rightPower = Math.round(basePower + correction);

        // Begrenzung der Motorleistung auf zulässige Werte
        if (leftPower < -100) {leftPower = -100;}
        else if (leftPower > 100) {leftPower = 100;}
        
        if (rightPower < -100) {rightPower = -100;}
        else if (rightPower > 100) {rightPower = 100;}

        // Motorleistung setzen
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // MONITOR
        
        monitor.writeControlVar("LeftSensor", "" + sensorLeftValue);
        monitor.writeControlVar("RightSensor", "" + sensorRightValue);
        monitor.writeControlVar("RightWheelPower", "" + rightPower);
		monitor.writeControlVar("LeftWheelPower", "" + leftPower);
		monitor.writeControlVar("Correction", "" + correction);
		monitor.writeControlVar("Error", "" + error);
		
		VARIANTE 1*/
	    
	     

        /* BEISPIEL CONTROL
		  
		int lowPower = 1;
		int highPower = 45;
  
		// MONITOR (example)
		monitor.writeControlVar("LeftSensor", "" + this.lineSensorLeft);
		monitor.writeControlVar("RightSensor", "" + this.lineSensorRight);	
		
        if(this.lineSensorLeft == 2 && (this.lineSensorRight == 1)){
			
			// when left sensor is on the line, turn left
    	    leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
        else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 1)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
			
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
			
		} 
		else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 0)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		}
		else if(this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
				
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn left");
				
		} 
		else if(this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
			
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
			
			// MONITOR (example)
			monitor.writeControlComment("turn right");
		} 
		BEISPIEL CONTROL */
	}
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	
	//Aufgabe 3.2
	private void drive(double v, double omega) {
	    // Fahrzeugparameter
	    double wheelRadius = 0.138; // Radius der Räder in Metern
	    double distanceLength = 0.12; // Abstand zwischen den Rädern in Metern
	    double pwmScalingFactor = 50; // Experimentell bestimmte Skalierung (z.B. PWM = 50 bei v = 1 m/s)
	    
	    // Berechne Radgeschwindigkeiten
	    double vLeft = v - (omega * distanceLength / 2);
	    double vRight = v + (omega * distanceLength / 2);
	    
	    // Berechne die benötigten PWM-Werte
	    int powerLeft = (int) (vLeft / wheelRadius * pwmScalingFactor);
	    int powerRight = (int) (vRight / wheelRadius * pwmScalingFactor);
	    
	    // Begrenzen der PWM-Werte auf den Bereich [0, 100]
	    powerLeft = Math.max(0, Math.min(100, powerLeft));
	    powerRight = Math.max(0, Math.min(100, powerRight));
	    
	    // Setze Motorleistung
	    leftMotor.setPower(powerLeft);
	    rightMotor.setPower(powerRight);
	    
	    // MONITOR: Gebe die Werte aus
	    monitor.writeControlVar("vLeft", "" + vLeft);
	    monitor.writeControlVar("vRight", "" + vRight);
	    monitor.writeControlVar("PowerLeft", "" + powerLeft);
	    monitor.writeControlVar("PowerRight", "" + powerRight);
	}
}