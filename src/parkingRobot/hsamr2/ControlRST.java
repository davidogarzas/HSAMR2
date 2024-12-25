package parkingRobot.hsamr2;


import lejos.robotics.navigation.Pose;
import lejos.util.Delay;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;                 
import parkingRobot.IPerception.*;
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
		  case DEMO_PRG1 	: update_DEMOPRG1_Parameter();
		                      exec_DEMOPRG1_ALGO();
		                      break;
		}

	}
	
	// Private methods


	private void update_DEMOPRG1_Parameter() {

	}


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
        double targetX = 120.0; // Ziel in X-Richtung
        double targetY = 0.0;   // Ziel in Y-Richtung

        // Regelparameter (PID)
        double v0 = 40.0;   // Zielgeschwindigkeit
        double kp = 1.0;    // Proportionalverstärker
        double ki = 0.1;    // Integralverstärker
        double kd = 0.5;    // Differenzialverstärker

        // Fehlerinitialisierung
        double lastError = 0.0;
        double integral = 0.0;

        // Schleife zur Regelung der Geradeausfahrt
        while (true) {
            // Aktuelle Pose abrufen
            Pose currentPose = getCurrentPose(); // Methode zur Abfrage der Fahrzeugpose
            
            // Überprüfen, ob Ziel erreicht ist (wenn x 120 erreicht hat)
            if (currentPose.getX() >= targetX) {
                break;
            }

            // Aktuelle Querabweichung berechnen
            double error = computeLateralError(currentPose, targetX, targetY);
            
            // Integrieren des Fehlers für den Integralanteil
            integral += error;

            // Differenzialanteil (Änderung des Fehlers)
            double derivative = error - lastError;

            // Regelgröße berechnen (PID-Formel)
            double correction = kp * error + ki * integral + kd * derivative;

            // Motorengeschwindigkeiten anpassen
            double u1 = v0 + correction; // Geschwindigkeit linkes Rad
            double u2 = v0 - correction; // Geschwindigkeit rechtes Rad

            // Setze die Motoren
            leftMotor.setPower((int) Math.max(0, u1)); // Begrenzung auf 0 oder mehr
            rightMotor.setPower((int) Math.max(0, u2));
            leftMotor.forward();
            rightMotor.forward();

            // Fehler aktualisieren
            lastError = error;

            // Kleine Pause für Stabilität
            Delay.msDelay(50);
        }

        // Stoppe Motoren nach Abschluss
        leftMotor.stop();
        rightMotor.stop();
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
    
    int status = 0;
    
    private void exec_DEMOPRG1_ALGO() {
        leftMotor.forward();
        rightMotor.forward();
        
        switch (status) {
            case 0: // Straight drive
            	move();//drive(0.1, 0); //10 cm/s
                if (navigation.getPose().getX() * 100 >= 120) { //Strecke 120 cm
                    stop();
            	    KpLeft = 1; KiLeft = 0.01; KdLeft = 0.1;
            		KpRight = 1; KiRight = 0.08; KdRight = 0.12;
                    status = 1;
                    break;
                }
                break;

            case 1: // Rotate at 15°/s
                drive(0, 15);
                if (navigation.getPose().getHeading() * (180 / Math.PI) >= 70) { //Drehung bis 90°
                    stop();
                	KpLeft = 4; KiLeft = 0.1; KdLeft = 0.1;
                	KpRight = 4; KiRight = 0.15; KdRight = 0.2;
                    status = 2;
                    break;
                }
                break;

            case 2: // Straight drive
                drive(0.05, 0); //5 cm/s
                if (navigation.getPose().getY() * 100 >= 30) { //Strecke 30 cm
                    stop();
            	    KpLeft = 0.5; KiLeft = 0.01; KdLeft = 0.1;
            		KpRight = 0.8; KiRight = 0.08; KdRight = 0.12;
                    status = 3;
                }
                break;

            case 3: // Rotate at -30°/s
                drive(0, -30);
                if (navigation.getPose().getHeading() * 180 / Math.PI == -90) { //Drehung bis 90°
                    stop();
                    status = 5;
                }
                break;

            case 4:
                exec_LINECTRL_ALGO(); //Linienverfolgung
                break;

            default:
                // Handle unexpected status if necessary
                break;
        }
    }
    
    private void move(){
    	leftMotor.setPower(40);
        rightMotor.setPower(40);
    }

    
    private void exec_LINECTRL_ALGO(){
        leftMotor.forward();
        rightMotor.forward();
        
        // Farbwerte: 0 = weiß, 1 = grau, 2 = schwarz
        int sensorLeftValue = this.lineSensorLeft;
        int sensorRightValue = this.lineSensorRight;

        // Leistungsparameter
        int basePower = 30;  // Basisgeschwindigkeit
        int maxCorrection = 50;  // Maximale Abweichungskorrektur
        float p = 25;           // Coef. proporcional
        float i = 0.2f;         // Coef. integral (ajustar según sea necesario)
        double d = 0.8;            // Coef. derivativo

        // Variables para el controlador PID
        int previousError = 0;      // Error anterior
        int integral = 0;           // Suma acumulada del error

        // Calcular el error
        int error = sensorLeftValue - sensorRightValue;

        // Calcular la derivada del error (la diferencia entre el error actual y el error anterior)
        int derivative = error - previousError;
        
        // Calcular el término integral (suma acumulada del error)
        integral += error;
        
        // Controlador PID
        double correction = (p * error) + (i * integral) + (d * derivative);

        // Limitar los valores de corrección
        if (correction < -maxCorrection) { correction = -maxCorrection; }
        else if (correction > maxCorrection) { correction = maxCorrection; }

        // Ajuste de la potencia del motor
        int leftPower = (int) Math.round(basePower - correction);
        int rightPower = (int) Math.round(basePower + correction);

        // Limitar la potencia del motor a los valores permisibles
        if (leftPower < -100) { leftPower = -100; }
        else if (leftPower > 100) { leftPower = 100; }

        if (rightPower < -100) { rightPower = -100; }
        else if (rightPower > 100) { rightPower = 100; }

        // Asignar la potencia a los motores
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Guardar el error actual para el cálculo de la derivada en la siguiente iteración
        previousError = error;

        // MONITOR
        
        monitor.writeControlVar("LeftSensor", "" + sensorLeftValue);
        monitor.writeControlVar("RightSensor", "" + sensorRightValue);
        monitor.writeControlVar("RightWheelPower", "" + rightPower);
		monitor.writeControlVar("LeftWheelPower", "" + leftPower);
		monitor.writeControlVar("Correction", "" + correction);
		monitor.writeControlVar("Error", "" + error);
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
	
	// Aufgabe 3.2
	
	// Parámetros PID separados para cada rueda
	double KpLeft = 10, KiLeft = 0.1, KdLeft = 0.1;
	double KpRight = 30, KiRight = 0.15, KdRight = 0.2;

	private void drive(double v, double omega) {
	    // Parámetros del robot
	    double wheelRadius = 0.028;    
	    double distanceLength = 0.138;
	    double maxIntegral = 20; 
	    double PWM_SCALE = 1.2;

	    // Variables de control (Integrales y errores anteriores para cada rueda)
	    double integralLeft = 0, previousErrorLeft = 0;
	    double integralRight = 0, previousErrorRight = 0;

	    // Cálculo de las velocidades deseadas de las ruedas
	    double vLeftDesired = v - (omega * distanceLength / 2.0);
	    double vRightDesired = v + (omega * distanceLength / 2.0);

	    // Obtener las velocidades actuales de las ruedas
	    double vLeftActual = getWheelSpeed("left");
	    double vRightActual = getWheelSpeed("right");

	    // PID para el motor izquierdo
	    double errorLeft = vLeftDesired - vLeftActual;
	    integralLeft += errorLeft;
	    integralLeft = Math.max(-maxIntegral, Math.min(maxIntegral, integralLeft));
	    double derivativeLeft = errorLeft - previousErrorLeft;
	    double controlLeft = KpLeft * errorLeft + KiLeft * integralLeft + KdLeft * derivativeLeft;
	    previousErrorLeft = errorLeft;

	    // PID para el motor derecho
	    double errorRight = vRightDesired - vRightActual;
	    integralRight += errorRight;
	    integralRight = Math.max(-maxIntegral, Math.min(maxIntegral, integralRight));
	    double derivativeRight = errorRight - previousErrorRight;
	    double controlRight = KpRight * errorRight + KiRight * integralRight + KdRight * derivativeRight;
	    previousErrorRight = errorRight;

	    // Cálculo de los valores PWM
	    int powerLeft = (int) (controlLeft / wheelRadius * PWM_SCALE);
	    int powerRight = (int) (controlRight / wheelRadius * PWM_SCALE);

	    // Limitar los valores PWM al rango permitido [-100, 100]
	    powerLeft = Math.max(-100, (int) (controlLeft / wheelRadius * PWM_SCALE));
	    powerRight = Math.max(-100, (int) (controlRight / wheelRadius * PWM_SCALE));

	    // Aplicar potencia a los motores
	    leftMotor.setPower(powerLeft);
	    rightMotor.setPower(powerRight);

	    // Monitor para depuración
	    monitor.writeControlVar("vLeftDesired", "" + vLeftDesired);
	    monitor.writeControlVar("vRightDesired", "" + vRightDesired);
	    monitor.writeControlVar("vLeftActual", "" + vLeftActual);
	    monitor.writeControlVar("vRightActual", "" + vRightActual);
	    monitor.writeControlVar("ErrorLeft", "" + errorLeft);
	    monitor.writeControlVar("ErrorRight", "" + errorRight);
	    monitor.writeControlVar("PowerLeft", "" + powerLeft);
	    monitor.writeControlVar("PowerRight", "" + powerRight);
	}

	// Método para medir las velocidades simuladas de las ruedas
	private double getWheelSpeed(String wheel) {
	    IPerception.AngleDifferenceMeasurement angleMeasurement;
	    double wheelRadius = 0.028;

	    // Selección del encoder y cálculo del radio
	    if (wheel.equalsIgnoreCase("left")) {
	        angleMeasurement = navigation.getAngleMeasuremntLeft();
	    } else if (wheel.equalsIgnoreCase("right")) {
	        angleMeasurement = navigation.getAngleMeasuremntRight();
	    } else {
	        throw new IllegalArgumentException("Rueda no válida: " + wheel);
	    }

	    // Calcular la velocidad angular en grados por segundo
	    double angleSpeed = angleMeasurement.getAngleSum() / ((double) angleMeasurement.getDeltaT() / 1000);

	    // Convertir velocidad angular a velocidad lineal (m/s)
	    double linearSpeed = (angleSpeed * Math.PI * wheelRadius) / 180;

	    return linearSpeed;
	}

}