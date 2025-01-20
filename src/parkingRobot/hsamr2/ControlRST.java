package parkingRobot.hsamr2;


import lejos.robotics.navigation.Pose;
import lejos.util.Delay;
import parkingRobot.IControl;
import parkingRobot.IMonitor;
import parkingRobot.IPerception;                 
import parkingRobot.IPerception.*;
import lejos.nxt.LCD;
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
	
	double KpLeft = 0, KiLeft = 0, KdLeft = 0;
	double KpRight = 0, KiRight = 0, KdRight = 0;
	
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
		// Aktuelle Pose des Roboters abrufen
	    Pose currentPose = navigation.getPose();

	    // Zielpose (z. B. Mitte der Parklücke) definieren
	    Pose targetPose = new Pose(80, 30, 0); // Beispielwerte: X, Y, Orientierung

	    // Berechnung der Abweichungen zur Zielpose
	    double dx = targetPose.getX() - currentPose.getX();
	    double dy = targetPose.getY() - currentPose.getY();
	    double distanceToTarget = Math.sqrt(dx * dx + dy * dy); // Abstand zum Ziel
	    double angleToTarget = Math.atan2(dy, dx); // Winkel zum Ziel relativ zur aktuellen Position

	    // Unterschied zwischen aktueller und Zielorientierung
	    double orientationError = angleToTarget - currentPose.getHeading();

	    // Querabweichung zur geplanten Bahn
	    double lateralError = computeLateralError(currentPose, targetPose.getX(), targetPose.getY());

	    // Aktualisieren der Regelparameter für Ein- und Ausparken
	    monitor.writeControlVar("CurrentX", String.valueOf(currentPose.getX()));
	    monitor.writeControlVar("CurrentY", String.valueOf(currentPose.getY()));
	    monitor.writeControlVar("TargetX", String.valueOf(targetPose.getX()));
	    monitor.writeControlVar("TargetY", String.valueOf(targetPose.getY()));
	    monitor.writeControlVar("DistanceToTarget", String.valueOf(distanceToTarget));
	    monitor.writeControlVar("OrientationError", String.valueOf(orientationError));
	    monitor.writeControlVar("LateralError", String.valueOf(lateralError));
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
    	// Zielposition definieren
        double targetX = 200.0; // Ziel in X-Richtung
        double targetY = 0.0;   // Ziel in Y-Richtung
        //double targetTheta = 0.0; // Zielorientierung

        // PID-Regelparameter
        double kp = 2.0;    // Proportionalverstärker
        double ki = 0.1;    // Integralverstärker
        double kd = 0.5;    // Differenzialverstärker

        // Initialisierung von Variablen
        double lastError = 0.0;
        double integral = 0.0;

        // Solange Ziel nicht erreicht
        while (true) {
            // Aktuelle Pose abrufen
            Pose currentPose = navigation.getPose();
            double currentX = currentPose.getX();
            double currentY = currentPose.getY();
            //double currentTheta = currentPose.getHeading();

            // Abbruchbedingung: Ziel erreicht
            if (Math.abs(currentX - targetX) < 1.0 && Math.abs(currentY - targetY) < 1.0) {
                break;
            }

            // Berechnung der Querabweichung
            double error = computeLateralError(currentPose, targetX, targetY);

            // PID-Regler
            integral += error;
            double derivative = error - lastError;
            double correction = kp * error + ki * integral + kd * derivative;

            // Steuergrößen berechnen
            double v = 10.0; // Konstante Geschwindigkeit
            double omega = correction;

            // In Radgeschwindigkeiten umrechnen und anwenden
            drive(v, omega);

            // Fehler aktualisieren
            lastError = error;

            // Pause für Stabilität
            Delay.msDelay(50);
        }

        // Motoren stoppen, wenn Ziel erreicht
        stop();
    }

    /**
     * Berechnet die Querabweichung zur Zielgeraden.
     */
    private double computeLateralError(Pose pose, double targetX, double targetY) {
        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getHeading();

        double dx = targetX - x;
        double dy = targetY - y;

        return -dy * Math.cos(theta) + dx * Math.sin(theta);
    }
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
		// Zielpositionen definieren (Start- und Endpunkt)
	    Pose startPose = navigation.getPose();
	    Pose targetPose = new Pose(100, 50, 0); // Beispielwerte: Zielposition
	    
	    // Bahnplanung: Hier eine einfache Kurve (kann durch ein Polynom ersetzt werden)
	    double t = 0.0; // Parameter für die Bahn (0 ≤ t ≤ 1)
	    double dt = 0.05; // Schrittweite
	    
	    while (t <= 1.0) {
	        // Berechne die aktuelle Zielposition entlang der Bahn
	        double x = startPose.getX() + t * (targetPose.getX() - startPose.getX());
	        double y = startPose.getY() + t * (targetPose.getY() - startPose.getY());
	        
	        // Berechne die Stellgrößen v und ω
	        double dx = x - navigation.getPose().getX();
	        double dy = y - navigation.getPose().getY();
	        double distance = Math.sqrt(dx * dx + dy * dy);
	        double angle = Math.atan2(dy, dx);
	        
	        double v = Math.min(10, distance); // Geschwindigkeit begrenzen
	        double omega = angle - navigation.getPose().getHeading(); // Drehwinkel
	        
	        // Umsetzen der Stellgrößen in Radgeschwindigkeiten
	        drive(v, omega);
	        
	        // Überprüfen, ob das Ziel erreicht wurde
	        if (distance < 2.0) { // Toleranzradius
	            break;
	        }
	        
	        // Zeitinkrement
	        t += dt;
	        Delay.msDelay(50);
	    }
	    
	    // Motoren stoppen
	    stop();
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
        navigation.setUseOnlyOdometry(true);
        
        //double TOLERANCE = 2.0; // Tolerancia para comparación en cm o grados

        switch (status) {
            case 0: // Geradeausfahrt 120 cm mit 10 cm/s
            	KpLeft = 0.005; KiLeft = 1.3; KdLeft = 0.8;
            	KpRight = 0.006; KiRight = 1.2; KdRight = 0.6;
                drive(10, 0); // Velocidad 10 cm/s, sin rotación
                if (navigation.getPose().getX() * 100 >= 120) {
                    stop();
                    status = 1;
                    break;
                }
                break;

            case 1: // Rotación a 15°/s
            	KpLeft = 1; KiLeft = 2; KdLeft = 0.05;
        		KpRight = 1; KiRight = 1.5; KdRight = 0.1;
                drive(0, 15); // Rotación a 15°/s
                if (navigation.getPose().getHeading() * (180 / Math.PI) >= 90){
                    stop();
                    status = 2;
                    break;
                }
                break;

            case 2: // Avanzar en línea recta 30 cm a 5 cm/s
            	KpLeft = 0.1; KiLeft = 0.3; KdLeft = 10;
            	KpRight = 0.1; KiRight = 0.5; KdRight = 15;
                drive(5, 0); // Velocidad 5 cm/s
                if (navigation.getPose().getY() * 100 >= 70) {
                    stop();
                    //status = 3;
                    break;
                }
                break;

            case 3: // Rotación a -30°/s
            	KpLeft = 1; KiLeft = 0.01; KdLeft = 0;
        		KpRight = 1; KiRight = 0.08; KdRight = 0.12;
                drive(0, -30); // Rotación a -30°/s
                if (navigation.getPose().getHeading() * 180 / Math.PI == -90) {
                    stop();
                    status = 4; // Cambia al seguimiento de línea
                    break;
                }
                break;

            case 4: // Seguimiento de línea
                exec_LINECTRL_ALGO();
                break;

            default:
                LCD.drawString("Unexpected status: " + status, 0, 7);
                break;
        }
    }

    
    /*private void move(){
    	leftMotor.setPower(40);
        rightMotor.setPower(40);
    }*/

    
    private void exec_LINECTRL_ALGO() {
        leftMotor.forward();
        rightMotor.forward();

        // Valores de los sensores: 0 = blanco, 1 = gris, 2 = negro
        int sensorLeftValue = this.lineSensorLeft;
        int sensorRightValue = this.lineSensorRight;

        // Parámetros de potencia para esquinas
        int basePowerCorner = 30;    // Velocidad base para esquinas
        int maxCorrectionCorner = 50; // Corrección máxima para esquinas
        float pCorner = 25;          // Coeficiente proporcional para esquinas
        float iCorner = 0.2f;        // Coeficiente integral para esquinas
        double dCorner = 0.8;        // Coeficiente derivativo para esquinas
        
        //float pCorner = 5;          // Coeficiente proporcional para esquinas
        //float iCorner = 8.0f;        // Coeficiente integral para esquinas
        //double dCorner = 0;        // Coeficiente derivativo para esquinas
        
        // Parámetros de potencia para línea recta
        int basePowerStraight = 30;   // Velocidad base para línea recta
        int maxCorrectionStraight = 20; // Corrección máxima para línea recta
        float pStraight = 20;        // Coeficiente proporcional para línea recta
        float iStraight = 0.05f;     // Coeficiente integral para línea recta
        double dStraight = 2;        // Coeficiente derivativo para línea recta

        // Variables para el controlador PID (esquinas)
        int previousError = 0;      // Error anterior
        int integral = 0;           // Suma acumulada del error

        // Detectar si está en línea recta o esquina
        boolean isCorner = (sensorLeftValue == 2 || sensorRightValue == 2); // Esquinas detectadas si alguno está en negro

        if (isCorner) {
            // Controlador PID para esquinas

            // Calcular el error
            int error = sensorLeftValue - sensorRightValue;

            // Calcular la derivada del error
            int derivative = error - previousError;

            // Calcular el término integral
            integral += error;

            // Controlador PID
            double correction = (pCorner * error) + (iCorner * integral) + (dCorner * derivative);

            // Limitar la corrección
            if (correction < -maxCorrectionCorner) { correction = -maxCorrectionCorner; }
            else if (correction > maxCorrectionCorner) { correction = maxCorrectionCorner; }

            // Ajustar potencia de los motores
            int leftPower = (int) Math.round(basePowerCorner - correction);
            int rightPower = (int) Math.round(basePowerCorner + correction);

            // Limitar la potencia del motor
            leftPower = Math.max(-100, Math.min(100, leftPower));
            rightPower = Math.max(-100, Math.min(100, rightPower));

            // Asignar la potencia a los motores
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Guardar el error actual
            previousError = error;

            // MONITOR
            monitor.writeControlVar("Mode", "Corner");
            monitor.writeControlVar("LeftSensor", "" + sensorLeftValue);
            monitor.writeControlVar("RightSensor", "" + sensorRightValue);
            monitor.writeControlVar("RightWheelPower", "" + rightPower);
            monitor.writeControlVar("LeftWheelPower", "" + leftPower);
            monitor.writeControlVar("Correction", "" + correction);
            monitor.writeControlVar("Error", "" + error);
        } else {
            // Controlador PID para líneas rectas

            // Calcular el error
            int error = sensorLeftValue - sensorRightValue;

            // Calcular la derivada del error
            int derivative = error - previousError;

            // Calcular el término integral
            integral += error;

            // Controlador PID
            double correction = (pStraight * error) + (iStraight * integral) + (dStraight * derivative);

            // Limitar la corrección
            if (correction < -maxCorrectionStraight) { correction = -maxCorrectionStraight; }
            else if (correction > maxCorrectionStraight) { correction = maxCorrectionStraight; }

            // Ajustar potencia de los motores
            int leftPower = (int) Math.round(basePowerStraight - correction);
            int rightPower = (int) Math.round(basePowerStraight + correction);

            // Limitar la potencia del motor
            leftPower = Math.max(-100, Math.min(100, leftPower));
            rightPower = Math.max(-100, Math.min(100, rightPower));

            // Asignar la potencia a los motores
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Guardar el error actual
            previousError = error;

            // MONITOR
            monitor.writeControlVar("Mode", "Straight");
            monitor.writeControlVar("LeftSensor", "" + sensorLeftValue);
            monitor.writeControlVar("RightSensor", "" + sensorRightValue);
            monitor.writeControlVar("RightWheelPower", "" + rightPower);
            monitor.writeControlVar("LeftWheelPower", "" + leftPower);
            monitor.writeControlVar("Correction", "" + correction);
            monitor.writeControlVar("Error", "" + error);
        }
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
	
	// ParÃ¡metros PID separados para cada rueda

	private void drive(double v, double omega) {
	    // Parámetros del robot
	    double wheelRadius = 0.028;
	    double distanceLength = 0.138;
	    double maxIntegral = 20;
	    double PWM_SCALE = 1.2;

	    // Factores de calibración para compensar diferencias entre las ruedas
	    double calibrationFactorLeft = 1.0;  // Ajustar según pruebas
	    double calibrationFactorRight = 1.05; // Ajustar según pruebas

	    // Variables de control (Integrales y errores anteriores para cada rueda)
	    double integralLeft = 0, previousErrorLeft = 0;
	    double integralRight = 0, previousErrorRight = 0;

	    // Cálculo de las velocidades deseadas de las ruedas
	    double vLeftDesired = v - (omega * distanceLength / 2.0);
	    double vRightDesired = v + (omega * distanceLength / 2.0);

	    // Obtener las velocidades actuales de las ruedas con suavizado
	    double alpha = 0.7;  // Factor de suavizado para reducir ruido
	    double vLeftActual = alpha * getWheelSpeed("left") + (1 - alpha) * getWheelSpeed("left");
	    double vRightActual = alpha * getWheelSpeed("right") + (1 - alpha) * getWheelSpeed("right");

	    // PID para el motor izquierdo
	    double errorLeft = vLeftDesired - vLeftActual;
	    if (Math.abs(errorLeft) < 0.01) errorLeft = 0; // Ignorar errores pequeños

	    integralLeft += errorLeft;
	    integralLeft = Math.max(-maxIntegral, Math.min(maxIntegral, integralLeft)); // Antiwindup

	    if (Math.signum(errorLeft) != Math.signum(previousErrorLeft)) integralLeft = 0; // Reset si cambia de signo

	    double derivativeLeft = errorLeft - previousErrorLeft;
	    double controlLeft = KpLeft * errorLeft + KiLeft * integralLeft + KdLeft * derivativeLeft;
	    previousErrorLeft = errorLeft;

	    // PID para el motor derecho
	    double errorRight = vRightDesired - vRightActual;
	    if (Math.abs(errorRight) < 0.01) errorRight = 0; // Ignorar errores pequeños

	    integralRight += errorRight;
	    integralRight = Math.max(-maxIntegral, Math.min(maxIntegral, integralRight)); // Antiwindup

	    if (Math.signum(errorRight) != Math.signum(previousErrorRight)) integralRight = 0; // Reset si cambia de signo

	    double derivativeRight = errorRight - previousErrorRight;
	    double controlRight = KpRight * errorRight + KiRight * integralRight + KdRight * derivativeRight;
	    previousErrorRight = errorRight;

	    // Cálculo de los valores PWM con factores de calibración
	    int powerLeft = (int) (controlLeft / wheelRadius * PWM_SCALE * calibrationFactorLeft);
	    int powerRight = (int) (controlRight / wheelRadius * PWM_SCALE * calibrationFactorRight);

	    // Limitar los valores PWM al rango permitido [-100, 100]
	    powerLeft = Math.max(-100, Math.min(100, powerLeft));
	    powerRight = Math.max(-100, Math.min(100, powerRight));

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
	    monitor.writeControlVar("LateralDeviation", "" + (vLeftActual - vRightActual));
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
