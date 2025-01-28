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
		  case DEMO_PRG2 	: update_DEMOPRG2_Parameter();
          					  exec_DEMOPRG2_ALGO();
          					  break;
		}

	}
	
	// Private methods


	private void update_DEMOPRG1_Parameter() {

	}
	
	private void update_DEMOPRG2_Parameter() {

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
	private void update_PARKCTRL_Parameter() {
	    // Aktuelle Pose des Roboters abrufen
	    Pose currentPose = navigation.getPose();

	    // Zielpose definieren (z. B. Mitte der Parkl�cke)
	    Pose targetPose = new Pose(80, 30, 0); // Beispielwerte: X, Y, Orientierung

	    // Calcular las diferencias
	    double dx = targetPose.getX() - currentPose.getX();
	    double dy = targetPose.getY() - currentPose.getY();
	    double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
	    double angleToTarget = Math.atan2(dy, dx);

	    // Calcular el error de orientación
	    double orientationError = angleToTarget - currentPose.getHeading();

	    // Querabweichung zur geplanten Bahn
	    double lateralError = computeLateralError(currentPose, targetPose.getX(), targetPose.getY());

	    // Aktualisieren der Regelparameter f�r Ein- und Ausparken
	    monitor.writeControlVar("CurrentX", String.valueOf(currentPose.getX()));
	    monitor.writeControlVar("CurrentY", String.valueOf(currentPose.getY()));
	    monitor.writeControlVar("TargetX", String.valueOf(targetPose.getX()));
	    monitor.writeControlVar("TargetY", String.valueOf(targetPose.getY()));
	    monitor.writeControlVar("DistanceToTarget", String.valueOf(distanceToTarget));
	    monitor.writeControlVar("OrientationError", String.valueOf(orientationError));
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
    
 // Aufgabe 3.3: Regelung der Geradeausfahrt
    private void exec_SETPOSE_ALGO() {
        // Zielposition definieren
        double targetX = 200.0; // Ziel in X-Richtung
        double targetY = 0.0;   // Ziel in Y-Richtung
        double targetTheta = 0.0; // Zielorientierung (Richtung der Gerade)

        // PID-Regelparameter
        double kp = 2.0;    // Proportionalverst�rker
        double ki = 0.1;    // Integralverst�rker
        double kd = 0.5;    // Differenzialverst�rker

        // Initialisierung von Variablen
        double lastError = 0.0;
        double integral = 0.0;

        // Solange Ziel nicht erreicht
        while (true) {
            // Aktuelle Pose abrufen
            Pose currentPose = navigation.getPose();
            double currentX = currentPose.getX();
            double currentY = currentPose.getY();
            double currentTheta = currentPose.getHeading();

            // Abbruchbedingung: Ziel erreicht (inkl. Orientierung)
            if (Math.abs(currentX - targetX) < 1.0 && 
                Math.abs(currentY - targetY) < 1.0 &&
                Math.abs(currentTheta - targetTheta) < 0.1) {
                break;
            }

            // Berechnung der Querabweichung
            double error = computeLateralError(currentPose, targetX, targetY);

            // PID-Regler
            integral += error;
            double derivative = error - lastError;
            double correction = kp * error + ki * integral + kd * derivative;

            // Dynamische Geschwindigkeit basierend auf Querabweichung
            double v = Math.max(5.0, 20.0 - Math.abs(error)); // Zwischen 5 und 20 cm/s
            double omega = correction; // Korrektur der Drehgeschwindigkeit

            // In Radgeschwindigkeiten umrechnen und anwenden
            drive(v, omega);

            // Fehler aktualisieren
            lastError = error;

            // Pause f�r Stabilit�t
            Delay.msDelay(50);
        }

        // Motoren stoppen, wenn Ziel erreicht
        stop();
    }

    /**
     * Berechnet die Querabweichung zur Zielgeraden.
     * Diese Methode ber�cksichtigt die Orientierung des Roboters.
     */
    private double computeLateralError(Pose pose, double targetX, double targetY) {
        double x = pose.getX();
        double y = pose.getY();
        double theta = pose.getHeading();

        // Abstand des Roboters von der Linie (projektion auf die Normale)
        double dx = targetX - x;
        double dy = targetY - y;

        return -dy * Math.cos(theta) + dx * Math.sin(theta);
    }
	/**
	 * PARKING along the generated path
	 */
    private void exec_PARKCTRL_ALGO() {
        // Coeficientes del polinomio de tercer grado (ajustables seg�n el comportamiento deseado)
        double a = 0.5;
        double b = 0.0;
        double c = 0.0;
        double d = 0.0;

        // Tiempo inicial (en milisegundos)
        long startTime = System.currentTimeMillis();

        // Tiempo total del movimiento (ajustable)
        long totalTime = 10000; // 10 segundos

        while (System.currentTimeMillis() - startTime <= totalTime) {
            // Calcular el tiempo actual normalizado entre 0 y 1
            double t = (double) (System.currentTimeMillis() - startTime) / totalTime;

            // Calcular la posici�n deseada y sus derivadas
            double y = a * Math.pow(t, 3) + b * Math.pow(t, 2) + c * t + d;        // Posici�n
            double dy = 3 * a * Math.pow(t, 2) + 2 * b * t + c;                    // Velocidad lineal
            double ddy = 6 * a * t + 2 * b;                                       // Aceleraci�n (opcional para control avanzado)

            // Velocidad lineal y angular deseada
            double v = dy;         // Velocidad lineal proporcional a la derivada
            double omega = ddy;    // Velocidad angular proporcional a la segunda derivada

            // Enviar comandos de velocidad al controlador
            drive(v, omega);

            // Pausar brevemente para evitar sobrecargar el procesador
            try {
                Thread.sleep(50); // 50 ms
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Detener los motores al finalizar
        drive(0, 0);
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
    
    int status = 6;
    
    private void exec_DEMOPRG1_ALGO() {
        leftMotor.forward();
        rightMotor.forward();
        navigation.setUseOnlyOdometry(true);
        
        switch (status) {
            case 0: // Geradeausfahrt 120 cm mit 10 cm/s
            	KpLeft = 0.7; KiLeft = 0.6; KdLeft = 0.2;
            	KpRight = 0.6; KiRight = 0.75; KdRight = 0.3;
                drive(1, 0); // Velocidad 10 cm/s, sin rotaci�n
                if (navigation.getPose().getX() * 100 >= 120) {
                    stop();
                    status = 1;
                    break;
                }
                break;

            case 1: // Rotaci�n a 15�/s
            	KpLeft = 0.8; KiLeft = 1.9; KdLeft = 0.05;
        		KpRight = 0.8; KiRight = 1.5; KdRight = 0.05;
                drive(0, 15); // Rotaci�n a 15�/s
                if (navigation.getPose().getHeading() * (180 / Math.PI) >= 70){
                    stop();
                    status = 2;
                    break;
                }
                break;

            case 2: // Avanzar en l�nea recta 30 cm a 5 cm/s
            	KpLeft = 1.4; KiLeft = 1.4; KdLeft = 0.4;
            	KpRight = 1.2; KiRight = 1.5; KdRight = 0.4;
                drive(0.5, 0); // Velocidad 5 cm/s
                if (navigation.getPose().getY() * 100 >= 25) {
                    stop();
                    status = 3;
                    break;
                }
                break;

            case 3: // Rotaci�n a -30�/s
            	KpLeft = 0.4; KiLeft = 0.95; KdLeft = 0.025;
        		KpRight = 0.4; KiRight = 0.75; KdRight = 0.025;
                drive(0, -30); // Rotaci�n a -30�/s
                if (navigation.getPose().getHeading() * 180 / Math.PI <= 0) {
                    stop();
                    status = 4; // Cambia al seguimiento de l�nea
                    break;
                }
                break;

            case 4: // Seguimiento de l�nea
            	update_LINECTRL_Parameter();
                exec_LINECTRL_ALGO();
                break;

            default:
                LCD.drawString("Unexpected status: " + status, 0, 7);
                break;
        }
    }
    
    private void exec_DEMOPRG2_ALGO() {
        leftMotor.forward();
        rightMotor.forward();
        navigation.setUseOnlyOdometry(true);
        
      	update_PARKCTRL_Parameter();
    	exec_PARKCTRL_ALGO();
        
        /*switch (status) {
            case 0: // Geradeausfahrt 120 cm mit 10 cm/s
            	KpLeft = 0.7; KiLeft = 0.5; KdLeft = 0.3;
            	KpRight = 0.6; KiRight = 0.7; KdRight = 0.4;
                drive(1, 0); // Velocidad 10 cm/s, sin rotaci�n
                if (navigation.getPose().getX() * 100 >= 120) {
                    stop();
                    status = 1;
                    break;
                }
                break;

            case 1: // Rotaci�n a 15�/s
            	KpLeft = 0.8; KiLeft = 1.9; KdLeft = 0.05;
        		KpRight = 0.8; KiRight = 1.5; KdRight = 0.05;
                drive(0, 15); // Rotaci�n a 15�/s
                if (navigation.getPose().getHeading() * (180 / Math.PI) >= 70){
                    stop();
                    status = 2;
                    break;
                }
                break;

            case 2: // Avanzar en l�nea recta 30 cm a 5 cm/s
            	KpLeft = 1.4; KiLeft = 1; KdLeft = 0.6;
            	KpRight = 1.2; KiRight = 1.5; KdRight = 0.8;
                drive(0.5, 0); // Velocidad 5 cm/s
                if (navigation.getPose().getY() * 100 >= 30) {
                    stop();
                    status = 3;
                    break;
                }
                break;

            case 3: // Rotaci�n a -30�/s
            	KpLeft = 0.4; KiLeft = 0.95; KdLeft = 0.025;
        		KpRight = 0.4; KiRight = 0.75; KdRight = 0.025;
                drive(0, -30); // Rotaci�n a -30�/s
                if (navigation.getPose().getHeading() * 180 / Math.PI <= 0) {
                    stop();
                    navigation.setPose(0, 0, 0);
                    status = 4; // Cambia al seguimiento de l�nea
                    break;
                }
                break;

            case 4: // Seguimiento de l�nea
            	update_LINECTRL_Parameter();
                exec_LINECTRL_ALGO();
                if (navigation.getPose().getX() * 100 <= -117) {
                    stop();
                    status = 5;
                    break;
                }
                
                break;
                
            case 5: // Rotaci�n
            	KpLeft = 0.8; KiLeft = 1.9; KdLeft = 0.05;
        		KpRight = 0.8; KiRight = 1.5; KdRight = 0.05;
                drive(0, 15); // Rotaci�n a 15�/s
                if (navigation.getPose().getHeading() * (180 / Math.PI) >= -30){
                    stop();
                    navigation.setPose(0, 0, 0);
                    status = 6;
                    break;
                }
                break;
                
            case 6: // Geradeausfahrt 120 cm mit 10 cm/s
            	KpLeft = 0.7; KiLeft = 0.6; KdLeft = 0.3;
            	KpRight = 0.6; KiRight = 0.7; KdRight = 0.35;
                drive(1, 0); // Velocidad 10 cm/s, sin rotaci�n
                if (navigation.getPose().getX() * 100 >= 70) {
                    stop();
                    status = 7;
                    break;
                }
                break;
                
            case 7:
            	//update_PARKCTRL_Parameter();
            	//exec_PARKCTRL_ALGO();
            	break;

            default:
                LCD.drawString("Unexpected status: " + status, 0, 7);
                break;
        }*/
    }
    
    private void exec_LINECTRL_ALGO() {
    	// Beide Motoren starten vorwärts
        leftMotor.forward();
        rightMotor.forward();

        // Sensorwerte: 0 = weiß, 1 = grau, 2 = schwarz
        int sensorLeftValue = this.lineSensorLeft;
        int sensorRightValue = this.lineSensorRight;

        // Leistungsparameter für Kurven
        int basePowerCorner = 30;    // Basisgeschwindigkeit für Kurven
        int maxCorrectionCorner = 50; // Maximale Korrektur für Kurven
        float pCorner = 25;          // Proportionalfaktor für Kurven
        float iCorner = 0.2f;        // Integralfaktor für Kurven
        double dCorner = 0.8;        // Differentialfaktor für Kurven
        
        // Leistungsparameter für gerade Linien
        int basePowerStraight = 30;   // Basisgeschwindigkeit für gerade Linien
        int maxCorrectionStraight = 20; // Maximale Korrektur für gerade Linien
        float pStraight = 20;        // Proportionalfaktor für gerade Linien
        float iStraight = 0.05f;     // Integralfaktor für gerade Linien
        double dStraight = 2;        // Differentialfaktor für gerade Linien

        // Variablen für den PID-Regler (Kurven)
        int previousError = 0;      // Vorheriger Fehler
        int integral = 0;           // Kumulierte Fehler-Summe

     // Erkennung: gerade Linie oder Kurve
        boolean isCorner = (sensorLeftValue == 2 || sensorRightValue == 2); // Kurve erkannt, wenn einer der Sensoren schwarz meldet

        if (isCorner) {
        	// PID-Regler für Kurven

        	// Fehler berechnen
            int error = sensorLeftValue - sensorRightValue;

            // Fehlerableitung berechnen
            int derivative = error - previousError;

            // Integralwert berechnen
            integral += error;

            // PID-Regler-Berechnung
            double correction = (pCorner * error) + (iCorner * integral) + (dCorner * derivative);

         // Korrektur begrenzen
            if (correction < -maxCorrectionCorner) { correction = -maxCorrectionCorner; }
            else if (correction > maxCorrectionCorner) { correction = maxCorrectionCorner; }

            // Motorleistung anpassen
            int leftPower = (int) Math.round(basePowerCorner - correction);
            int rightPower = (int) Math.round(basePowerCorner + correction);

            // Motorleistung auf -100 bis 100 begrenzen
            leftPower = Math.max(-100, Math.min(100, leftPower));
            rightPower = Math.max(-100, Math.min(100, rightPower));

            // Leistung an die Motoren senden
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Fehler speichern
            previousError = error;

            // Überwachungsinformationen
            monitor.writeControlVar("Mode", "Corner");
            monitor.writeControlVar("LeftSensor", "" + sensorLeftValue);
            monitor.writeControlVar("RightSensor", "" + sensorRightValue);
            monitor.writeControlVar("RightWheelPower", "" + rightPower);
            monitor.writeControlVar("LeftWheelPower", "" + leftPower);
            monitor.writeControlVar("Correction", "" + correction);
            monitor.writeControlVar("Error", "" + error);
        } else {
        	// PID-Regler für gerade Linien

            // Fehler berechnen
            int error = sensorLeftValue - sensorRightValue;

            // Fehlerableitung berechnen
            int derivative = error - previousError;

            // Integralwert berechnen
            integral += error;

            // PID-Regler-Berechnung
            double correction = (pStraight * error) + (iStraight * integral) + (dStraight * derivative);

            // Korrektur begrenzen
            if (correction < -maxCorrectionStraight) { correction = -maxCorrectionStraight; }
            else if (correction > maxCorrectionStraight) { correction = maxCorrectionStraight; }

            // Motorleistung anpassen
            int leftPower = (int) Math.round(basePowerStraight - correction);
            int rightPower = (int) Math.round(basePowerStraight + correction);

            // Motorleistung auf -100 bis 100 begrenzen
            leftPower = Math.max(-100, Math.min(100, leftPower));
            rightPower = Math.max(-100, Math.min(100, rightPower));

            // Leistung an die Motoren senden
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // Fehler speichern
            previousError = error;

            // Überwachungsinformationen
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
	
	// Parámetros PID separados para cada rueda

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