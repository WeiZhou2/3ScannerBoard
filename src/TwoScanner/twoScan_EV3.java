package TwoScanner;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class twoScan_EV3 {
	
	private final int MAX_MOTOR_PORTS = 4;
	
	private RegulatedMotor[] motorArray;
	
	private EV3ColorSensor colorSensor1; //right
	private EV3ColorSensor colorSensor2;//mid
	private EV3ColorSensor colorSensor3;//Left
	
	private int currentRotation[];
	private int side = 0; // 0 == LEFT, 1 == RIGHT
	
	public twoScan_EV3() {
		
		initializeMotors();
		System.out.println("Init Motors");
		initializeSensors();
		System.out.println("Init Sensors");
		
		System.out.println("Init Complete");
		System.out.println("Continue = ESC\nLeft Side Scan\n=LEFT\nRight Side Scan\n=RIGHT");
		
		Sound.setVolume(50);
		Sound.playTone(100, 1000);
		
		for(int i=0; i<4; i++) {
			Button.LEDPattern(i);
			Delay.msDelay(200);
		}
		Button.LEDPattern(0);

		moveBackward(20, 400);
		while(!isEscDown()) {
			if(Button.UP.isDown()) {
				side = 0;
				System.out.println("SCAN == Two Scanner");
			}
//			
//			if(Button.RIGHT.isDown()) {
//				side = 1;
//				System.out.println("SCAN == RIGHT");
//			}
			// Endless loop till user presses ESC on the EV3
		}
		Delay.msDelay(2000);
	}
	
	//================== Start Initialization Methods ==================
	// Creates the initial motors (Should only be called once.)
	public void initializeMotors() {
		motorArray = new RegulatedMotor[4];
		currentRotation=new int[4];
		currentRotation[0]=0;
		currentRotation[1]=0;
		currentRotation[2]=0;
		currentRotation[3]=0;
//		
		//B Left A back C right
		motorArray[0] = new EV3LargeRegulatedMotor(MotorPort.A);//new EV3LargeRegulatedMotor(MotorPort.A);
		motorArray[1] = new EV3LargeRegulatedMotor(MotorPort.B);
		motorArray[2] = new EV3LargeRegulatedMotor(MotorPort.C);
		motorArray[3] = new EV3LargeRegulatedMotor(MotorPort.D);
		
		
		for (RegulatedMotor motor: motorArray) {
			motor.setAcceleration(400);
			motor.setSpeed(400);
		}
	}
	
	// Creates the initial sensors (Should only be called once.)
	public void initializeSensors() {
		colorSensor1 = new EV3ColorSensor(SensorPort.S1);
		colorSensor1.setCurrentMode("RGB");
		colorSensor2 = new EV3ColorSensor(SensorPort.S2);
		colorSensor2.setCurrentMode("RGB");
		colorSensor3 = new EV3ColorSensor(SensorPort.S3);
		colorSensor3.setCurrentMode("RGB");
	}
	//================== End of Initialization Methods ==================
	
	//================== Start of Other Methods ==================
	public int[] getCurrentRotation() {
		return currentRotation;
	}
	
	public boolean isEscDown() {
		return Button.ESCAPE.isDown();
	}
	
	public int getSideScan() {
		return side;
	}
	//================== End of Other Methods ==================
	
	//================== Start of Motor Methods ==================\
	public void moveNoStop()
	{
		motorArray[1].setSpeed(100);
		motorArray[2].setSpeed(100);
		
		motorArray[1].forward();
		motorArray[2].backward();
	}
	public void shoot(int delay) {
		try {
				motorArray[3].rotate(360);
				
				Delay.msDelay(delay);
				
			} catch(Exception e){
				e.printStackTrace();
			}
	}
	
	public void delay(int delay){
		Delay.msDelay(delay);
	}
	
//	public void moveBackward(int speed, int delay) {
//		try {
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//			motorArray[1].backward();
//			motorArray[2].forward();
//			
//			
//			
//			Delay.msDelay(delay);
//		
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//
//			Delay.msDelay(500);
//		} catch(Exception e){
//			e.printStackTrace();
//		}
//	}
//	
//	public void moveForward(int speed, int delay) {
//		try {
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//			motorArray[1].forward();
//			motorArray[2].backward();
//			
//			
//			
//			Delay.msDelay(delay);
//		
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//			Delay.msDelay(500);
//		} catch(Exception e){
//			e.printStackTrace();
//		}
//	}
//	public void moveLeft(int speed, int delay) {
//		try {
//
//			motorArray[0].setSpeed(speed);
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//
//			motorArray[0].backward();
//			motorArray[1].backward();
//			motorArray[2].backward();
//			
//			Delay.msDelay(delay);
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//			Delay.msDelay(500);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//	
//	public void moveRight(int speed, int delay) {
//		try {
//			motorArray[0].setSpeed(speed);
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//
//			motorArray[0].forward();
//			motorArray[1].forward();
//			motorArray[2].forward();
//			
//			Delay.msDelay(delay);
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//
//			Delay.msDelay(500);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//	
//	public void moveLeftBack(int speed, int delay) {
//		try {
//
//			motorArray[0].setSpeed(speed);
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//
//			motorArray[0].forward();
//			motorArray[1].backward();
//			
//			Delay.msDelay(delay);
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//			Delay.msDelay(500);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//	
//	public void moveRightBack(int speed, int delay) {
//		try {
//
//			motorArray[0].setSpeed(speed);
//			motorArray[1].setSpeed(speed);
//			motorArray[2].setSpeed(speed);
//			
//
//			motorArray[0].backward();
//			motorArray[2].forward();
//			
//			Delay.msDelay(delay);
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//			Delay.msDelay(500);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
	//rotate unsucueesful code
	public void moveForward(int rotation, int delay) {
		try {
			System.out.println("Forward: "+rotation);
			
			currentRotation[1]+=rotation;
			currentRotation[2]+=-rotation;
			System.out.println(currentRotation[0]+"  "+currentRotation[1]+"  "+currentRotation[2]);
			motorArray[1].rotateTo(currentRotation[1],true);
			motorArray[2].rotateTo(currentRotation[2],true);
			Delay.msDelay(delay);
		
			motorArray[1].stop(true);
			motorArray[2].stop(true);
			Delay.msDelay(500);
		} catch(Exception e){
			e.printStackTrace();
		}
	}
	public void moveBackward(int rotation, int delay) {
		try {
			System.out.println("Backward: "+rotation);
			
			currentRotation[1]+=-rotation;
			currentRotation[2]+=rotation;
			System.out.println(currentRotation[0]+"  "+currentRotation[1]+"  "+currentRotation[2]);
			motorArray[1].rotateTo(currentRotation[1],true);
			motorArray[2].rotateTo(currentRotation[2],true);
			Delay.msDelay(delay);
		
			motorArray[1].stop(true);
			motorArray[2].stop(true);

			Delay.msDelay(500);
		} catch(Exception e){
			e.printStackTrace();
		}
	}
	public void moveLeftForward(int rotation, int delay) {
		try {
			System.out.println("LeftForward: "+rotation);
			currentRotation[0]+=rotation;
			currentRotation[1]-=rotation;
			
			motorArray[0].rotateTo(currentRotation[0],true);
			motorArray[1].rotateTo(currentRotation[1],true);
			Delay.msDelay(delay);
		
			motorArray[0].stop(true);
			motorArray[1].stop(true);

			Delay.msDelay(500);
		} catch(Exception e){
			e.printStackTrace();
		}
	}
	
	public void moveRightForward(int rotation, int delay) {
		try {
			System.out.println("RightForward: "+rotation);
			currentRotation[0]+=-rotation;
			currentRotation[2]+=rotation;
			
			motorArray[0].rotateTo(currentRotation[0],true);
			motorArray[2].rotateTo(currentRotation[2],true);
			Delay.msDelay(delay);
		
			motorArray[0].stop(true);
			motorArray[2].stop(true);

			Delay.msDelay(500);
		} catch(Exception e){
			e.printStackTrace();
		}
	}
	
	
	
	public void moveLeft(int rotation, int delay) {
		try {
			currentRotation[0]+=rotation;
			currentRotation[1]-=rotation/2;
			currentRotation[2]-=rotation/2;
			
			System.out.println(currentRotation[0]+"  "+currentRotation[1]+"  "+currentRotation[2]);
			
			motorArray[1].setSpeed(200);
			motorArray[2].setSpeed(200);
			motorArray[1].setAcceleration(200);
			motorArray[2].setAcceleration(200);
			
			motorArray[0].rotateTo(currentRotation[0],true);
			motorArray[1].rotateTo(currentRotation[1],true);
			motorArray[2].rotateTo(currentRotation[2],true);
			Delay.msDelay(delay);
			motorArray[0].stop(true);
			motorArray[1].stop(true);
			motorArray[2].stop(true);

			motorArray[1].setSpeed(400);
			motorArray[2].setSpeed(400);
			motorArray[1].setAcceleration(400);
			motorArray[2].setAcceleration(400);
			Delay.msDelay(500);
			
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public void moveRight(int rotation, int delay) {
		try {
			currentRotation[0]-=rotation;
			currentRotation[1]+=rotation/2;
			currentRotation[2]+=rotation/2;
			
			System.out.println(currentRotation[0]+"  "+currentRotation[1]+"  "+currentRotation[2]);
			motorArray[1].setSpeed(200);
			motorArray[2].setSpeed(200);
			motorArray[1].setAcceleration(200);
			motorArray[2].setAcceleration(200);
			motorArray[0].rotateTo(currentRotation[0],true);
			motorArray[1].rotateTo(currentRotation[1],true);
			motorArray[2].rotateTo(currentRotation[2],true);
			Delay.msDelay(delay);
			motorArray[0].stop(true);
			motorArray[1].stop(true);
			motorArray[2].stop(true);

			motorArray[1].setSpeed(400);
			motorArray[2].setSpeed(400);
			motorArray[1].setAcceleration(400);
			motorArray[2].setAcceleration(400);
			Delay.msDelay(500);
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	
	//old code
	
	
	
//	public void moveRight(int rotation, int delay) {
//		try {
//			motorArray[0].rotate(-rotation - rotation/3,true);
//			motorArray[1].rotate(-rotation - rotation/3,true);
//			motorArray[2].rotate(rotation,true);
//			Delay.msDelay(delay);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//			motorArray[3].stop(true);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//
//	public void turnLeft(int degree, int delay) {
//		try {
//			motorArray[0].rotate(degree,true);
//			motorArray[1].rotate(degree,true);
//			motorArray[2].rotate(degree,true);
//			Delay.msDelay(delay);
//			
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//	
//	public void turnRight(int degree, int delay) {
//		try {
//			motorArray[0].rotate(-degree,true);
//			motorArray[1].rotate(-degree,true);
//			motorArray[2].rotate(-degree,true);
//			Delay.msDelay(delay);
//			
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//		} catch(Exception e) {
//			e.printStackTrace();
//		}
//	}
//	
//	
//	public void rotateFrontRight(int degree, int delay) {
//		try {
//			motorArray[0].rotate(-degree,true);
//			motorArray[1].rotate(-degree,true);
//			motorArray[2].rotate(-degree,true);
//			Delay.msDelay(delay);
//			
//			motorArray[0].stop(true);
//			motorArray[1].stop(true);
//			motorArray[2].stop(true);
//		} catch(Exception e) {
//			e.printStackTrace();
//			closeAllMotor();
//		}
//	}
//	
	
	
	
	public void setMotorSpeed(int motor, int speed) {
		motorArray[motor].setSpeed(speed);
	}
	
	public void setAllMotorsSpeed(int speed) {
		for(RegulatedMotor motor : motorArray) {
			motor.setSpeed(speed);
		}
	}
	
	public void setMotorAcceleration(int motor, int acceleration) {
		motorArray[motor].setAcceleration(acceleration);
	}
	
	public void setAllMotorsAccel(int acceleration) {
		for(RegulatedMotor motor : motorArray) {
			motor.setAcceleration(acceleration);
		}
	}
	
	public void stopMotors(){
		Motor.A.stop();
		Motor.B.stop();
		Motor.C.stop();
		Motor.D.stop();
	}
	
	
	public void closeAllMotor() {
		for (RegulatedMotor motor : motorArray) {
			motor.close();
		}
	}
	
	//Motor Scan Methods
//	public void moveForwardScan(int rotation, int delay) {
//		try {
//			currentRotation += 15;
//			
//			motorArray[2].rotateTo(-currentRotation,true);
//			motorArray[0].rotateTo(currentRotation,true);
//			Delay.msDelay(delay);
//		
//			motorArray[2].stop(true);
//			motorArray[0].stop(true);
//		} catch(Exception e){
//			e.printStackTrace();
//		}
//	}
	//================== End of Motor Methods ==================
	
	//================== Start of Sensor Methods ==================
	public int getReadingRight() {
		int reading = colorSensor1.getColorID();
		return reading;
	}
	public int getReadingMid() {
		int reading = colorSensor2.getColorID();
		return reading;
	}
	public int getReadingLeft() {
		int reading = colorSensor3.getColorID();
		return reading;
	}
	
	public void closeAllColorSensors() {
		colorSensor1.close();
		colorSensor2.close();
		colorSensor3.close();
	}
	//================== End of Motor Methods ==================
}
