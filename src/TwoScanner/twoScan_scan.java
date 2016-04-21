package TwoScanner;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import lejos.hardware.Sound;
import lejos.hardware.device.NXTCam;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

public class twoScan_scan {

	private boolean scanFinish = false;

	private twoScan_EV3 ev3;

	private int correction = 2;
	private int previousNumLeft = 0;
	private int previousNumRight = 0;

	public twoScan_scan(twoScan_EV3 robot, int currentSide) throws IOException {
		ev3 = robot;
		initializeField();
	}

	public void initializeField() throws IOException {
		File file= new File ("map.txt");
		file.createNewFile();
		FileWriter writer= new FileWriter(file);
		// ev3.moveForward(100, 2000);
		// ev3.moveBackward(100, 2000);
		// ev3.moveRight(200, 2000);
		// ev3.moveLeft(200, 2000);
		int mapArray[][]=new int[12][37];
		
//		for(int y=0; y<28;y++){
//			ev3.moveForward(40, 400);
//		}
//		ev3.delay(500);
//		for(int y=0; y<28;y++){
//			ev3.moveBackward(40, 400);
//		}
		
//		ev3.moveLeft(150, 10000);
//Test length
//		int speed = 100;
//		int x=0;
//		while (!ev3.isEscDown()) {
//			ev3.moveForward(50, 1000);
//			x++;
//		}
//		System.out.println(x);//22
//
//		turnRight90();
//		ev3.moveForward(50, 2100);
//		turnLeft90();
//		turnRight90();
//		ev3.moveForward(130, 2100);
//		turnLeft90();
//		ev3.delay(5000);
		
		
		

		int x=0;
		boolean term=false;
		while (!ev3.isEscDown()) {
			
		// The 90degree turn and move for change line to scan
			if(term)
			{
				for(int y=0; y<37;y++){
					mapArray[3*x][36-y]=ev3.getReadingLeft();
					mapArray[3*x+1][36-y]=ev3.getReadingMid();
					mapArray[3*x+2][36-y]=ev3.getReadingRight();
					ev3.moveBackward(30, 400);
					}
					
				ev3.moveRight(150, 2000);
				term=false;
				x++;
				ev3.delay(200);
			}else{

				for(int y=0; y<37;y++){
					ev3.moveForward(30, 400);
					mapArray[3*x][y]=ev3.getReadingLeft();
					mapArray[3*x+1][y]=ev3.getReadingMid();
					mapArray[3*x+2][y]=ev3.getReadingRight();
					
				}
				ev3.moveRight(150, 2000);
				term=true;
				x++;
				ev3.delay(200);
			}
			if(x==4) {
				break;
			}
			
			
		}
		
		String map="";
		for(int countX=0; countX<12; countX++)
		{
			for(int countY=0; countY<37; countY++)
			{
				
				if(mapArray[countX][countY]==0)
				{
					map+="$ ";
				}else if(mapArray[countX][countY]==13||mapArray[countX][countY]==7||mapArray[countX][countY]==2)
				{
					map+="= ";
				}else if(mapArray[countX][countY]==6)
				{
					map+="/ ";
				}else{
					map+="? ";
				}
				
				
			}
			map+="\n";
		}
				
		writer.write(map);
		
		writer.flush();
		writer.close();

	}
	


	public void turnRight90() {

		ev3.moveRight(210, 2100);
	}

	public void turnLeft90() {
		ev3.moveLeft(210, 2100);
	}

	public boolean isScanFinished() {
		return scanFinish;
	}
}
