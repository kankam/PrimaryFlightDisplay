package data;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferStrategy;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;


import display.Assets;
import display.Display;
public class Game implements Runnable{
	
	public static String fileName;
	public static SimpleDateFormat sdf = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss");
	public int pitch, roll, yaw, heading, VerticalSpeed_meter_min_int, VerticalSpeed_feet_min_int;
	public double pressure;
	public static double SeaLevelPressure;
	public float altitude_meter, VerticalSpeed_meter_min, altitude_feet, VerticalSpeed_feet_min,
	GPS_Lat, GPS_Lon, GPS_Speed;
	float VerticalSpeed_meter_min_1_10, VerticalSpeed_meter_min_10_10;
	int vs_counter;
	int logCounter;
	boolean meter_feet = false;
	long vs_lastTime = System.nanoTime();
	long vs_nowTime;
	float altitude_old;
	boolean GPS_State;
	
	public Display display;
	public int width, height;
	public String title;
	
	private boolean running = false;
	private Thread thread;
	
	private BufferStrategy bs;
	private Graphics g;
	
	private static Font sanSerifFont = new Font("SanSerif", Font.BOLD, 24);
	private static Font sanSerifFont2 = new Font("SanSerif", Font.BOLD, 12);
	private static Font sanSerifFont3 = new Font("SanSerif", Font.BOLD, 16);
	//private BufferedImage blocker,main,roll;
	
	public Game(String title, int width, int height) {
		this.width = width;
		this.height = height;
		this.title = title;
		
		
	}
	
	private void init() {
		display = new Display(title, width, height);
		Assets.init();
		//blocker = ImageLoader.loadImage("/textures/blocker.png");
		//main = ImageLoader.loadImage("/textures/PFD_main.png");
		//roll = ImageLoader.loadImage("/textures/PFD_roll.png");
	}

	private void tick() throws IOException {
		//pitch= 0;
		pitch=RXTX.SerialTest.RAW[1] * -1;
		//roll = 0;
		roll=RXTX.SerialTest.RAW[2];
		yaw=RXTX.SerialTest.RAW[0];
		//heading = 10;
		heading = RXTX.SerialTest.RAW[3];
		//altitude_meter = 10.25f;
		//float Raw_5 = RXTX.SerialTest.RAW[5];
		pressure = (double)RXTX.SerialTest.RAW[4];
		altitude_meter = (float) (44330 * (1 - Math.pow((pressure / SeaLevelPressure), (1/5.255))));
		altitude_meter =(float)( RXTX.SerialTest.RAW[4]/100f);
		altitude_feet = (float)( altitude_meter*3.28f);
		altitude_feet = (float) (Math.round(altitude_feet * 100.0) / 100.0);
		vs_nowTime=System.nanoTime();
		if(vs_nowTime>(vs_lastTime + 50000000)) {
			VerticalSpeed_meter_min_1_10 = (altitude_old - altitude_meter);
			altitude_old = altitude_meter;
			vs_lastTime = System.nanoTime();
		
		if(vs_counter<20) {
			VerticalSpeed_meter_min_10_10 = VerticalSpeed_meter_min_10_10 + VerticalSpeed_meter_min_1_10;
			vs_counter++;
		}
		else if(vs_counter==20) {
			VerticalSpeed_meter_min = VerticalSpeed_meter_min_10_10*6;
			VerticalSpeed_meter_min_10_10 =0;
			vs_counter=0;
			}
		}
		//VerticalSpeed_meter_min = (float) (Math.round(VerticalSpeed_meter_min * 100.0) / 100.0);
		VerticalSpeed_feet_min = VerticalSpeed_meter_min *3.28f;
		VerticalSpeed_meter_min_int = (int)VerticalSpeed_meter_min;
		VerticalSpeed_feet_min_int = (int)VerticalSpeed_feet_min;
		//VerticalSpeed_feet_min = (float) (Math.round(VerticalSpeed_feet_min * 100.0) / 100.0);
		//GPS
		if(RXTX.SerialTest.RAW[5]==1) {
		GPS_State = true;
		GPS_Lat = RXTX.SerialTest.RAW[6]/10000f;
		GPS_Lon = RXTX.SerialTest.RAW[7]/10000f;
		}
		else {GPS_State = false;}

		GPS_Speed = RXTX.SerialTest.RAW[8]/100f;
		logCounter++;
		if(logCounter>29) {
			Timestamp timestamp = new Timestamp(System.currentTimeMillis());
			String timestampString = sdf.format(timestamp);
			FileWriter fw = new FileWriter(fileName,true);
			fw.write(timestampString);
			fw.write("\t");
			fw.write("Pitch: ");
			fw.write(String.valueOf(pitch));
			fw.write("\t");
			fw.write("Roll: ");
			fw.write(String.valueOf(roll));
			fw.write("\t");
			fw.write("Yaw: ");
			fw.write(String.valueOf(yaw));
			fw.write("\t");
			fw.write("Altitude in feet: ");
			fw.write(String.valueOf(altitude_feet));
			fw.write("\t");
			fw.write("GPS_Lat: ");
			fw.write(String.valueOf(GPS_Lat));
			fw.write("\t");
			fw.write("GPS_Lon: ");
			fw.write(String.valueOf(GPS_Lon));
			fw.write("\t");
			fw.write("Speed: ");
			fw.write(String.valueOf(GPS_Speed));
			fw.write("\n");
			fw.flush();
			fw.close();
			logCounter = 0;
			}
	} 
	
	private void render() {
		bs = display.getCanvas().getBufferStrategy();
		if(bs == null) {
			display.getCanvas().createBufferStrategy(3);
			return;
		}
		g = bs.getDrawGraphics();
		//clear Screen
		g.clearRect(0, 0, width, height);
		//Draw Here
		
		//Draw Black Rectangle
		g.setColor(Color.BLACK);
		g.fillRect(0, 0, width, height);
		
		//Draw PFD_Main
		AffineTransformOp pfdMain_op = CalRotation.pfdMain(roll, pitch);
		int PFD_Y = (int)(-106 +  pitch * 5);
		g.drawImage(pfdMain_op.filter(Assets.pfdMain, null), 192, PFD_Y, null);

		
		//Draw Altitude Bar
		if(meter_feet) {
			int Alt_Y = (int) (-416 + altitude_meter);
			g.drawImage(Assets.altitude,781, Alt_Y, null);
		}
		else {
			int Alt_Y = (int) (-416 + altitude_feet);
			g.drawImage(Assets.altitude,781, Alt_Y, null);
		};   
		
		//Draw Speed Bar
		int Seed_Y = (int) (-416 + GPS_Speed);
		g.drawImage(Assets.altitude,143, Seed_Y, null);
		
	    //Draw Roll Pointer
	    AffineTransformOp roll_op = CalRotation.rollPointer(roll);
		g.drawImage(roll_op.filter(Assets.roll, null), 387, 130, null);
		
		//Draw Blocker
		g.drawImage(Assets.blocker,0, 0, null);
		

	    //Draw Heading Pointer
	    AffineTransformOp heading_op = CalRotation.headingPointer(heading);
		g.drawImage(heading_op.filter(Assets.heading, null), 304, 644, null);
		
		//Print Heading
		g.setColor(Color.WHITE);
		g.setFont(sanSerifFont);
	    FontMetrics hd = g.getFontMetrics();
	    String hd_t = Integer.toString(heading);
	    int hd_w = hd.stringWidth(hd_t);
	    int hd_h = hd.getAscent();
	    g.drawString(hd_t, 512 - (hd_w / 2), 735 + (hd_h / 4));
	    
	    //Draw Vertical Speed Pointer and print vertical speed
	    if(meter_feet) {
		    if (VerticalSpeed_meter_min > 1000) {
			    int VS_Y = 375 - 11 * 10;
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_meter_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
		    else if (VerticalSpeed_meter_min < -1000) {
			    int VS_Y = 375 + 11 * 10;
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_meter_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
		    
		    else{
			    int VS_Y = (int) (375 - VerticalSpeed_meter_min * 0.1);
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_meter_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
	    }
	    else {
	    	if (VerticalSpeed_feet_min > 1000) {
			    int VS_Y = 375 - 11 * 10;
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_feet_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
		    else if (VerticalSpeed_feet_min < -1000) {
			    int VS_Y = 375 + 11 * 10;
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_feet_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
		    
		    else{
			    int VS_Y = (int) (375 - VerticalSpeed_feet_min * 0.1);
				g.drawImage(Assets.v_speed, 915, VS_Y, null);
				//Print altitude
				g.setColor(Color.WHITE);
				g.setFont(sanSerifFont2);
			    FontMetrics VS = g.getFontMetrics();
			    String VS_t = Integer.toString(VerticalSpeed_feet_min_int);
			    int VS_w = VS.stringWidth(VS_t);
			    int VS_h = VS.getAscent();
			    g.drawString(VS_t, 950 - (VS_w / 2), VS_Y + 10 + (VS_h / 4));
		    }
	    }
		//Print altitude
		g.setColor(Color.WHITE);
		g.setFont(sanSerifFont);
	    FontMetrics alt = g.getFontMetrics();
	    if(meter_feet) {
		    String alt_t = Float.toString(altitude_meter);
		    int alt_w = alt.stringWidth(alt_t);
		    int alt_h = alt.getAscent();
		    g.drawString(alt_t, 848 - (alt_w / 2), 385 + (alt_h / 4));}
	    else {String alt_t = Float.toString(altitude_feet);
		    int alt_w = alt.stringWidth(alt_t);
		    int alt_h = alt.getAscent();
		    g.drawString(alt_t, 848 - (alt_w / 2), 385 + (alt_h / 4));
	    }
	    //Print Speed
	    g.setColor(Color.WHITE);
		g.setFont(sanSerifFont);
	    FontMetrics speed = g.getFontMetrics();
	    String speed_t = Float.toString(GPS_Speed);
	    int speed_w = speed.stringWidth(speed_t);
	    int speed_h = speed.getAscent();
	    g.drawString(speed_t, 176 - (speed_w / 2), 385 + (speed_h / 4));
	    
	    //print meter / feet
	    if(meter_feet) {
			g.setColor(Color.GREEN);
			g.setFont(sanSerifFont);
		    FontMetrics m = g.getFontMetrics();
		    String m_t = "m";
		    int m_w = m.stringWidth(m_t);
		    int m_h = m.getAscent();
		    g.drawString(m_t, 854 - (m_w / 2), 165 + (m_h / 4));
	    }
	    else {
			g.setColor(Color.GREEN);
			g.setFont(sanSerifFont);
		    FontMetrics m = g.getFontMetrics();
		    String m_t = "ft";
		    int m_w = m.stringWidth(m_t);
		    int m_h = m.getAscent();
		    g.drawString(m_t, 854 - (m_w / 2), 165 + (m_h / 4));
	    }
	    
	    //print Meter/min / Feet/min
	    if(meter_feet) {
			g.setColor(Color.GREEN);
			g.setFont(sanSerifFont);
		    FontMetrics m = g.getFontMetrics();
		    String m_t = "m/min";
		    int m_w = m.stringWidth(m_t);
		    int m_h = m.getAscent();
		    g.drawString(m_t, 920 - (m_w / 2), 270 + (m_h / 4));
	    }
	    else {
			g.setColor(Color.GREEN);
			g.setFont(sanSerifFont);
		    FontMetrics m = g.getFontMetrics();
		    String m_t = "ft/min";
		    int m_w = m.stringWidth(m_t);
		    int m_h = m.getAscent();
		    g.drawString(m_t, 920 - (m_w / 2), 270 + (m_h / 4));
	    }
	    
	    
	    //altimeter mode
	    g.setColor(Color.GREEN);
		g.setFont(sanSerifFont3);
	    FontMetrics alm = g.getFontMetrics();
	    String alm_t = "QNH";
	    int alm_w = alm.stringWidth(alm_t);
	    int alm_h = alm.getAscent();
	    g.drawString(alm_t, 806 - (alm_w / 2), 165 + (alm_h / 4));
	    
	    //Speed Mode
	    g.setColor(Color.GREEN);
		g.setFont(sanSerifFont);
	    FontMetrics spm = g.getFontMetrics();
	    String spm_t = "GS";
	    int spm_w = spm.stringWidth(spm_t);
	    int spm_h = spm.getAscent();
	    g.drawString(spm_t, 167 - (spm_w / 2), 165 + (spm_h / 4));
	    g.setFont(sanSerifFont3);
	    g.drawString("knots", 213 - (spm_w / 2), 165 + (spm_h / 4));
	    
	    //Display COM Port
	    g.setColor(Color.GREEN);
		g.setFont(sanSerifFont);
	    String[] PORT_Names = RXTX.SerialTest.PORT_NAMES;
	    String PORT = PORT_Names[0];
	    g.drawString(PORT, 20,20);
	    
	    //GPS
	    if(GPS_State) {
		    g.setColor(Color.GREEN);
			g.setFont(sanSerifFont);
		    String gps_t = "GPS: Online";
		    g.drawString(gps_t, 817, 655);
		    g.drawString("Lat: ", 817, 675);
		    String lat_t = Float.toString(GPS_Lat);
		    g.drawString(lat_t,870, 675);
		    g.drawString("Lon: ", 817, 695);
		    String lon_t = Float.toString(GPS_Lon);
		    g.drawString(lon_t,870, 695); 
	    }
	    else {
		    g.setColor(Color.RED);
			g.setFont(sanSerifFont);
		    String gps_t = "GPS: Offline";
		    g.drawString(gps_t, 817, 655);
		    g.drawString("Lat: ", 817, 675);
		    String lat_t = Float.toString(GPS_Lat);
		    g.drawString(lat_t,870, 675);
		    g.drawString("Lon: ", 817, 695);
		    String lon_t = Float.toString(GPS_Lon);
		    g.drawString(lon_t,870, 695);
		    
		    }
    
	    
		//End Drawing
		bs.show();
		g.dispose();
	}
	
	public void run() {
		
		init();
		
		int fps = 30;
		double timePerTick = 1000000000 / fps;
		double delta = 0;
		long now;
		long lastTime = System.nanoTime();
		long timer = 0;
		int ticks = 0;
		
		while(running) {
			now = System.nanoTime();
			delta += (now - lastTime)/ timePerTick;
			timer += now - lastTime;
			lastTime = now;
			
			if(delta >= 1) {
			try {
				tick();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			render();
			ticks++;
			delta--;
			}
			
			if(timer >= 1000000000) {
				System.out.println("Ticks and Frames: " + ticks);
				ticks = 0;
				timer = 0;
			}
		}
		
		stop();
	}
	
	public synchronized void start() {
		if(running)
			return;
		running = true;
		thread = new Thread(this);
		thread.start();
	}
	
	public synchronized void stop() {
		if(!running)
			return;
		running = false;
		try {
			thread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

}
