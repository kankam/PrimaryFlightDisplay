package display;

import java.awt.image.BufferedImage;

import data.ImageLoader;

public class Assets {
	public static BufferedImage blocker, pfdMain, roll, heading, altitude, v_speed;
	
	public static void init() {
		blocker = ImageLoader.loadImage("/textures/PFD_Blocker_1024x768.png");
		//pfdMain = ImageLoader.loadImage("/textures/PFD_main.png");
		pfdMain = ImageLoader.loadImage("/textures/PFD_main_640x980.png");
		roll = ImageLoader.loadImage("/textures/PFD_roll.png");
		heading = ImageLoader.loadImage("/textures/PFD_Heading_416x416.png");
		altitude =  ImageLoader.loadImage("/textures/PFD_Alt_100x1000.png");
		v_speed = ImageLoader.loadImage("/textures/PFD_VSI_54x18.png");
		
	}
	
}
