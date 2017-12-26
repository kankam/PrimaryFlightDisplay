package data;

import java.util.Scanner;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;


public class Launcher {
	public static void main(String[] args) throws IOException {
		Scanner scanner = new Scanner(System.in);
		String Port;
		System.out.println("Enter COM Port:");
		Port=(scanner.next());
		System.out.println(Port);
		RXTX.SerialTest.PORT_NAMES[0]= Port;
		String SeaLevelPressure;
		System.out.println("Enter Sea Level Pressure:");
		SeaLevelPressure=(scanner.next());
		System.out.println(SeaLevelPressure);
		data.Game.SeaLevelPressure= Double.parseDouble(SeaLevelPressure);
		scanner.close();
		
		Timestamp timestamp = new Timestamp(System.currentTimeMillis());
		String timestampString = data.Game.sdf.format(timestamp);
		data.Game.fileName = timestampString + ".txt"; 
		FileWriter fw = new FileWriter(data.Game.fileName);		
		fw.write(timestampString);
		fw.write("\n");
		fw.flush();
		fw.close();

		Game game = new Game("PFD", 1024, 768);
		try {
			RXTX.SerialTest.main(args);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		game.start();
	}

}
