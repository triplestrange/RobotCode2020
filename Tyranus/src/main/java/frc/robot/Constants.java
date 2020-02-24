/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	// control panel constants
	public static final class ControlPanel {
		// control panel motor constants
		public static final class Motor {
			// control panel motor CAN ID
			public static final int bus_id = 1;
			// control panel motor speed
			public static final double speed = 1.0;
		}
	
		public static final Map<Character, String> ColorMap = new HashMap<Character, String>();
		static {
			ColorMap.putIfAbsent('R', "blue");
			ColorMap.putIfAbsent('G', "yellow");
			ColorMap.putIfAbsent('B', "red");
			ColorMap.putIfAbsent('Y', "green");
		}
	
		// control panel colors (RGB values for sensor)
		public static final class Colors {
			public static final class Blue {
				public static final double r = 0.149;
				public static final double g = 0.417;
				public static final double b = 0.435;
			}
	
			public static final class Green {
				public static final double r = 0.196;
				public static final double g = 0.595;
				public static final double b = 0.210;
			}
	
			public static final class Red {
				public static final double r = 0.544;
				public static final double g = 0.337;
				public static final double b = 0.119;
			}
	
			public static final class Yellow {
				public static final double r = 0.324;
				public static final double g = 0.572;
				public static final double b = 0.105;
			}
		}
	}

}
