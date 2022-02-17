/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;

/**
 * We use compass headings to reference positions on the chassis.
 * 
 * When looking down at top of robot:
 * 
 *          N
 *          |
 *      W -- -- E
 *          |
 *          S
 * 
 * When robot is flipped over on its back:
 * 
 *          N
 *          |
 *      E -- -- W
 *          |
 *          S
 */

/** 
 * Constants for things plugged in to the roboRIO.
 */
public class RobotMap {

    /**
     * Create an entry for each CAN device, preferrably grouped by device type and
     * then in numerical order. Note that for CAN, ids must be unique per device type, but not
     * across types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a
     * SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {
            public static class TALONFX {
            public static final int NORTH_WEST_INNER = 1;
            public static final int NORTH_WEST_OUTER = 2;
            public static final int SOUTH_WEST_OUTER = 4;
            public static final int SOUTH_WEST_INNER = 3;
            public static final int SOUTH_EAST_INNER = 6;
            public static final int SOUTH_EAST_OUTER = 5;
            public static final int NORHT_EAST_OUTER = 8;
            public static final int NORTH_EAST_INNER = 7;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that
     * for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {}

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that
     * for PCM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PCM {}

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order. Note that
     * only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {}

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order. Note that
     * for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {
        public static final int MODE_SWITCH = 0;
        public static final int POSITION_SWITCH = 1;
    }
    /**
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that
     * for DIO only one device can connect to each port, so the numbers should be unique.
     * 
     * These are the encoders for our swerve modules.
     * 
     * Note: the order of the encoders is wack-a-doodle:
     * 
     *  NW (3)  NE (4)
     *  
     *  SW (2)  SE (1)
     * 
     */
    public static class DIO {
        public static final int SOUTH_EAST = 1;
        public static final int SOUTH_WEST = 2;
        public static final int NORTH_WEST = 3;
        public static final int NORTH_EAST = 4;
    }
}
