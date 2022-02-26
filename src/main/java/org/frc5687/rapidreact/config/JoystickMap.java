package org.frc5687.rapidreact.config;

import edu.wpi.first.wpilibj.Joystick.AxisType;

/** Joystick and gamepad USB and button mapping
 * 
 * <p>Take button and controller mapping out of the OI class, making it easier to
 * change button assignment without breaking the OI code.
 * 
 * <p>The class is broken into three parts:
 * 
 * <ol>
 *  <li> The Controllers class holds the USB port assignments for each controller.  Note that if a controller is to be unplugged
 *  you should set the USB port to -1.  For the most part this will cause OI to ignore it.
 * 
 *  <li> The Axes class has a subclass for each axis (or group of axes) in use.  Each axis subclass should have a constant defining
 *   which controller it's mapped to and a constant for each axis in the group.
 * 
 *  <li> The Buttons class has a subclass for each button in use.  Each button subclass should have a constant defining
 *   which controller it's mapped to and a constant for the button number itself.
 * </ol>
 */
public class JoystickMap {

    /** Joysticks and gamepads mapped to USB ports */
    public static class Controllers {
        /** 
         * USB ports of joysticks or gamepads.
         * -1 means not in use.
        */
        public static final int TRANSLATOR_JOYSTICK = 0;
        public static final int ROTATOR_JOYSTICK = -1;
        public static final int ROTATOR_GAMEPAD = 1;
    }

    /** Define axes of control */
    public static class Axes {
        /** Translation is movement in X and Y directions. */
        public static class Translation {
            public static int Controller = Controllers.TRANSLATOR_JOYSTICK;
            public static int X = AxisType.kX.value;
            public static int Y = AxisType.kY.value;
        }

        /** Rotation is angular movement around center of robot. */
        public static class Rotation {

            /**
             * To move the Rotation control from joystick to a gamepad:
             * 
             * 1. Change the ROTATOR_GAMEPAD constant above to a valid USB port
             * 2. Change Rotation.Controller to Controllers.ROTATOR_GAMEPAD.
             * 3. Optional: You could also change Rotation.Twist to a different axis number if needed.
             *
             *  See commented code below.
             */

            // Joystick control of rotation
            //public static int Controller = Controllers.ROTATOR_JOYSTICK;
            //public static int Twist = AxisType.kX.value;

            // Gamepad control of rotation
            public static int Controller = Controllers.ROTATOR_GAMEPAD;
            public static int Twist = AxisType.kX.value;
        }
    }

    public static class Buttons {
        public static class SHOOT {
            public static int Controller = Controllers.TRANSLATOR_JOYSTICK;
            public static int Button = 0;
        }

        public static class RESET_NAVX {
            public static int Controller = Controllers.ROTATOR_GAMEPAD;
            public static int Button = 5;
        }

        public static class RUN_AUTO {
            public static int Controller = Controllers.TRANSLATOR_JOYSTICK;
            public static int Button = 1;
        }
    }
}