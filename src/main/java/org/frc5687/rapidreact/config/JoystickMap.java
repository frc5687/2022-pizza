package org.frc5687.rapidreact.config;

import edu.wpi.first.wpilibj.Joystick.AxisType;

import org.frc5687.rapidreact.OI;

/** Joystick and gamepad USB and button mapping
 * 
 * <p>Take button and controller mapping out of the OI class, making it easier to
 * change button assignment without breaking the OI code.
 * 
 * @see OI
 */
public class JoystickMap {

    public static final int NOT_IN_USE = -1; // flag for joysticks or gamepads not in use

    // Step 1. Set USB ports of joysticks and gamepads to use

    // Joysticks
    public static final int JOYSTICK_TRANSLATION_USB = 0;
    public static final int JOYSTICK_ROTATION_USB = 1;
    public static final int JOYSTICK_DEBUG_USB = NOT_IN_USE;

    // Gamepads
    public static final int GAMEPAD_TRANSLATION_USB = NOT_IN_USE;
    public static final int GAMEPAD_ROTATION_USB = NOT_IN_USE;
    public static final int GAMEPAD_DEBUG_USB = 2;

    // Step 2. Configure joysticks and map commands to buttons

    /** Button mappings for the translation joystick */
    public static class JOYSTICK_TRANSLATION {

        public static final String MODEL = "Logitech Attack 3";
        public static final int X = AxisType.kX.value;
        public static final int Y = AxisType.kY.value;
        public static final int BUTTON_COUNT = 11; // how many buttons on it

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.CATAPULT_SHOOT, // 1 trigger
            OI.Command.NOT_IN_USE, // 2 thumb lower
            OI.Command.NOT_IN_USE, // 3 thumb upper
            OI.Command.NOT_IN_USE, // 4 side stick left
            OI.Command.NOT_IN_USE, // 5 side stick right
            OI.Command.NOT_IN_USE, // 6 left pad left
            OI.Command.NOT_IN_USE, // 7 left pad right
            OI.Command.NOT_IN_USE, // 8 mid pad left
            OI.Command.NOT_IN_USE, // 9 mid pad right
            OI.Command.NOT_IN_USE, // 10 right pad left
            OI.Command.NOT_IN_USE // 11 right pad right
        };

    }

    /** Button mappings for the rotation joystick */
    public static class JOYSTICK_ROTATION {

        public static final String MODEL = "Logitech Attack 3";
        public static int Twist = AxisType.kX.value;
        public static final int BUTTON_COUNT = 11; // how many buttons on it

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.INTAKE_DEPLOY, // 1 trigger
            OI.Command.NOT_IN_USE, // 2 thumb lower
            OI.Command.NOT_IN_USE, // 3 thumb upper
            OI.Command.NOT_IN_USE, // 4 side stick left
            OI.Command.NOT_IN_USE, // 5 side stick right
            OI.Command.NOT_IN_USE, // 6 left pad left
            OI.Command.NOT_IN_USE, // 7 left pad right
            OI.Command.NOT_IN_USE, // 8 mid pad left
            OI.Command.NOT_IN_USE, // 9 mid pad right
            OI.Command.NOT_IN_USE, // 10 right pad left
            OI.Command.NOT_IN_USE // 11 right pad right
        };

    }

    /** Button mappings for the debug joystick */
    public static class JOYSTICK_DEBUG {

        public static final String MODEL = "Extreme 3D Pro";
        public static final int BUTTON_COUNT = 11; // how many buttons on it

        // TODO: characterize Extreme 3D Pro joystick

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.CATAPULT_SHOOT, // 1 trigger
            OI.Command.NOT_IN_USE, // 2 thumb lower
            OI.Command.NOT_IN_USE, // 3 thumb upper
            OI.Command.NOT_IN_USE, // 4 side stick left
            OI.Command.NOT_IN_USE, // 5 side stick right
            OI.Command.NOT_IN_USE, // 6 left pad left
            OI.Command.NOT_IN_USE, // 7 left pad right
            OI.Command.NOT_IN_USE, // 8 mid pad left
            OI.Command.NOT_IN_USE, // 9 mid pad right
            OI.Command.NOT_IN_USE, // 10 right pad left
            OI.Command.NOT_IN_USE // 11 right pad right
        };

    }

    /** Button mappings for the translation gamepad */
    public static class GAMEPAD_TRANSLATION {

        public static final String MODEL = "GamerSir";
        public static final int X = AxisType.kX.value;
        public static final int Y = AxisType.kY.value;
        public static final int BUTTON_COUNT = 8; // how many buttons on it

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.CATAPULT_SHOOT, // 1 A button
            OI.Command.NOT_IN_USE, // 2 B button
            OI.Command.NOT_IN_USE, // 3 X button
            OI.Command.NOT_IN_USE, // 4 Y button
            OI.Command.NOT_IN_USE, // 5 L1
            OI.Command.NOT_IN_USE, // 6 R1
            OI.Command.NOT_IN_USE, // 7 Select
            OI.Command.NOT_IN_USE // 8 Start
        };

    }

    /** Button mappings for the rotation gamepad */
    public static class GAMEPAD_ROTATION {

        public static final String MODEL = "GamerSir";
        public static int Twist = AxisType.kX.value;
        public static final int BUTTON_COUNT = 8; // how many buttons on it

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.CATAPULT_SHOOT, // 1 A button
            OI.Command.NOT_IN_USE, // 2 B button
            OI.Command.NOT_IN_USE, // 3 X button
            OI.Command.NOT_IN_USE, // 4 Y button
            OI.Command.NOT_IN_USE, // 5 L1
            OI.Command.NOT_IN_USE, // 6 R1
            OI.Command.NOT_IN_USE, // 7 Select
            OI.Command.NOT_IN_USE // 8 Start
        };

    }

    /** Button mappings for the debug gamepad */
    public static class GAMEPAD_DEBUG {

        public static final String MODEL = "GamerSir";
        public static final int BUTTON_COUNT = 8; // how many buttons on it

        // Map buttons to commands
        public static final OI.Command[] BUTTONS = {
            OI.Command.CATAPULT_SHOOT, // 1 A button
            OI.Command.NOT_IN_USE, // 2 B button
            OI.Command.NOT_IN_USE, // 3 X button
            OI.Command.NOT_IN_USE, // 4 Y button
            OI.Command.NOT_IN_USE, // 5 L1
            OI.Command.NOT_IN_USE, // 6 R1
            OI.Command.NOT_IN_USE, // 7 Select
            OI.Command.NOT_IN_USE // 8 Start
        };

    }

}
