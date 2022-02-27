package org.frc5687.rapidreact.config;

/** Configuration for autonomous mode
 * 
 * @see Constants
 */
public class AutoConfig {
    public static final double DRIVETRAIN_POWER = 0.5;

    // TODO: replace this with AutoChooser once we have a switch

    /** Choose auto mode to run */ 
    public static enum Mode {
        ZERO_BALL, // just move out of tarmac
        ONE_BALL, // shoot one ball, then move out of tarmac
        TWO_BALL, // then get 2nd ball and shoot it
        THREE_BALL, // then get 3rd ball and shoot it
        FOUR_BALL, // then get 4th ball and shoot it
        FIVE_BALL // then get 5th ball and shoot it
    }
    public static final Mode AUTO_MODE = Mode.ZERO_BALL;

    /** Choose auto starting position
     * 
     * <p>one ball tarmac (left tarmac) has one of our balls next to it
     * 
     * <p>two ball tarmac (right tarmac) has two of our balls next to it
     */
    public static enum Position {
        OBT_LEFT, // one ball tarmac, left side
        OBT_RIGHT, // one ball tarmac, right side
        TBT_LEFT, // two ball tarmac, left side
        TBT_RIGHT // two ball tarmac, right side
    }
    public static final Position AUTO_POSITION = Position.TBT_RIGHT;

}
