package org.frc5687.rapidreact.util;

import org.frc5687.rapidreact.config.RobotMap;

/** Created by Ben Bernard on 2/2/2018. */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _autoSwitch;
    private RotarySwitch _positionSwitch;
    private final double TOLERANCE = 0.05;

    public AutoChooser() {
        _autoSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH, TOLERANCE, 0.07, 0.14, 0.23, 0.3, 0.38, 0.46,
                0.54, 0.62, 0.69, 0.77, 0.85, 0.93);        
        _positionSwitch = new RotarySwitch(RobotMap.Analog.POSITION_SWITCH, TOLERANCE, 0.07, 0.15, 0.21, 0.3, 0.38, 0.46,
                0.54, 0.61, 0.69, 0.76, 0.83, 0.93);


        // _autoSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH,
        // Constants.RotarySwitch.TOLERANCE, .09, .17, .23, .31, .5, .59, .68, .75, .82,
        // .91, .96);
    }

    public Mode getSelectedMode() {
        int raw = _autoSwitch.get();
        if (raw >= Mode.values().length) {
            raw = 0;
        }
        try {
            return Mode.values()[raw];
        } catch (Exception e) {
            return Mode.ZeroBall;
        }
    }

    public Position getSelectedPosition() {
        int raw = _positionSwitch.get();
        if (raw >= Position.values().length) {
            raw = 0;
        }
        try {
            return Position.values()[raw];
        } catch (Exception e) {
            return Position.LeftOneBall;
        }
    }

    public void updateDashboard() {
        metric("Label/Mode", getSelectedMode().getLabel());
        metric("Raw/Mode", _autoSwitch.getRaw());
        metric("Numeric/Mode", _autoSwitch.get());
        metric("Label/Position", getSelectedPosition().getLabel());
        metric("Raw/Position", _positionSwitch.getRaw());
        metric("Numeric/Position", _positionSwitch.get());
    }

    public enum Position {
        LeftOneBall(0, "Left1ballT"), RightOneBall(1, "Right1ballT"), 
        LeftTwoBall(2, "Left2ballT"), RightTwoBall(3, "Right2ballT");

        private String _label;
        private int _value;

        Position(int value, String label) {
            _value = value;
            _label = label;
//            _metric.put("Position", label);
        }

        public int getValue() {
            return _value;
        }

        public String getLabel() {
            return _label;
        }
    }
    public enum Mode {
        ZeroBall(0, "ZB"), OneBall(1, "ZB");

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
//            _metric.put("Auto Mode", label);
        }

        public int getValue() {
            return _value;
        }

        public String getLabel() {
            return _label;
        }
    }

    public String getPath(Mode mode, Position position) {
        String pathMode = mode.getLabel();
        String pathPosition = position.getLabel();
        return "output/" + pathMode + pathPosition + ".wpilib.json";
    }
}


