package org.frc5687.rapidreact.util;

import org.frc5687.rapidreact.RobotMap;
import org.frc5687.rapidreact.util.RotarySwitch;

/** Created by Ben Bernard on 2/2/2018. */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _autoSwitch;
    private RotarySwitch _positionSwitch;
    private final double TOLERANCE = 0.2;
    private static MetricTracker _metric;

    public PositionChooser(OutliersContainer.IdentityMode identityMode) {
        _positionSwitch = new RotarySwitch(RobotMap.Analog.POSITION_SWITCH, TOLERANCE, 0.25, 0.5, 0.75, 1);
        // _positionSwitch = new RotarySwitch(RobotMap.Analog.POSITION_SWITCH,
        // Constants.RotarySwitch.TOLERANCE, .09, .17, .23, .31, .5, .59, .68, .75, .82,
        // .91, .96);
    }

    public AutoChooser(OutliersContainer.IdentityMode identityMode) {
        _autoSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH, TOLERANCE, 0.077, 0.154, 0.231, 0.308, 0.385, 0.462,
                0.538, 0.615, 0.693, 0.770, 0.847, 0.925);
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
            return Mode.StayPut;
        }
    }

    public Position getSelectedPosition() {
        int raw = _autoSwitch.get();
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
        LeftOneBall(0, "Left One-Ball Tarmac"), RightOneBall(1, "Right One-Ball Tarmac"), 
        LeftTwoBall(2, "Left Two-Ball Tarmac"), RightTwoBall(3, "Right Two-Ball Tarmac");

        private String _label;
        private int _value;

        Position(int value, String label) {
            _value = value;
            _label = label;
            _metric.put("Position", label);
        }

        public int getValue() {
            return _value;
        }

        public String getLabel() {
            return _label;
        }
    }
    public enum Mode {
        StayPut(0, "Stay Put"), ShootAndGo(1, "Shoot and Cross"), ShootAndNearTrench(2, "Shoot and Near Trench"),
        ShootAndFarTrench(3, "Shoot and Far Trench"), Generator2NearTrench(4, "Generator 2 and Near Trench"),
        Generator2FarTrench(5, "Generator 2 and Far Trench"), SnipeAndNearTrench(6, "Snipe and Near Trench"),
        StealTenBall(7, "Steal and Generator Balls");

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
            _metric.put("Auto Mode", label);
        }

        public int getValue() {
            return _value;
        }

        public String getLabel() {
            return _label;
        }
    }
}


