package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

public class FrskyController extends Joystick {
	private static final int LEFT_HORIZ_AXIS = 3;
	private static final int LEFT_VERT_AXIS = 0;
	private static final int RIGHT_HORIZ_AXIS = 1;
	private static final int RIGHT_VERT_AXIS = 2;
	private static final int TOP_LEFT_SWITCH_AXIS = 5;
    private static final int TOP_RIGHT_SWITCH_AXIS = 4;
	
	public FrskyController(int port) {
		super(port);
	}

	public Button topLeftSwitch = new Button(() -> getRawAxis(TOP_LEFT_SWITCH_AXIS) > 0.5);	
	public Button topRightSwitch = new Button(() -> getRawAxis(TOP_RIGHT_SWITCH_AXIS) > 0.5);

	public boolean isTopLeftSwitchOn() {
		return topLeftSwitch.get();
	}
	
	public boolean isTopRightSwitchOn() {
		return topRightSwitch.get();
	}
	
	public double getRightStickY() {
		return this.getRawAxis(RIGHT_VERT_AXIS);
	}

	public double getRightStickX() {
		return this.getRawAxis(RIGHT_HORIZ_AXIS);
	}

	public double getLeftStickY() {
		return this.getRawAxis(LEFT_VERT_AXIS);
	}

	public double getLeftStickX() {
		return this.getRawAxis(LEFT_HORIZ_AXIS);
	}
}
