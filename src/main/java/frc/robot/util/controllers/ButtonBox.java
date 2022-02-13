package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBox extends Joystick {
	
    private static int INTAKE = 1;
    private static int OUTGEST = 2;
    private static int SHOOT = 3;
    private static int CLIMB_DIRECTION = 4;
    private static int LOW_BAR = 5;
    private static int MID_BAR = 6;
    private static int HIGH_BAR = 7;
    private static int TRAVERSAL_BAR = 8;
    private static int STOW_CLIMBER = 9;
    private static int PREP_CLIMBER = 10;
    private static int RAISE_CLIMBER = 11;
    private static int CLIMB = 12;
	
	public ButtonBox(int port) {
		super(port);
	}

	public Button intakeButton = new JoystickButton(this, INTAKE);
    public Button outgestButton = new JoystickButton(this, OUTGEST);
    public Button shootButton = new JoystickButton(this, SHOOT);
    public Button climbDirectionSwitch = new JoystickButton(this, CLIMB_DIRECTION);
    public Button lowBarButton = new JoystickButton(this, LOW_BAR);
    public Button midBarButton = new JoystickButton(this, MID_BAR);
    public Button highBarButton = new JoystickButton(this, HIGH_BAR);
    public Button traversalBarButton = new JoystickButton(this, TRAVERSAL_BAR);
    public Button stowClimberButton = new JoystickButton(this, STOW_CLIMBER);
    public Button prepClimberButton = new JoystickButton(this, PREP_CLIMBER);
    public Button raiseClimberButton = new JoystickButton(this, RAISE_CLIMBER);
    public Button climbButton = new JoystickButton(this, CLIMB);

	public boolean isIntakeButtonPressed() {
		return intakeButton.get();
	}

    public boolean isOutgestButtonPressed() {
		return outgestButton.get();
	}

    public boolean isShootButtonPressed() {
		return shootButton.get();
	}

    public boolean isClimbDirectionSwitchOn() {
		return climbDirectionSwitch.get();
	}

    public boolean isLowBarButtonPressed() {
		return lowBarButton.get();
	}

    public boolean isMidBarButtonPressed() {
		return midBarButton.get();
	}

    public boolean isHighBarButtonPressed() {
		return highBarButton.get();
	}

    public boolean isTraversalBarButtonPressed() {
		return traversalBarButton.get();
	}

    public boolean isStowClimberButtonPressed() {
		return stowClimberButton.get();
	}

    public boolean isPrepClimberButtonPressed() {
		return prepClimberButton.get();
	}

    public boolean isRaiseClimberButtonPressed() {
		return raiseClimberButton.get();
	}

    public boolean isClimbButtonPressed() {
		return climbButton.get();
	}
}
