package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.ThisRobotTable;

public class ButtonBox extends Joystick {

    public enum ClimbBar {
        LOW,
        MID,
        HIGH,
        TRAVERSAL
    }
    public enum ClimbDirection {
        FORWARDS,
        BACKWARDS
    }
    public enum ClimbAction {
        PREP,
        RAISE,
        CLIMB
    }
	
    private static int INTAKE = 2;
    private static int OUTGEST = 3;
    private static int AUX_1 = 5;
    private static int AUX_2 = 4;
    private static int CENTRALIZER_UP = 6;
    private static int CENTRALIZER_DOWN = 7;
    private static int CHAMBER_UP = 8;
    private static int CHAMBER_DOWN = 9;
    private static int CLIMBER_STOW = 10;
    private static int CLIMBER_PREP = 11;
    private static int CLIMBER_RAISE = 12;
    private static int CLIMBER_CLIMB = 13;
    private static int SHOOT = 14;
    private static int TURRET_TRACK = 15;
    private static int CLIMBER_DIRECTION = 16;
    private static int AUX_3 = 17;
    private static int AUX_4 = 18;
    private static int MID_BAR = 20;
    private static int HIGH_BAR = 21;
    private static int TRAVERSAL_BAR = 19;

	public ButtonBox(int port) {
		super(port);
	}

	public Button intakeButton = new JoystickButton(this, INTAKE);
    public Button outgestButton = new JoystickButton(this, OUTGEST);
    public Button centralizerUp = new JoystickButton(this, CENTRALIZER_UP);
    public Button centralizerDown = new JoystickButton(this, CENTRALIZER_DOWN);
    public Button chamberUp = new JoystickButton(this, CHAMBER_UP);
    public Button chamberDown = new JoystickButton(this, CHAMBER_DOWN);
    public Button stowClimberButton = new JoystickButton(this, CLIMBER_STOW);
    public Button prepClimberButton = new JoystickButton(this, CLIMBER_PREP);
    public Button raiseClimberButton = new JoystickButton(this, CLIMBER_RAISE);
    public Button climbButton = new JoystickButton(this, CLIMBER_CLIMB);
    public Button shootButton = new JoystickButton(this, SHOOT);
    public Button turretTrackSwitch = new JoystickButton(this, TURRET_TRACK);
    public Button climbDirectionSwitch = new JoystickButton(this, CLIMBER_DIRECTION);
    public Button traversalBarSwitch = new JoystickButton(this, TRAVERSAL_BAR);
    public Button highBarSwitch = new JoystickButton(this, HIGH_BAR);
    public Button midBarSwitch = new JoystickButton(this, MID_BAR);
    public Button lowBarSwitch = new Button(() -> !(traversalBarSwitch.get() || highBarSwitch.get() || midBarSwitch.get()));
    public Button rosDisableSwitch = new JoystickButton(this, AUX_4);
    public Button autoShootSwitch = new JoystickButton(this, AUX_3);
    public Button defaultTurretButton = new JoystickButton(this, AUX_1);
    public Button cancelClimb = new JoystickButton(this, AUX_2);

	public boolean isIntakeButtonPressed() {
		return intakeButton.get();
	}

    public boolean isOutgestButtonPressed() {
		return outgestButton.get();
	}

    public boolean isCentralizerUpButtonPressed() {
        return centralizerUp.get();
    }

    public boolean isCentralizerDownButtonPressed() {
        return centralizerDown.get();
    }

    public boolean isChamberUpButtonPressed() {
        return chamberUp.get();
    }

    public boolean isChamberDownButtonPressed() {
        return chamberDown.get();
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

    public boolean isShootButtonPressed() {
		return shootButton.get();
	}

    public boolean isTrackTurretSwitchOn() {
        return turretTrackSwitch.get();
    }

    public ClimbDirection getClimbDirection() {
		// return climbDirectionSwitch.get() ? ClimbDirection.FORWARDS : ClimbDirection.BACKWARDS;
        return ClimbDirection.BACKWARDS;
	}

    public ClimbBar getClimbBar() {
        if (traversalBarSwitch.get()) {
            return ClimbBar.TRAVERSAL;
        } else if  (highBarSwitch.get()) {
            return ClimbBar.HIGH;
        } else if (midBarSwitch.get()) {
            return ClimbBar.MID;
        } else {
            return ClimbBar.LOW;
        }
    }

    public boolean isROSDisableSwitchOn() {
        return rosDisableSwitch.get();
    }

    public boolean isAutoShootSwitchOn() {
        return autoShootSwitch.get();
    }

    public boolean isDefaultTurretButtonPressed() {
        return defaultTurretButton.get();
    }

    public boolean isCancelClimbPressed() {
        return cancelClimb.get();
    }
}
