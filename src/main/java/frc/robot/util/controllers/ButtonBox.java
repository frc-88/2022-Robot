package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
	
    private static int INTAKE = 10;
    private static int OUTGEST = 12;
    private static int SHOOT = 2;
    private static int CLIMB_DIRECTION = 14;
    private static int LOW_BAR = 5;
    private static int MID_BAR = 6;
    private static int HIGH_BAR = 7;
    private static int TRAVERSAL_BAR = 8;
    private static int STOW_CLIMBER = 9;
    private static int PREP_CLIMBER = 10;
    private static int RAISE_CLIMBER = 11;
    private static int CLIMB = 12;

    private ClimbBar m_currentClimbBar;
	
	public ButtonBox(int port) {
		super(port);
	}

	public Button intakeButton = new JoystickButton(this, INTAKE);
    public Button outgestButton = new JoystickButton(this, OUTGEST);
    public Button shootButton = new JoystickButton(this, SHOOT);
    public Button stowClimberButton = new JoystickButton(this, STOW_CLIMBER);
    public Button prepClimberButton = new JoystickButton(this, PREP_CLIMBER);
    public Button raiseClimberButton = new JoystickButton(this, RAISE_CLIMBER);
    public Button climbButton = new JoystickButton(this, CLIMB);
    public Button climbDirectionChange = new Button() {
        private ClimbDirection lastState = ClimbDirection.FORWARDS;
        @Override
        public boolean get() {
            ClimbDirection currentState = getClimbDirection();
            boolean ret = currentState == lastState;
            lastState = currentState;
            return ret;
        }
    };
    public Button climbBarChange = new Button() {
        private ClimbBar lastState = ClimbBar.TRAVERSAL;
        @Override
        public boolean get() {
            ClimbBar currentState = getClimbBar();
            boolean ret = currentState == lastState;
            lastState = currentState;
            return ret;
        }
    };

	public boolean isIntakeButtonPressed() {
		return intakeButton.get();
	}

    public boolean isOutgestButtonPressed() {
		return outgestButton.get();
	}

    public boolean isShootButtonPressed() {
		return shootButton.get();
	}

    public ClimbDirection getClimbDirection() {
		return getRawButton(CLIMB_DIRECTION) ? ClimbDirection.FORWARDS : ClimbDirection.BACKWARDS;
	}

    public ClimbBar getClimbBar() {
        if (getRawButton(TRAVERSAL_BAR)) {
            m_currentClimbBar = ClimbBar.TRAVERSAL;
        } else if (getRawButton(HIGH_BAR)) {
            m_currentClimbBar = ClimbBar.HIGH;
        } else if (getRawButton(MID_BAR)) {
            m_currentClimbBar = ClimbBar.MID;
        } else if (getRawButton(LOW_BAR)) {
            m_currentClimbBar = ClimbBar.LOW;
        }

        return m_currentClimbBar;
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
