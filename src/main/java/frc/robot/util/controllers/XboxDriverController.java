package frc.robot.util.controllers;

import frc.robot.Constants;
import frc.robot.util.drive.DriveUtils;

public class XboxDriverController extends XboxController implements DriverController{

    public XboxDriverController(int port) {
        super(port);
    }

    @Override
    public double getThrottle() {
        return DriveUtils.deadbandExponential(getLeftStickY(), Constants.DRIVE_SPEED_EXP_XBOX, Constants.XBOX_DEADBAND);
    }

    @Override
    public double getTurn() {
        double cheesyDriveMinTurn = getForceLowGear() ? Constants.CHEESY_DRIVE_FORCE_LOW_MIN_TURN : Constants.CHEESY_DRIVE_MIN_TURN;
        double cheesyDriveMaxTurn = getForceLowGear() ? Constants.CHEESY_DRIVE_FORCE_LOW_MAX_TURN : Constants.CHEESY_DRIVE_MAX_TURN;
        return DriveUtils.cheesyTurn(getThrottle(), 
            DriveUtils.deadbandExponential(
                getRightStickX(), Constants.DRIVE_SPEED_EXP_XBOX, Constants.XBOX_DEADBAND),
            cheesyDriveMinTurn, cheesyDriveMaxTurn);
    }

    @Override
    public boolean getForceLowGear() {
        return getLeftTrigger() > 0.5;
    }

    @Override
    public boolean getMolassesMode() {
        return getRightTrigger() > 0.5;
    }
    
}
