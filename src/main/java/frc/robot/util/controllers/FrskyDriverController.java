package frc.robot.util.controllers;

import frc.robot.Constants;
import frc.robot.util.drive.DriveUtils;

public class FrskyDriverController extends FrskyController implements DriverController{

    public FrskyDriverController(int port) {
        super(port);
    }

    @Override
    public double getThrottle() {
        double ret = DriveUtils.deadbandExponential(getLeftStickY(), Constants.DRIVE_SPEED_EXP_FRSKY, Constants.FRSKY_DEADBAND);
        // System.out.println("getThrottle(): " + ret);
        return ret;
    }

    @Override
    public double getTurn() {
        double cheesyDriveMinTurn = getForceLowGear() ? Constants.CHEESY_DRIVE_FORCE_LOW_MIN_TURN : Constants.CHEESY_DRIVE_MIN_TURN;
        double cheesyDriveMaxTurn = getForceLowGear() ? Constants.CHEESY_DRIVE_FORCE_LOW_MAX_TURN : Constants.CHEESY_DRIVE_MAX_TURN;
        double ret = DriveUtils.cheesyTurn(getThrottle(), 
            DriveUtils.deadbandExponential(
                getRightStickX(), Constants.DRIVE_SPEED_EXP_FRSKY, Constants.FRSKY_DEADBAND),
            cheesyDriveMinTurn, cheesyDriveMaxTurn);
            // System.out.println("getTurn(): " + ret);
        return ret;
    }

    @Override
    public boolean getForceLowGear() {
        return isTopRightSwitchOn();
    }

    @Override
    public boolean getShooterMode() {
        return isTopLeftSwitchOn();
    }
    
}
