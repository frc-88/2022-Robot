package frc.robot.util.controllers;

public class FrskyDriverController extends FrskyController implements DriverController{

    public FrskyDriverController(int port) {
        super(port);
    }

    @Override
    public double getTranslationX() {
        return -getLeftStickX();
    }

    @Override
    public double getTranslationY() {
        return getLeftStickY();
    }

    @Override
    public double getRotation() {
        return getRightStickX();
    }

    @Override
    public boolean getGyroReset() {
        return isTopRightSwitchOn();
    }
    
}
