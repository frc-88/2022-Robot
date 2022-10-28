package frc.robot.util.controllers;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;
import frc.robot.util.drive.DriveUtils;

public class XboxDriverController extends XboxController implements DriverController{

    public XboxDriverController(int port) {
        super(port);
    }

    @Override
    public double getTranslationX() {
        return getLeftStickX();
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
        return buttonBack.get();
    }

    @Override
    public Button getShootButton() {
        return buttonLeftBumper;
    }
    
    
}
