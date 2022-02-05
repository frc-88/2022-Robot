package frc.robot.util.preferenceconstants;

import java.util.Objects;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Preferences constant for float values.
 */
public class FloatPreferenceConstant extends BasePreferenceConstant<Float> {

    String name;
    float defaultValue;

    /**
     * Constructor. Will call update() once.
     * 
     * @param name
     *                         The name to be used as a key in WPILib preferences
     * @param defaultValue
     *                         The value that will be set as default if the value
     *                         doesn't exist in WPILib preferences
     */
    public FloatPreferenceConstant(String name, float defaultValue) {
        this.name = Objects.requireNonNull(name);
        this.defaultValue = Objects.requireNonNull(defaultValue);
        if (!Preferences.containsKey(name)) {
            this.setValue(defaultValue);
        } else {
            update();
        }
    }

    @Override
    protected Float getFromPreferences() {
        return Preferences.getFloat(name, defaultValue);
    }

    @Override
    protected void setInPreferences(Float value) {
        Preferences.setFloat(name, value);
    }

}
