/**
 * 
 */
package frc.robot.util.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * @author TJ2
 *
 */
public class SharpIR extends AnalogInput {

private static final int OVERSAMPLE_BITS = 4;
private static final int AVERAGE_BITS = 8;
private static final double K = 5.2725;
	
	/**
	 * @param channel
	 */
	public SharpIR(int channel) {
		super(channel);
		this.setOversampleBits(OVERSAMPLE_BITS);
		this.setAverageBits(AVERAGE_BITS);
	}
	
	public double getDistance() {
		return (K / getAverageVoltage()) - 0.42;
	}
	
	public double pidGet() {
		return getDistance();
	}

}
