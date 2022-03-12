/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.transmission.ShiftingTransmission;

/**
 * Add your docs here.
 */
public class TJDriveModule extends WPI_TalonFX {
    private WPI_TalonFX followers[];

    private ShiftingTransmission m_transmission;

    public TJDriveModule(TJDriveModuleConfiguration config, ShiftingTransmission transmission) {
        super(config.master);
        this.configFactoryDefault();
        this.configAllSettings(config.masterConfiguration);
        this.enableVoltageCompensation(config.enableVoltageCompensation);
        this.setInverted(config.invertMotor);
        this.setSensorPhase(config.invertSensor);
        this.setNeutralMode(config.neutralMode);

        followers = new WPI_TalonFX [config.followers.length];
        for (int i = 0; i < config.followers.length; i++) {
            followers[i] = new WPI_TalonFX(config.followers[i]);
            followers[i].configFactoryDefault();
            followers[i].configAllSettings(config.followerConfiguration);
            followers[i].follow(this);
            followers[i].setInverted(config.invertMotor);
            followers[i].setSensorPhase(config.invertSensor);
            followers[i].setNeutralMode(config.neutralMode);
        }

        setStatusFrames();

        m_transmission = transmission;
    }

    /**
     * Puts all talons in brake mode.
     */
    public void brakeAll() {
        this.setNeutralMode(NeutralMode.Brake);
        for (WPI_TalonFX follower : followers) {
            follower.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * Puts all talons in coast mode.
     */
    public void coastAll() {
        this.setNeutralMode(NeutralMode.Coast);
        for (WPI_TalonFX follower : followers) {
            follower.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Get the number of talon followers this drive module has.
     */
    public int getNumTalonFollowers() {
        return followers.length;
    }

    /**
     * Get the output current of the ith talon follower
     */
    public double getFollowerCurrent(int i) {
        return followers[i].getSupplyCurrent();
    }

    /**
     * Get the total output current of the master and all talon followers
     */
    public double getTotalCurrent() {
        double total = this.getSupplyCurrent();
        for (WPI_TalonFX follower : followers) {
            total += follower.getSupplyCurrent();
        }
        return total;
    }

    /**
     * Get the position being read by the sensor on the master, scaled to the
     * output units of the transmission.
     */
    public double getScaledSensorPosition() {
        return m_transmission.convertSensorPositionToOutput(
                this.getSelectedSensorPosition());
    }

    /**
     * Get the velocity being read by the sensor on the master, scaled to the
     * output units of the transmission.
     */
    public double getScaledSensorVelocity() {
        return m_transmission.convertSensorVelocityToOutput(
                this.getSelectedSensorVelocity());
    }

    /**
     * Get the velocity reading of the Falcon sensor assuming we are in low gear.
     */
    public double getFalconVelocityLowGear() {
        return this.getSelectedSensorVelocity(1) * m_transmission.getLowInputToOutputRatio() * 10 / 2048;
    }

    /**
     * Get the velocity reading of the Falcon sensor assuming we are in low gear.
     */
    public double getFalconVelocityHighGear() {
        return this.getSelectedSensorVelocity(1) * m_transmission.getHighInputToOutputRatio() * 10 / 2048;
    }


    /**
     * Drives this module to the given output velocity while using a system
     * model of the drivetrain to not command a voltage that would result in
     * exceeding the given current limit.
     * 
     * Parameters:
     * @param targetVelocity The desired velocity at the output of the transmission
     * @param currentLimit The maximum current, in amps, that we want the sum
     *                     of all the motors to draw
     */
    public void setVelocityCurrentLimited(double targetVelocity, double currentLimit) {
        double percentOut = m_transmission.getCurrentLimitedVoltage(
            targetVelocity, this.getSelectedSensorVelocity(), currentLimit) / 12.;
        this.set(ControlMode.PercentOutput, Math.max(-1., Math.min(1., percentOut)));
    }

    /**
     * Gets the expected current draw in order to achieve the given velocity.
     * @param targetVelocity The velocity to target
     */
    public double getExpectedCurrentDraw(double targetVelocity) {
        return m_transmission.getExpectedCurrentDraw(targetVelocity, this.getSelectedSensorVelocity());
    }

    public void setStatusFrames() {
        this.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20);
    }

}