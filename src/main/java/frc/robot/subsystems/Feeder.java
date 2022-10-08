package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Feeder extends SubsystemBase {

    private final TalonFX m_centralizer;
    private final TalonFX m_chamber;

    private final DoublePreferenceConstant p_centralizerIntakeSpeed;
    private final DoublePreferenceConstant p_centralizerOutgestSpeed;
    private final DoublePreferenceConstant p_centralizerShootSpeed;
    private final DoublePreferenceConstant p_chamberOutgestSpeed;
    private final DoublePreferenceConstant p_chamberShootSpeed;

    public Feeder() {
        m_centralizer = new TalonFX(Constants.CENTRALIZER_ID, "1");
        m_chamber = new TalonFX(Constants.CHAMBER_ID, "1");

        m_centralizer.configFactoryDefault();
        m_chamber.configFactoryDefault();

        m_chamber.configOpenloopRamp(0.1);
        m_centralizer.configOpenloopRamp(0.1);

        p_centralizerIntakeSpeed = new DoublePreferenceConstant("Centralizer Intake Speed", .75);
        p_centralizerOutgestSpeed = new DoublePreferenceConstant("Centralizer Outgest Speed", 1);
        p_centralizerShootSpeed = new DoublePreferenceConstant("Centralizer Shoot Speed", .75);
        p_chamberOutgestSpeed = new DoublePreferenceConstant("Chamber Outgest Speed", 1);
        p_chamberShootSpeed = new DoublePreferenceConstant("Chamber Shoot Speed", .75);
    }

    public boolean hasBallInCentralizer() {
        return m_centralizer.isFwdLimitSwitchClosed() > 0;
    }

    public boolean hasBallInChamber() {
        return m_chamber.isFwdLimitSwitchClosed() > 0;
    }

    private void enableLimits() {
        m_centralizer.overrideLimitSwitchesEnable(true);
        m_chamber.overrideLimitSwitchesEnable(true);
    }
    private void disableLimits() {
        m_centralizer.overrideLimitSwitchesEnable(false);
        m_chamber.overrideLimitSwitchesEnable(false);
    }

    public void intake() {
        enableLimits();
        m_chamber.set(TalonFXControlMode.PercentOutput, 0);
        if (!hasBallInChamber() || !hasBallInCentralizer()) {
            m_centralizer.set(TalonFXControlMode.PercentOutput, p_centralizerIntakeSpeed.getValue());
        } else {
            m_centralizer.set(TalonFXControlMode.PercentOutput, 0);
        }
    }

    public void outgest() {
        disableLimits();
        m_chamber.set(TalonFXControlMode.PercentOutput, -p_chamberOutgestSpeed.getValue());
        m_centralizer.set(TalonFXControlMode.PercentOutput, -p_centralizerOutgestSpeed.getValue());
    }

    public void shoot() {
        enableLimits();
        m_chamber.set(TalonFXControlMode.PercentOutput, p_chamberShootSpeed.getValue());
        if (hasBallInChamber()) {
            m_centralizer.set(TalonFXControlMode.PercentOutput, 0);
        } else {
            m_centralizer.set(TalonFXControlMode.PercentOutput, p_centralizerShootSpeed.getValue());
        }
    }

    public void hold() {
      if (hasBallInCentralizer()) {
        intake();
      } else {
        m_chamber.set(TalonFXControlMode.PercentOutput, 0);
        m_centralizer.set(TalonFXControlMode.PercentOutput, 0);
      }
    }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putBoolean("Ball In Centralizer", hasBallInCentralizer());
    //     SmartDashboard.putBoolean("Ball In Chamber", hasBallInChamber());
    // }
    
}
