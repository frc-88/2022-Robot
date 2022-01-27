// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private TalonFX centralizerMotor, chamberMotor;
  private DigitalInput centralizer, chamber;

  /** Creates a new Hopper. */
  public Hopper() {
    centralizerMotor = new TalonFX(Constants.HOPPER_CENTRALIZER_MOTOR_ID);
    chamberMotor = new TalonFX(Constants.HOPPER_CHAMBER_MOTOR_ID);
    centralizer = new DigitalInput(Constants.HOPPER_CENTRALIZER_BEAMBREAK);
    chamber = new DigitalInput(Constants.HOPPER_CHAMBER_BEAMBREAK);
  }

  public boolean isCargoInCentralizer() {
    return centralizer.get();
  }

  public boolean isCargoInChamber() {
    return chamber.get();
  }

  public void runCentralizer() {
    centralizerMotor.set(ControlMode.PercentOutput, Constants.HOPPER_CENTRALIZER_SPEED);
  }

  public void reverseCentralizer() {
    centralizerMotor.set(ControlMode.PercentOutput, -Constants.HOPPER_CENTRALIZER_SPEED);
  }

  public void stopCentralizer() {
    centralizerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void runChamber() {
    chamberMotor.set(ControlMode.PercentOutput, Constants.HOPPER_CHAMBER_SPEED);
  }

  public void reverseChamber() {
    chamberMotor.set(ControlMode.PercentOutput, -Constants.HOPPER_CHAMBER_SPEED);
  }

  public void stopChamber() {
    chamberMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Centralizer Cargo?", isCargoInCentralizer());
    SmartDashboard.putBoolean("Chamber Cargo?", isCargoInChamber());
  }
}
