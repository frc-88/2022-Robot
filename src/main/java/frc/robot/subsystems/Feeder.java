// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private TalonFX feederMotor;
  private DigitalInput feederBeambreak;
  private double feederMotorSpeed;

  public Feeder(int feederMotorId, int feederSensorId, double feederMotorSpeed) {
    feederMotor = new TalonFX(feederMotorId);
    feederBeambreak = new DigitalInput(feederSensorId);
    this.feederMotorSpeed = feederMotorSpeed;
  }

  public boolean hasCargo() {
    return feederBeambreak.get();
  }

  public void run() {
    feederMotor.set(ControlMode.PercentOutput, feederMotorSpeed);
  }

  public void reverse() {
    feederMotor.set(ControlMode.PercentOutput, -feederMotorSpeed);
  }

  public void stop() {
    feederMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Feeder:" + feederMotor.getDeviceID() + ":hasCargo?", hasCargo());
  }
}
