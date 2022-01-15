/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.Arrays;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.util.transmission.Falcon500;

public class CalculateDriveEfficiency extends CommandBase {

  private Drive drive;

  private static final double VOLTAGE_START = 0.25;
  private static final double VOLTAGE_INCREMENT = 0.25;
  private static final double MAX_VOLTAGE = 12.0;
  private static final double RAMP_TIME = 2_000_000; // 2 seconds
  private static final int NUM_SAMPLES = 5;

  private double currentVoltage;
  private double currentDirection;
  private boolean ramping;
  private long rampStartTime;
  private double[] leftEfficiencies = new double[NUM_SAMPLES];
  private double[] rightEfficiencies = new double[NUM_SAMPLES];
  private int efficiencyIdx;

  /**
   * Creates a new CalculateDriveEfficiency.
   */
  public CalculateDriveEfficiency(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentVoltage = VOLTAGE_START;
    currentDirection = 1.;
    ramping = true;
    rampStartTime = RobotController.getFPGATime();
    efficiencyIdx = 0;

    SmartDashboard.putBoolean("SetInHigh", false);
    SmartDashboard.putBoolean("RunEfficiencyTest", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!SmartDashboard.getBoolean("RunEfficiencyTest", false)) {
      drive.basicDrive(0, 0);
      return;
    }

    double staticFrictionVoltage;
    double gearRatio;
    if (SmartDashboard.getBoolean("SetInHigh", false)) {
      drive.shiftToHigh();
      staticFrictionVoltage = Constants.DRIVE_HIGH_STATIC_FRICTION_VOLTAGE;
      gearRatio = Constants.HIGH_DRIVE_RATIO;
    } else {
      drive.shiftToLow();
      staticFrictionVoltage = Constants.DRIVE_LOW_STATIC_FRICTION_VOLTAGE;
      gearRatio = Constants.LOW_DRIVE_RATIO;
    }

    if (ramping && RobotController.getFPGATime() - rampStartTime > RAMP_TIME) {
      ramping = false;
      efficiencyIdx = 0;
    } else if (!ramping && efficiencyIdx >= NUM_SAMPLES) {
      System.out.println("Voltage: " + currentVoltage
          + " -- Left Eff: " + Arrays.stream(leftEfficiencies).average()
          + " -- Right Eff: " + Arrays.stream(rightEfficiencies).average());

      ramping = true;
      currentDirection *= -1;
      currentVoltage += VOLTAGE_INCREMENT;
      rampStartTime = RobotController.getFPGATime();
    }

    if (!ramping) {
      double expectedSpeed = currentDirection * (currentVoltage - staticFrictionVoltage)
          / (new Falcon500()).getVelocityConstant() * gearRatio;
      leftEfficiencies[efficiencyIdx] = drive.getLeftSpeed() / expectedSpeed;
      rightEfficiencies[efficiencyIdx] = drive.getRightSpeed() / expectedSpeed;
      ++efficiencyIdx;
    }

    double commandPercent = currentDirection * currentVoltage / 12.;
    drive.basicDrive(commandPercent, commandPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.basicDrive(0, 0);

    SmartDashboard.delete("SetInHigh");
    SmartDashboard.delete("RunEfficiencyTest");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentVoltage > MAX_VOLTAGE;
  }
}
