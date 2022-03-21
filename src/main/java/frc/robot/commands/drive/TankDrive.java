/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TankDrive extends CommandBase {
  private Drive drive;
  private DoubleSupplier leftVelocity;
  private DoubleSupplier rightVelocity;

  /**
   * Creates a new TankDrive.
   */
  public TankDrive(Drive drive, DoubleSupplier leftVelocity, DoubleSupplier rightVelocity) {
    this.drive = drive;
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.shiftToHigh();
    // drive.updateCurrentGear();
    drive.basicDrive(leftVelocity.getAsDouble(), rightVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.basicDriveLimited(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
