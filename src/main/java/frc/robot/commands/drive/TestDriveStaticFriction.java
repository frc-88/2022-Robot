/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TestDriveStaticFriction extends CommandBase {

  private Drive drive;

  /**
   * Creates a new TestDriveStaticFriction.
   */
  public TestDriveStaticFriction(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SetLeftSpeed", 0);
    SmartDashboard.putNumber("SetRightSpeed", 0);
    SmartDashboard.putBoolean("SetInHigh", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("SetInHigh", false)) {
      drive.shiftToHigh();
    } else {
      drive.shiftToLow();
    }
    drive.basicDrive(SmartDashboard.getNumber("SetLeftSpeed", 0) / 12.,
        SmartDashboard.getNumber("SetRightSpeed", 0) / 12.);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.delete("SetLeftSpeed");
    SmartDashboard.delete("SetRightSpeed");
    SmartDashboard.delete("SetInHigh");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
