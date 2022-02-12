/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ArcadeDrive extends CommandBase {
  private Drive drive;
  private DoubleSupplier speed;
  private DoubleSupplier turn;
  private Runnable shift;
  private DoubleSupplier maxSpeed;

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Drive drive, DoubleSupplier speed, DoubleSupplier turn, Runnable shift, DoubleSupplier maxSpeed) {
    this.turn = turn;
    this.drive = drive;
    this.speed = speed;
    this.shift = shift;
    this.maxSpeed = maxSpeed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setMaxSpeed(maxSpeed.getAsDouble());
    shift.run();
    drive.updateCurrentGear();
    drive.arcadeDrive(speed.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMaxSpeed(Constants.MAX_SPEED_HIGH);
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
