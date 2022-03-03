// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretLock extends CommandBase {
  private Turret m_turret;
  /** Creates a new TurretLock. */
  public TurretLock(Turret turret) {
    m_turret = turret;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setNeutralMode(NeutralMode.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.goToFacing(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setNeutralMode(NeutralMode.Coast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
