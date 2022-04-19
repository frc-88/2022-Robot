// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.util.CargoSource;
import frc.robot.util.CargoTarget;

public class FeederPassthru extends CommandBase {
  private Feeder m_feeder;
  private CargoSource m_source;
  private CargoTarget m_target;

  /** Creates a new FeederPassThru. */
  public FeederPassthru(Feeder feeder, CargoSource source, CargoTarget target) {
    m_feeder = feeder;
    m_source = source;
    m_target = target;

    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_target.wantsCargo() && m_source.hasCargo()) {
      m_feeder.forceForwards();
    } else {
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
