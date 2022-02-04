// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretIdle extends InstantCommand {
  private Turret m_turret;
  private Sensors m_sensors;

  public TurretIdle(Turret turret, Sensors sensors) {
    m_turret = turret;
    m_sensors = sensors;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_sensors.limelight.ledOff();
    m_turret.goToPosition(0.0);
  }
}
