// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.sensors.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightToggle extends InstantCommand {
  private boolean m_on;
  private Limelight m_limelight;

  public LimelightToggle(Limelight limelight, boolean on) {
    m_on = on;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_on) {
      m_limelight.ledOn();
    } else {
      m_limelight.ledOff();
    }

  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}



