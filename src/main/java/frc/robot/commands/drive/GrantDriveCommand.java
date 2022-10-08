package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class GrantDriveCommand extends CommandBase {
    private final SwerveDrive m_drivetrainSubsystem;

    private final DoubleSupplier m_throttleSupplier;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public GrantDriveCommand(SwerveDrive drivetrainSubsystem,
                               DoubleSupplier throttleSupplier,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_throttleSupplier = throttleSupplier;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double heading = Math.atan2(-m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());
        double vx = m_throttleSupplier.getAsDouble() * Math.cos(heading);
        double vy = m_throttleSupplier.getAsDouble() * Math.sin(heading);

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy,
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
