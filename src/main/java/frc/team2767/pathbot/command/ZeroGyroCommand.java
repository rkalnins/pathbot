package frc.team2767.pathbot.command;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team2767.pathbot.Robot;
import frc.team2767.pathbot.subsystem.DriveSubsystem;

public final class ZeroGyroCommand extends InstantCommand {

  private static final DriveSubsystem swerve = Robot.DRIVE;

  public ZeroGyroCommand() {
    requires(swerve);
  }

  @Override
  protected void initialize() {
    swerve.zeroGyro();
  }
}
