package frc.team2767.pathbot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.pathbot.Robot;
import frc.team2767.pathbot.subsystem.DriveSubsystem;

public class PathCommand extends Command {

  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private final String name;

  public PathCommand(String name) {
    this.name = name;
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.startPath(name, 0.0);
  }

  @Override
  protected void interrupted() {
    DRIVE.interruptPath();
  }

  @Override
  protected boolean isFinished() {
    return DRIVE.isPathFinished();
  }
}
