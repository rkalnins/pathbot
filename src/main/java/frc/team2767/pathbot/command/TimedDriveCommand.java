package frc.team2767.pathbot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.pathbot.Robot;
import frc.team2767.pathbot.subsystem.DriveSubsystem;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;

public class TimedDriveCommand extends Command {

  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private static final double TIME = 2.0;
  private static final double FORWARD = 0.2;
  private double start;

  public TimedDriveCommand() {}

  @Override
  protected void initialize() {
    start = Timer.getFPGATimestamp();
    DRIVE.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);
    DRIVE.drive(FORWARD, 0.0, 0.0);
  }

  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() > start + TIME;
  }
}
