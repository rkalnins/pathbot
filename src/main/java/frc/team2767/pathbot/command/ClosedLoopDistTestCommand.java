package frc.team2767.pathbot.command;

import static org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode.CLOSED_LOOP;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.pathbot.Robot;
import frc.team2767.pathbot.subsystem.DriveSubsystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/** Test closed loop drive-velocity mode. */
public class ClosedLoopDistTestCommand extends Command {

  private static final Logger logger = LoggerFactory.getLogger(ClosedLoopDistTestCommand.class);
  private static final DriveSubsystem DRIVE = Robot.DRIVE;
  private static final int DRIVE_DIST = 200_000;
  private double[] talonPositions = new double[4];
  private double[] curTalonPositions = new double[4];

  public ClosedLoopDistTestCommand() {
    requires(DRIVE);
  }

  @Override
  protected void initialize() {
    DRIVE.setDriveMode(CLOSED_LOOP);
    for (int i = 0; i < 4; i++) {
      talonPositions[i] = DRIVE.getDrivePosition(i);
    }
    DRIVE.driveWheels(0, 0.5);
  }

  @Override
  protected boolean isFinished() {
    double sum = 0;

    for (int i = 0; i < 4; i++) {
      curTalonPositions[i] = DRIVE.getDrivePosition(i);
      sum += Math.abs(curTalonPositions[i] - talonPositions[i]);
    }
    logger.debug("wheels moved {}", sum / 4);

    return (sum / 4) >= DRIVE_DIST;
  }

  @Override
  protected void end() {
    DRIVE.stop();
  }
}
