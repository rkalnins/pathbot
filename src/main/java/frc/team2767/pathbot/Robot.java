package frc.team2767.pathbot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team2767.pathbot.control.Controls;
import frc.team2767.pathbot.subsystem.DriveSubsystem;
import java.util.Date;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.telemetry.TelemetryController;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

public class Robot extends TimedRobot {

  public static final TelemetryService TELEMETRY = new TelemetryService(TelemetryController::new);
  public static final DriveSubsystem DRIVE = new DriveSubsystem();
  public static final Controls CONTROLS = new Controls();

  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  @Override
  public void robotInit() {
    logger.info("Today is {}", new Date());
    DRIVE.zeroAzimuthEncoders();
    DRIVE.zeroGyro();

    TELEMETRY.start();
  }

  @Override
  public void disabledInit() {
    if (DRIVE.isDrivingPath()) {
      DRIVE.interruptPath();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }
}
