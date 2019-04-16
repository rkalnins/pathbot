package frc.team2767.pathbot.subsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2767.pathbot.Robot;
import frc.team2767.pathbot.command.TeleOpDriveCommand;
import frc.team2767.pathbot.motion.PathController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

public class DriveSubsystem extends Subsystem {

  private static final double DRIVE_SETPOINT_MAX = 25_000.0;
  private static final double ROBOT_LENGTH = 21.0;
  private static final double ROBOT_WIDTH = 26.0;

  private final SwerveDrive swerve = getSwerve();
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private PathController pathController;

  public DriveSubsystem() {}

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new TeleOpDriveCommand());
  }

  public void setDriveMode(DriveMode mode) {
    logger.debug("setting drive mode to {}", mode);
    swerve.setDriveMode(mode);
  }

  public void zeroAzimuthEncoders() {
    swerve.zeroAzimuthEncoders();
  }

  public void drive(double forward, double strafe, double azimuth) {
    swerve.drive(forward, strafe, azimuth);
  }

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
    logger.info("resetting gyro zero ({})", adj);
  }

  public void startPath(String path, double targetYaw) {
    logger.debug("starting path");
    this.pathController = new PathController(swerve, path, targetYaw);
    pathController.start();
  }

  public boolean isPathFinished() {
    return pathController.isFinished();
  }

  public void interruptPath() {
    logger.debug("path interrupted");
    pathController.interrupt();
  }

  // Swerve configuration

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheels();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  private Wheel[] getWheels() {
    TalonSRXConfiguration azimuthConfig = new TalonSRXConfiguration();
    // NOTE: ensure encoders are in-phase with motor direction. Encoders should increase
    // when azimuth motor runs in forward direction.
    azimuthConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    azimuthConfig.continuousCurrentLimit = 10;
    azimuthConfig.peakCurrentDuration = 0;
    azimuthConfig.peakCurrentLimit = 0;
    azimuthConfig.slot0.kP = 10.0;
    azimuthConfig.slot0.kI = 0.0;
    azimuthConfig.slot0.kD = 100.0;
    azimuthConfig.slot0.kF = 0.0;
    azimuthConfig.slot0.integralZone = 0;
    azimuthConfig.slot0.allowableClosedloopError = 0;
    azimuthConfig.motionAcceleration = 10_000;
    azimuthConfig.motionCruiseVelocity = 800;

    TalonSRXConfiguration driveConfig = new TalonSRXConfiguration();
    driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    driveConfig.continuousCurrentLimit = 40;
    driveConfig.peakCurrentDuration = 45;
    driveConfig.peakCurrentLimit = 40;
    driveConfig.slot0.kP = 0.05;
    driveConfig.slot0.kI = 0.0005;
    driveConfig.slot0.kD = 0.0;
    driveConfig.slot0.kF = 0.032;
    driveConfig.slot0.integralZone = 1000;
    driveConfig.slot0.maxIntegralAccumulator = 150_000;
    driveConfig.slot0.allowableClosedloopError = 0;
    driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
    driveConfig.velocityMeasurementWindow = 64;
    driveConfig.voltageCompSaturation = 12;

    TelemetryService telemetryService = Robot.TELEMETRY;
    telemetryService.stop();

    Wheel[] wheels = new Wheel[4];

    for (int i = 0; i < 4; i++) {
      TalonSRX azimuthTalon = new TalonSRX(i);
      azimuthTalon.configAllSettings(azimuthConfig);

      TalonSRX driveTalon = new TalonSRX(i + 10);
      driveTalon.configAllSettings(driveConfig);
      driveTalon.setNeutralMode(NeutralMode.Brake);

      telemetryService.register(azimuthTalon);
      telemetryService.register(driveTalon);

      Wheel wheel = new Wheel(azimuthTalon, driveTalon, DRIVE_SETPOINT_MAX);
      wheels[i] = wheel;
    }

    return wheels;
  }
}
