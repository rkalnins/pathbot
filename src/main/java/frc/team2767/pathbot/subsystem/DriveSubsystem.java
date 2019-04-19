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
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.thirdcoast.swerve.SwerveDrive;
import org.strykeforce.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.strykeforce.thirdcoast.swerve.SwerveDriveConfig;
import org.strykeforce.thirdcoast.swerve.Wheel;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;
import org.strykeforce.thirdcoast.telemetry.grapher.Measure;
import org.strykeforce.thirdcoast.telemetry.item.Item;

public class DriveSubsystem extends Subsystem implements Item {

  public static final double DRIVE_SETPOINT_MAX = 40_000.0;
  private static final double ROBOT_LENGTH = 21.0;
  private static final double ROBOT_WIDTH = 26.0;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  private PathController pathController;
  private boolean isDrivingPath = false;
  private double forward;
  private double strafe;
  private double yaw;
  private double pathTargetYaw;
  private Wheel[] wheels;
  private final SwerveDrive swerve = getSwerve();

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

  public void zeroGyro() {
    AHRS gyro = swerve.getGyro();
    gyro.setAngleAdjustment(0);
    double adj = gyro.getAngle() % 360;
    gyro.setAngleAdjustment(-adj);
    logger.info("resetting gyro zero ({})", adj);
  }

  public void startPath(String path, double targetYaw) {
    logger.debug("starting path");
    this.pathController = new PathController(path, targetYaw);
    pathTargetYaw = targetYaw;
    isDrivingPath = true;
    pathController.start();
  }

  public boolean isPathFinished() {

    if (pathController.isFinished()) {
      isDrivingPath = false;
      return true;
    }

    return false;
  }

  public void interruptPath() {
    logger.debug("path interrupted");
    pathController.interrupt();
    isDrivingPath = false;
  }

  public void driveWheels(double azimuth, double drive) {
    for (Wheel wheel : swerve.getWheels()) wheel.set(azimuth, drive);
  }

  public int getDrivePosition(int wheel) {
    return swerve.getWheels()[wheel].getDriveTalon().getSelectedSensorPosition(0);
  }

  public boolean isDrivingPath() {
    return isDrivingPath;
  }

  private SwerveDrive getSwerve() {
    SwerveDriveConfig config = new SwerveDriveConfig();
    config.wheels = getWheelsConfig();
    config.gyro = new AHRS(SPI.Port.kMXP);
    config.length = ROBOT_LENGTH;
    config.width = ROBOT_WIDTH;
    config.gyroLoggingEnabled = true;
    config.summarizeTalonErrors = false;

    return new SwerveDrive(config);
  }

  // Swerve configuration

  private Wheel[] getWheelsConfig() {
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
    driveConfig.slot0.kP = 0.01;
    driveConfig.slot0.kI = 0.0003;
    driveConfig.slot0.kD = 0.60;
    driveConfig.slot0.kF = 0.028;
    driveConfig.slot0.integralZone = 3000;
    driveConfig.slot0.maxIntegralAccumulator = 200_000;
    driveConfig.slot0.allowableClosedloopError = 0;
    driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
    driveConfig.velocityMeasurementWindow = 64;
    driveConfig.voltageCompSaturation = 12;

    TelemetryService telemetryService = Robot.TELEMETRY;
    telemetryService.stop();
    telemetryService.register(this);

    wheels = new Wheel[4];

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

  public Wheel[] getWheels() {
    return wheels;
  }

  public AHRS getGyro() {
    return swerve.getGyro();
  }

  public void stop() {
    drive(0, 0, 0);
  }

  public void drive(double forward, double strafe, double azimuth) {
    this.forward = forward;
    this.strafe = strafe;
    this.yaw = azimuth;
    swerve.drive(forward, strafe, azimuth);
  }

  @NotNull
  @Override
  public String getDescription() {
    return "drive";
  }

  @Override
  public int getDeviceId() {
    return 0;
  }

  @NotNull
  @Override
  public Set<Measure> getMeasures() {
    return Set.of(
        Measure.ANGLE,
        Measure.COMPONENT_FORWARD,
        Measure.COMPONENT_STRAFE,
        Measure.COMPONENT_YAW,
        Measure.CLOSED_LOOP_ERROR);
  }

  @NotNull
  @Override
  public String getType() {
    return "drive subsystem";
  }

  @Override
  public int compareTo(@NotNull Item item) {
    return 0;
  }

  @NotNull
  @Override
  public DoubleSupplier measurementFor(@NotNull Measure measure) {
    switch (measure) {
      case ANGLE:
        return () -> Math.IEEEremainder(swerve.getGyro().getAngle(), 360);
      case COMPONENT_FORWARD:
        return () -> forward;
      case COMPONENT_STRAFE:
        return () -> strafe;
      case COMPONENT_YAW:
        return () -> yaw;
      case CLOSED_LOOP_ERROR:
        return () -> pathTargetYaw - Math.IEEEremainder(swerve.getGyro().getAngle(), 360);
      default:
        return () -> 2767.0;
    }
  }
}
