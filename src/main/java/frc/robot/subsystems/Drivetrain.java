package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase{

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // radians

  private static final double kTrackWidth = 0.57; // meters
  private static final double kWheelRadius = 0.0762; // meters
  private static final double gearRatio = 22.67; // low gear

	public final DoubleSolenoid gearShift;

  private CANSparkMax frontLeft = new CANSparkMax(Constants.FL_DRIVE_PORT, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(Constants.FR_DRIVE_PORT, MotorType.kBrushless);
  private CANSparkMax midLeft = new CANSparkMax(Constants.ML_DRIVE_PORT, MotorType.kBrushless);
  private CANSparkMax midRight = new CANSparkMax(Constants.MR_DRIVE_PORT, MotorType.kBrushless);
  private CANSparkMax backLeft = new CANSparkMax(Constants.BL_DRIVE_PORT, MotorType.kBrushless);
  private CANSparkMax backRight = new CANSparkMax(Constants.BR_DRIVE_PORT, MotorType.kBrushless);
  
  private final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = frontRight.getEncoder();

  public final MotorControllerGroup m_leftGroup = new MotorControllerGroup(frontLeft, midLeft, backLeft);
  public final MotorControllerGroup m_rightGroup = new MotorControllerGroup(frontRight, midRight, backRight);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
 
  private final PIDController m_leftPIDController = new PIDController((309.0 / 4096.0), 0, (15.682 / 4096.0));
  private final PIDController m_rightPIDController = new PIDController((309.0 / 4096.0), 0, (15.682 / 4096.0));

  private final DifferentialDriveKinematics m_kinematics =new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.18346, 5.6299, 1.3615);

  //private final Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  //private final DifferentialDrive drive;

  // private final AnalogInput pressureSensor = new AnalogInput(0);

  PneumaticHub m_ph = new PneumaticHub(1);

  public Drivetrain() {

    gearShift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0);  

    m_rightGroup.setInverted(true);

    leftEncoder.setPositionConversionFactor((2 * Math.PI * kWheelRadius) / gearRatio);
    rightEncoder.setPositionConversionFactor((2 * Math.PI * kWheelRadius) / gearRatio);

    leftEncoder.setVelocityConversionFactor(((2
     * Math.PI * kWheelRadius) / gearRatio) / 60.0);
    rightEncoder.setVelocityConversionFactor(((2 * Math.PI * kWheelRadius) / gearRatio) / 60.0);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

    m_odometry = new DifferentialDriveOdometry(getYaw(), leftEncoder.getPosition(), -rightEncoder.getPosition());

    gearShift.set(DoubleSolenoid.Value.kForward);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    midLeft.setIdleMode(IdleMode.kBrake);
    midRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    m_ph.enableCompressorAnalog(85, 95);
  }

  // public void tankDrive(double left, double right, boolean squareInputs){
  //   drive.tankDrive(left, right, squareInputs);
  // }

  // public void disableCompressor(){
  //   pcmCompressor.disable();
  // }

  // public void enableCompressor(){
  //   pcmCompressor.enableDigital();
  // }


  public void burnFlash(){
    frontLeft.burnFlash();
    frontRight.burnFlash();
    midLeft.burnFlash();
    midRight.burnFlash();
    backLeft.burnFlash();
    backRight.burnFlash();
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  
   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftPIDController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(-rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  // public void arcadeDrive(double speed, double rotate, boolean squareInputs) {
	// 	drive.arcadeDrive(speed, rotate, squareInputs);
	// }

  public void setGearShift(boolean shift){
    if (shift){
      gearShift.set(DoubleSolenoid.Value.kForward);
    }
    else{
      gearShift.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void switchGears(){
    gearShift.toggle();
  }

  public double getEncoderLeft() {
    return (frontLeft.getEncoder().getPosition() + backLeft.getEncoder().getPosition() + midLeft.getEncoder().getPosition())/3.0;
  }
  
  public double getEncoderRight() {
    return (frontRight.getEncoder().getPosition() + backRight.getEncoder().getPosition() + midRight.getEncoder().getPosition())/3.0;
  }
  
  public double getEncoderAverage() {
    return (getEncoderLeft() - getEncoderRight()) / 2.0;
  }

  public Rotation2d getYaw() {
    return (Rotation2d.fromDegrees(-gyro.getYaw()));
  }
  
  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public void forgetAngle(){
    gyro.zeroYaw();
  }

  /** Updates the field-relative position. */
  
  public void updateOdometry() {
    m_odometry.update(getYaw(), leftEncoder.getPosition(), -rightEncoder.getPosition());
  }

  public RamseteController getRamseteController() {
      return new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
  }

  public SimpleMotorFeedforward getFeedforward() {
      return m_feedforward;
  }

  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(getYaw(), leftEncoder.getPosition(), -rightEncoder.getPosition(), pose);
  }

  public PIDController getLeftPIDController(){
    return m_leftPIDController;
  }

  public PIDController getRightPIDController(){
    return m_rightPIDController;
  }

  public Pose2d getPose(){
      return m_odometry.getPoseMeters();
  }

  public double getLeftEncoderRateAsMeters(){
      return (leftEncoder.getVelocity());
  }

  public double getRightEncoderRateAsMeters(){
      return (rightEncoder.getVelocity());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRateAsMeters(), getRightEncoderRateAsMeters());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
      m_leftGroup.setVoltage(leftVolts);
      m_rightGroup.setVoltage(rightVolts);
      // drive.feed();
      SmartDashboard.putNumber("Left Volts", leftVolts);
      SmartDashboard.putNumber("Right Volts", rightVolts);
  }

  public void resetEncoders() {
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
    midLeft.getEncoder().setPosition(0);
    midRight.getEncoder().setPosition(0);
    backLeft.getEncoder().setPosition(0);
    backRight.getEncoder().setPosition(0);
  }

  public double getRoll(){
    return gyro.getRoll();
  }

  public void setDriveMotors(double speed){
    frontLeft.set(speed);
    frontRight.set(speed);
    midLeft.set(speed);
    midRight.set(speed);
    backLeft.set(speed);
    backRight.set(speed);
  }


  @Override
        public void periodic() {
        // This method will be called once per scheduler run
        updateOdometry();
        SmartDashboard.putNumber("Gyro Degrees", (getYaw().getDegrees()));
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("rightEncoder", -rightEncoder.getPosition());
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getPosition());      
        SmartDashboard.putNumber("PoseX", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("PoseY", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Wheel Speed Left", getLeftEncoderRateAsMeters());
        SmartDashboard.putNumber("Wheel Speed Right", getRightEncoderRateAsMeters());
        SmartDashboard.putNumber("roll", getRoll());
        SmartDashboard.putNumber("PSI(value)", m_ph.getPressure(0));
        // if (m_ph.getPressure(0) > 115){
        //   pcmCompressor.disable();
        // }
        // if (m_ph.getPressure(0) <= 100){
        //   pcmCompressor.enableDigital();
        // }
    }


}