/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveTrain extends SubsystemBase {
  CANSparkMax m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave;
  CANEncoder  neo_leftEncoder, neo_rightEncoder;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  DifferentialDrive m_dDrive;
  Supplier<Double> gyroAngleRadians;
  AHRS m_navx;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    m_leftMaster = new CANSparkMax(DriveConstants.kLeftMotor1_id, MotorType.kBrushless);
    m_leftMaster.restoreFactoryDefaults();
    m_leftMaster.setInverted(false);
    m_leftMaster.setIdleMode(IdleMode.kBrake);
    neo_leftEncoder = m_leftMaster.getEncoder();
    neo_leftEncoder.setPositionConversionFactor(DriveConstants.kNeoPositionConversionFactor);
    neo_leftEncoder.setVelocityConversionFactor(DriveConstants.kNeoVelocityConversionFactor);

    m_leftSlave = new CANSparkMax(DriveConstants.kLeftMotor2_id, MotorType.kBrushless);
    m_leftSlave.restoreFactoryDefaults();
    m_leftSlave.setIdleMode(IdleMode.kBrake);
    m_leftSlave.follow(m_leftMaster);

    m_rightMaster = new CANSparkMax(DriveConstants.kRightMotor1_id, MotorType.kBrushless);
    m_rightMaster.restoreFactoryDefaults();
    m_rightMaster.setInverted(false);
    m_rightMaster.setIdleMode(IdleMode.kBrake);
    neo_rightEncoder = m_rightMaster.getEncoder();
    neo_rightEncoder.setPositionConversionFactor(DriveConstants.kNeoPositionConversionFactor);
    neo_rightEncoder.setVelocityConversionFactor(DriveConstants.kNeoVelocityConversionFactor);

    m_rightSlave = new CANSparkMax(DriveConstants.kRightMotor2_id, MotorType.kBrushless);
    m_rightSlave.restoreFactoryDefaults();
    m_rightSlave.setIdleMode(IdleMode.kBrake);
    m_rightSlave.follow(m_rightMaster);

    m_dDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
    m_dDrive.setDeadband(0);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    m_navx = new AHRS(SPI.Port.kMXP);
    gyroAngleRadians = () -> -1 * Math.toRadians(m_navx.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    // Note that the setPositionConversionFactor() has been applied to the left and right encoders in the constructor.
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), neo_leftEncoder.getPosition(), neo_rightEncoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(neo_leftEncoder.getVelocity(), neo_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_dDrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // ToDo:  Verify whether setVolatge() is applied to the follower motor controller.
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(-rightVolts);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Note that the position conversion factors have been applied to both the encoders in the constructor
    return (neo_leftEncoder.getPosition() + neo_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return neo_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return neo_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_dDrive.setMaxOutput(maxOutput);
  }


  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    neo_leftEncoder.setPosition(0.0);
    neo_rightEncoder.setPosition(0.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
