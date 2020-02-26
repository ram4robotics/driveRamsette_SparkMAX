/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_motorL, m_motorR;
  private final CANEncoder  m_encoderL, m_encoderR;
  private final CANPIDController m_pidVelocity;
  SimpleMotorFeedforward m_feedforward;

  private final Spark msim_motorL, msim_motorR;
  private final Encoder msim_encoderL, msim_encoderR;

  private double m_velocityL, m_velocityR;
  private double m_kP, m_kI, m_kD, m_kIz, m_kFF;
  private double m_velocitySetpoint;

  private ShuffleboardTab mytab = Shuffleboard.getTab("Launcher PIDF tuning");
  private NetworkTableEntry m_vel_setp_setter = mytab.add("Velocity setpoint setter", 2000.0).getEntry();

  private NetworkTableEntry m_velL_getter = mytab.add("Velocity L", 0.0).getEntry();
  private NetworkTableEntry m_velR_getter = mytab.add("Velocity R", 0.0).getEntry();
  private NetworkTableEntry m_kFF_getter = mytab.add("m_kFF value calculated", 0.0).getEntry();

  /**
   * Creates a new Launcher.
   */
  public Launcher() {
    if (Robot.isReal()) {
      m_motorL = new CANSparkMax(LauncherConstants.kLauncherMotorLeft_id, MotorType.kBrushless);
      m_motorR = new CANSparkMax(LauncherConstants.kLauncherMotorRight_id, MotorType.kBrushless);
      m_motorL.restoreFactoryDefaults();
      m_motorR.restoreFactoryDefaults();

      m_motorL.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
      m_motorR.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
      m_motorL.setIdleMode(LauncherConstants.kIdleMode);
      m_motorR.setIdleMode(LauncherConstants.kIdleMode);

      m_motorL.setInverted(true);
      m_motorR.follow(m_motorL, true); // Set the output of right motor to opposite of that of the left motor

      m_encoderL = new CANEncoder(m_motorL);
      m_encoderR = new CANEncoder(m_motorR);
      
      // Setup the controller on leader; the follower will get the voltage values from leader.
      m_pidVelocity = new CANPIDController(m_motorL);

      // set PID coefficients
      m_kP = LauncherConstants.kP_RPM;
      m_kI = LauncherConstants.kI;
      m_kD = LauncherConstants.kD;
      m_kIz = LauncherConstants.kIz;
      m_feedforward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);
      // Initially set the m_kFF with half the max velocity
      m_kFF = m_feedforward.calculate(LauncherConstants.kMaxRPM / (2 * 60), LauncherConstants.kMaxRPM / 60) / 
      (LauncherConstants.kMaxVoltage * LauncherConstants.kMaxRPM);
      // m_pidVelocity.setP(m_kP);
      m_pidVelocity.setP(m_kP);
      m_pidVelocity.setI(m_kI);
      m_pidVelocity.setD(m_kD);
      m_pidVelocity.setIZone(m_kIz);
      m_pidVelocity.setFF(m_kFF);
      m_pidVelocity.setOutputRange(LauncherConstants.kMinOutput, LauncherConstants.kMaxOutput);

      msim_motorL = msim_motorR = null;
      msim_encoderL = msim_encoderR = null;
    } else {
      // This code is running in Simulation
      msim_motorL = new Spark(4);
      msim_motorR = new Spark(5);
      msim_encoderL = new Encoder(4, 5);
      msim_encoderR = new Encoder(6, 7);

      m_motorL = m_motorR = null;
      m_encoderL = m_encoderR = null;
      m_pidVelocity = null;
      m_feedforward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);
    }
  }

  private boolean update_setPoint() {
    boolean setPointChanged = false;
    double m_velocitySetpoint_new = m_vel_setp_setter.getDouble(2000.0);
    if (m_velocitySetpoint_new != m_velocitySetpoint) {
      m_velocitySetpoint = m_velocitySetpoint_new;
      // Calculate m_kFF for the given set point velocity, and for acceleration of (maxRPM/60) Rotation/second^2.
      m_kFF = m_feedforward.calculate(m_velocitySetpoint / 60, LauncherConstants.kMaxRPM / 60) / 
        (LauncherConstants.kMaxVoltage * LauncherConstants.kMaxRPM);
      m_kFF_getter.setDouble(m_kFF);
      setPointChanged = true;
      if (Robot.isReal()) {
        m_pidVelocity.setFF(m_kFF);
        m_pidVelocity.setP(LauncherConstants.kP_RPM);
      }
    }
    return (setPointChanged);
  }

  @Override
  public void periodic() {
    // update_pidf();
    boolean setPointChanged = update_setPoint();
    if (Robot.isReal()) {
      // This method will be called once per scheduler run
      m_velocityL = m_encoderL.getVelocity();
      m_velocityR = m_encoderR.getVelocity();
    } else {
      m_velocityL = msim_encoderL.getRate();
      m_velocityR = msim_encoderR.getRate();
    }
    m_velL_getter.setDouble(m_velocityL);
    m_velR_getter.setDouble(m_velocityR);
    if (setPointChanged) {
      launchAtRPM(m_velocitySetpoint);
    }
  }


  public void launchAtRPM(double rpm) {
    System.out.println("lunchAtRPM: rpm = " + rpm);
    if (Robot.isReal()) {
      m_pidVelocity.setReference(rpm, ControlType.kVelocity);  
    } else {
      double pctSpeed = rpm / LauncherConstants.kMaxRPM;
      msim_motorL.set(pctSpeed);
      msim_motorR.set(-pctSpeed);
    }
  }

  public void launch(double pctSpeed) {
    if (Robot.isReal()) {
      double setPoint = pctSpeed * LauncherConstants.kMaxRPM;
      m_pidVelocity.setReference(setPoint, ControlType.kVelocity);  
    } else {
      msim_motorL.set(pctSpeed);
      msim_motorR.set(-pctSpeed);
    }
  }
}
