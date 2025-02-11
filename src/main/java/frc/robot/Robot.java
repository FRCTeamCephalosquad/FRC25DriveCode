// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftDrive = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightDrive = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_leftDriveFollower = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_rightDriveFollower = new SparkMax(5, MotorType.kBrushed);
  public static SparkMax m_coralFeeder = new SparkMax(6, MotorType.kBrushless);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);

    m_robotDrive.setMaxOutput(0.5);

    SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    rightFollowConfig.follow(m_rightDrive);
    m_rightDriveFollower.configure(rightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
    leftFollowConfig.follow(m_leftDrive);
    m_leftDriveFollower.configure(leftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else if (m_timer.get() > 2 && m_timer.get() < 2.5) {
      //m_leftDrive.set(-0.1);
      //m_rightDrive.set(0.1);
      m_robotDrive.arcadeDrive(0, .5, false);
    } else if (m_timer.get() > 2.5 && m_timer.get() < 3.5) {
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }

    // If time is greater than two but less than three run the coral feeder at 40%
    // speed

    if (m_timer.get() > 3.5 && m_timer.get() < 4.5) {
      m_coralFeeder.set(-0.4);
    } else {
      m_coralFeeder.stopMotor();
    }

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    if (m_controller.getRightBumperButton()) {
      // do this
      m_coralFeeder.set(-0.4);
    } else {
      // do that
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
