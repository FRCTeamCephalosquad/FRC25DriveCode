// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

  private final AHRS gyro = new AHRS(NavXComType.kUSB1);
  private final Vision vision = new Vision();

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

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  Command driveForTime(double x) {
    return Commands.sequence(
        new RunCommand(() -> m_robotDrive.arcadeDrive(0.5, 0.0, false)).raceWith(new WaitCommand(x)),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.0, false)));
  }

  Command turnForTime(double x) {
    return Commands.sequence(
        new RunCommand(() -> m_robotDrive.arcadeDrive(0, -0.5, false)).raceWith(new WaitCommand(x)),
        new InstantCommand(() -> m_robotDrive.arcadeDrive(0, 0.0, false)));
  }

  Command coralYeeter() {
    return Commands.sequence(
        new RunCommand(() -> m_coralFeeder.set(-0.2)).raceWith(new WaitCommand(1)),
        new InstantCommand(() -> m_coralFeeder.stopMotor()));
  }

  Command park(){
    return new RunCommand(()-> m_robotDrive.stopMotor());
  }

  Command seekAprilTagAhead(int tagId) {
    return new Command() {
      @Override
      public void initialize() {
        m_robotDrive.setDeadband(0);
        vision.lookFor(tagId);
      }

      @Override
      public void end(boolean interrited) {
        m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
        vision.lookFor(0);
      }

      @Override
      public void execute() {

        if (vision.isTargetInSight()) {
          System.out.println("I SEE IT! " + vision.targetPosition());
          double targetOffset = -vision.targetPosition();
          double turn = 0;
          if (targetOffset > .3) {
            turn = .5;
          } else if (targetOffset < -.3) {
            turn = -.5;
          } else if (targetOffset > .1) {
            turn = .3;
          } else if (targetOffset < -.1) {
            turn = -.3;
          }
          // turn = clamp(Math.sqrt(turn), -.9, .9);
          m_robotDrive.arcadeDrive(0.5, turn, false);
        } else {
          System.out.println("I don't see it yet...");
          m_robotDrive.arcadeDrive(0.4, 0, false);
        }
      }
    };
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    vision.start();
    CommandScheduler.getInstance().schedule(//
        Commands.sequence(
            driveForTime(2.7),
            turnForTime(0.7),
            seekAprilTagAhead(1)//
                .raceWith(new WaitCommand(5)),
            new WaitCommand(1),
            coralYeeter(),
            park()
        //
        ));

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
    vision.stop();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  double totalError = 0;

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    /// System.out.println(vision.targetPosition());

    double forwardSpeed = -m_controller.getLeftY();

    // Left is positive
    double rotateSpeed = -m_controller.getRightX();

    // System.out.println(forwardSpeed + " " + rotateSpeed);

    /*
     * double gyroRotate = gyro.getRawGyroZ() / 250.0;
     * double error = gyroRotate - rotateSpeed;
     * 
     * totalError = totalError + error;
     * 
     * if (error > 0)
     * System.out.println(error + "," + rotateSpeed + "," + gyroRotate);
     * 
     * m_robotDrive.setDeadband(0);
     * 
     * m_robotDrive.arcadeDrive(forwardSpeed, rotateSpeed - totalError * .1, false);
     */
    m_robotDrive.setDeadband(RobotDriveBase.kDefaultDeadband);
    m_robotDrive.arcadeDrive(forwardSpeed, rotateSpeed);

    if (m_controller.getRightBumperButton()) {
      // Eject coral into level 1
      m_coralFeeder.set(-0.2);
    } else if (m_controller.getLeftBumperButton()) {
      // TEST eject coral to level 2?
      m_coralFeeder.set(0.5);
    } else {
      // stop
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    m_robotDrive.setDeadband(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putBoolean("DB/LED 0", gyro.isConnected());
    SmartDashboard.putString("DB/String 0", gyro.getYaw() + "");

    if (m_controller.getPOV() == 0) {
      // m_robotDrive.arcadeDrive(0.6, 0, false);
    } else if (m_controller.getYButton()) {
      m_robotDrive.setDeadband(0);
      if (vision.isTargetInSight()) {
        System.out.println(vision.targetPosition());
        double targetOffset = -vision.targetPosition();
        double turn = 0;
        if (targetOffset > .3) {
          turn = .5;
        } else if (targetOffset < -.3) {
          turn = -.5;
        } else if (targetOffset > .1) {
          turn = .3;
        } else if (targetOffset < -.1) {
          turn = -.3;
        }
        // turn = clamp(Math.sqrt(turn), -.9, .9);
        m_robotDrive.arcadeDrive(0.5, turn, false);
      } else {
        m_robotDrive.arcadeDrive(0.4, 0, false);
      }
    } else {
      teleopPeriodic();
    }
  }

  @Override
  public void testExit() {
    vision.stop();
  }

}
