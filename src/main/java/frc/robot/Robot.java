// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.zip.Deflater;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  // Controller
  private final XboxController m_controller = new XboxController(0);

  DriveSubsystem drive = new DriveSubsystem();

  // Coral Motor
  private final SparkMax m_coralFeeder = new SparkMax(6, MotorType.kBrushless);

  // Navigation Aids
  private final AHRS gyro = new AHRS(NavXComType.kUSB1);
  private final Vision vision = new Vision();

  // Power
  PowerDistribution pdu = new PowerDistribution(1, ModuleType.kRev);

  /**
   * Set up the robot, this is called ONCE when the robot code starts
   */
  public Robot() {

    vision.start();

    gyro.resetDisplacement();
    gyro.zeroYaw();
    autonomousOptionSetup();

    ledSetup();

  }

  private final AddressableLED m_led = new AddressableLED(9);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(270);

  AddressableLEDBufferView rearLeftLEDs = m_ledBuffer.createView(0, 36);
  AddressableLEDBufferView sideLeftLEDs = m_ledBuffer.createView(37, 77);
  AddressableLEDBufferView frontLEDs = m_ledBuffer.createView(78, 114);
  AddressableLEDBufferView sideRightLEDs = m_ledBuffer.createView(115, 155).reversed();
  AddressableLEDBufferView rearRightLEDs = m_ledBuffer.createView(156, 189).reversed();
  AddressableLEDBufferView rearLEDs = m_ledBuffer.createView(190, 269);

  private void ledSetup() {
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data

    m_led.setLength(m_ledBuffer.getLength());

    LEDPattern.solid(Color.kBlack).applyTo(rearLeftLEDs);
    LEDPattern.solid(Color.kGreen).applyTo(sideLeftLEDs);
    LEDPattern.solid(Color.kBlack).applyTo(frontLEDs);
    LEDPattern.solid(Color.kRed).applyTo(sideRightLEDs);
    LEDPattern.solid(Color.kBlack).applyTo(rearRightLEDs);
    LEDPattern.solid(Color.kBlack).applyTo(rearLEDs);

    // Set the data

    m_led.start();
  }

  private void ledPeriodic() {
    m_led.setData(m_ledBuffer);
  }

  /**
   * This is called over and over again while the robot is running.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    ledPeriodic();

    drive.drivePeriodic();

    // Auto Information
    SmartDashboard.putString("Selected Auto:", auto_chooser.getSelected());

    // Send Gyro Information
    SmartDashboard.putNumber("Gyro", gyro.getYaw());

    // Send power information
    SmartDashboard.putNumber("Voltage", pdu.getVoltage());
    SmartDashboard.putNumber("Total Current", pdu.getTotalCurrent());

    SmartDashboard.putNumber("Drive R1 (Red) Current", pdu.getCurrent(10));
    SmartDashboard.putNumber("Drive R2 (Black) Current", pdu.getCurrent(8));
    SmartDashboard.putNumber("Drive L1 (Blue) Current", pdu.getCurrent(11));
    SmartDashboard.putNumber("Drive L2 (Yellow) Current", pdu.getCurrent(9));

    SmartDashboard.putNumber("Yeeter Motor", pdu.getCurrent(19));

    SmartDashboard.putNumber("LED Current", pdu.getCurrent(1));
  }

  /**
   * Drive for the given amount of time, at the given speed
   */
  Command driveForTime(double time, double speed) {
    return Commands.sequence(
        new RunCommand(() -> drive.arcadeDrive(speed, 0.0)).raceWith(new WaitCommand(time)),
        new InstantCommand(() -> drive.arcadeDrive(0, 0.0)));
  }

  /**
   * Clamp the value 's absolute value between the limits given
   * 
   * @param v
   * @param absMin
   * @param absMax
   * @return
   */
  double signedClamp(double v, double absMin, double absMax) {
    double sign = Math.signum(v);
    v = Math.abs(v);
    v = Math.max(Math.abs(absMin), v);
    v = Math.min(Math.abs(absMax), v);
    v = v * sign;
    return v;
  }

  Command driveDistance(double meters, double speed, double timeLimit) {
    PIDController pid = new PIDController(0.4, 0.01, 0);
    return new Command() {
      @Override
      public void initialize() {
        System.out.println("Starting to drive " + meters);
        drive.resetDistanceForward();
      }

      @Override
      public void execute() {
        double s = pid.calculate(drive.getDistanceForward(), meters);
        double sign = Math.signum(s);
        s = Math.abs(s);
        s = Math.max(0.1, s);
        s = Math.min(speed, s);
        s = s * sign;
        drive.arcadeDrive(s, 0.0);
      }

      @Override
      public boolean isFinished() {
        if (meters < 0)
          return drive.getDistanceForward() <= meters;
        else
          return drive.getDistanceForward() >= meters;
      }

      @Override
      public void end(boolean interrupted) {
        System.out.println("Done driving " + interrupted);
        drive.arcadeDrive(0.0, 0.0);
      }
    }.raceWith(new WaitCommand(timeLimit));

  }

  /**
   * Turn for the given amount of time, at the given speed
   */
  Command turnForTime(double time, double turn) {
    return Commands.sequence(
        new RunCommand(() -> drive.arcadeDrive(0, turn)).raceWith(new WaitCommand(time)),
        new InstantCommand(() -> drive.arcadeDrive(0, 0.0)));
  }

  Command dramaticWait(double duration) {
    return new Command() {
      Timer timer = new Timer();
      LEDPattern pattern = LEDPattern.progressMaskLayer(() -> timer.get() / duration);
      LEDPattern basePattern = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kBlue);

      public void initialize() {
        System.out.println("DRAMA");
        timer.start();
      };

      @Override
      public void execute() {
        basePattern.mask(pattern).applyTo(sideLeftLEDs);
        basePattern.mask(pattern).applyTo(sideRightLEDs);
      }

      @Override
      public boolean isFinished() {
        System.out.println("Drama Done");
        return timer.get() >= duration;

      }
    };
  }

  /**
   * YEET that coral
   */
  Command coralYeeter() {
    return Commands.sequence(
        new RunCommand(() -> m_coralFeeder.set(-0.2)).raceWith(new WaitCommand(1)),
        new InstantCommand(() -> m_coralFeeder.stopMotor()));
  }

  /**
   * Do nothing
   */
  Command park() {
    return new RunCommand(() -> drive.stopMotor());
  }

  /*
   * Turn the robot by a number of degrees, using the gyro
   */
  Command turnDegrees(double degrees) {
    final double MAX = 0.6;

    // gyro Positive is right
    // Gyro jumps from 180 to -180
    return new Command() {
      // boolean done = false;
      Timer done = new Timer();

      PIDController pid = new PIDController(0.03, 0.00, 0.001);

      /**
       * When this command starts it sets the control deadband to
       * zero, and resets the gyro yaw to zero.
       */
      @Override
      public void initialize() {
        gyro.zeroYaw();
        System.out.println("Turning " + degrees);
      }

      @Override
      public void execute() {
        double s = pid.calculate(gyro.getYaw(), degrees);
        s = signedClamp(s, 0.0, MAX);
        drive.arcadeDrive(0, -s);
      }

      /*
       * Is this command finished?
       */
      @Override
      public boolean isFinished() {
        if (done.isRunning()) {
          return done.get() > .25;
        } else if (Math.abs(gyro.getYaw() - degrees) < 5) {
          done.start();
        }
        return false;
      }

      /**
       * When done turn off motors and reset deadband
       */
      @Override
      public void end(boolean interrupted) {
        System.out.println("Done Turning");
        drive.stopMotor();
      }

    };
  }

  /**
   * This command drives towards an april tag.
   * 
   * It will just drive forward slowly no matter what, once it sees
   * an april tag it will start steering towards it.
   * 
   * @param tagId
   * @return
   */
  Command seekAprilTagAhead(int tagId) {

    final double SLOW_TURN = 0.3; // Speed to turn when it is almost straight ahead
    final double FAST_TURN = 0.5; // Speed to turn if it is off to the side

    final double TAG_SPEED = 0.6; // Speed to drive forward when we see it
    final double SEARCH_SPEED = 0.6; ; // Speed to drive forward when we do not see it

    return new Command() {
      @Override
      public void initialize() {
        vision.lookFor(tagId);
      }

      @Override
      public void execute() {

        if (vision.isTargetInSight()) {
          System.out.println("AprilTag " + tagId + " in sight: " + vision.targetPosition());
          double targetOffset = -vision.targetPosition();
          double turn = 0;
          if (targetOffset > .3) {
            turn = FAST_TURN;
          } else if (targetOffset < -.3) {
            turn = -FAST_TURN;
          } else if (targetOffset > .1) {
            turn = SLOW_TURN;
          } else if (targetOffset < -.1) {
            turn = -SLOW_TURN;
          }

          drive.arcadeDrive(TAG_SPEED, turn);
        } else {
          System.out.println("AprilTag not in sight. Moving forward.");
          drive.arcadeDrive(SEARCH_SPEED, 0);
        }
      }

      @Override
      public void end(boolean interrupted) {
        drive.stopMotor();
        vision.lookFor(0);
      }
    };
  }

  private static final String autoDefault = "Drive Off Line";
  private static final String autoRedRight = "Red Right (Red Reef) <-";
  private static final String autoRedMiddle = "Red Middle";
  private static final String autoRedLeft = "Red Left ->";

  private static final String autoBlueRight = "Blue Right (Blue Reef) ->";
  private static final String autoBlueMiddle = "Blue Middle";
  private static final String autoBlueLeft = "Blue Left <-";

  private final SendableChooser<String> auto_chooser = new SendableChooser<>();

  public void autonomousOptionSetup() {
    auto_chooser.setDefaultOption(autoDefault, autoDefault);
    auto_chooser.addOption(autoRedRight, autoRedRight);
    auto_chooser.addOption(autoRedMiddle, autoRedMiddle);
    auto_chooser.addOption(autoRedLeft, autoRedLeft);
    auto_chooser.addOption(autoBlueRight, autoBlueRight);
    auto_chooser.addOption(autoBlueMiddle, autoBlueMiddle);
    auto_chooser.addOption(autoBlueLeft, autoBlueLeft);
    SmartDashboard.putData("Auto choices", auto_chooser);
  }

  private Command autoLeaveLine() {
    return Commands.sequence(
        driveForTime(2, 0.5),
        park());
  }

  private Command autoMiddleCommand(int aprilTag) {
    return Commands.sequence(
        driveForTime(1, 0.5),
        seekAprilTagAhead(aprilTag)
            .raceWith(new WaitCommand(3)),
        dramaticWait(1),
        coralYeeter(), // YEET
        new WaitCommand(1),
        driveForTime(.1, -.5),
        park());
  }

  private Command autoLeftCommand(int aprilTag) {
    return twoCoralAuto(aprilTag, 1);
  }

  private Command autoRightCommand(int aprilTag) {
    return twoCoralAuto(aprilTag, -1);
  }

  private Command twoCoralAuto(int aprilTag, double LR) {
    return Commands.sequence(
        // Coral 1, drive turn yeet
        driveDistance(1.8, 0.7, 3),
        turnDegrees(45 * LR),
        // driveDistance(1, 0.5, 3),
        seekAprilTagAhead(aprilTag)
            .raceWith(new WaitCommand(1.5)),
        // dramaticWait(1),
        coralYeeter(),
        // new WaitCommand(1),
        // Back, turn back
        driveDistance(-2, 0.7, 3),
        turnDegrees(120 * LR),
        driveDistance(-4, 0.7, 3),

        // Turn butt to feeder, back up
        turnDegrees(-45 * LR),
        driveDistance(-2, 0.7, 3)
            .raceWith(new WaitCommand(2)),
        //new WaitCommand(.5),

        // Forward and yeet
        seekAprilTagAhead(6)
            .raceWith(new WaitCommand(1.5)),
        // turnDegrees(10 * LR)
        // .raceWith(new WaitCommand(1)),
        // driveDistance(4, 0.6, 3),
        // dramaticWait(1),
        coralYeeter(),
        driveDistance(-.1, 0.7, 1),

        // new WaitCommand(1),
        park());

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    final String autoName = auto_chooser.getSelected();

    final Command autoCommand = switch (autoName) {
      case autoRedRight -> autoRightCommand(11);
      case autoRedMiddle -> autoMiddleCommand(10);
      case autoRedLeft -> autoLeftCommand(9);

      case autoBlueRight -> autoRightCommand(20);
      case autoBlueMiddle -> autoMiddleCommand(21);
      case autoBlueLeft -> autoLeftCommand(22);

      default -> autoLeaveLine();
    };

    CommandScheduler.getInstance().schedule(autoCommand);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
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

    // Drive Code

    // Left trigger = slow speed
    if (m_controller.getLeftTriggerAxis() > 0.9) {
      drive.setMaxOutput(0.4);
    } else {
      drive.setMaxOutput(0.6);
    }

    // Get forward speed
    double forwardSpeed = -m_controller.getLeftY();
    // Left is positive
    double rotateSpeed = -m_controller.getLeftX();

    drive.arcadeDriveJoystick(forwardSpeed, rotateSpeed);

    // Coral YEET Teleop code
    if (m_controller.getRightBumperButton()) {
      // Eject coral into level 1
      m_coralFeeder.set(-0.25);
    } else if (m_controller.getBButton()) {
      m_coralFeeder.set(0.3);
    } else {
      // stop
      m_coralFeeder.set(0);
      m_coralFeeder.stopMotor();
    }
  }

  @Override
  public void teleopExit() {
    drive.setMaxOutput(1);
  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    if (m_controller.getXButtonPressed()) {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(turnDegrees(-45));
    }
    if (m_controller.getBButtonPressed()) {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(turnDegrees(45));
    }

    if (m_controller.getYButtonPressed()) {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(driveDistance(2, .5, 3));
    }
    if (m_controller.getAButtonPressed()) {
      CommandScheduler.getInstance().cancelAll();
      CommandScheduler.getInstance().schedule(driveDistance(-2, .5, 3));
    }
  }

  @Override
  public void testExit() {

  }

}
