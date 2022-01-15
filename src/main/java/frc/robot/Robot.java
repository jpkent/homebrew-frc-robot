
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import java.lang.Math;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  // autonomous rotation constants
  private static final double TOLERANCE = 2.0; // How close enough is good enough
  private static final double TARGETANGLE = 90.0; // angle to turn to
  private static final double MAXROTSPEED = 0.3; // Maximum rotation speed
  private static final double MINROTSPEED = 0.01; // minimum rotational speed
  private static final double ANG2CUTSPEED = 45; // angle error at which to start slowing
  private static final double DRIFT = 10; // how far bot fishtails after power is off
  private static final double SLOPE = (MAXROTSPEED - MINROTSPEED) / ANG2CUTSPEED;
  private static final double SIGN = -Math.signum(TARGETANGLE); // get the sign of the target angle
  private static final double STOPANGLE = TARGETANGLE - DRIFT;

  // don't brown out constants
  private static final double ACCEL_INCREMENT = 0.02;
  private static final double DECEL_MULTIPLIER = 0.98;
  private static final double VOLTAGE_TRIGGER = 7.3;
  private static final double DEAD_ZONE = 0.02;

  // instantiate the motor controllers
  WPI_TalonSRX leftMotor = new WPI_TalonSRX(1);
  WPI_TalonSRX rightMotor = new WPI_TalonSRX(0);
  WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);
  WPI_TalonSRX rightFollower = new WPI_TalonSRX(2);
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  AHRS ahrs;
  PowerDistributionPanel PDP = new PowerDistributionPanel(0);
  Joystick joy = new Joystick(0);

  @Override
  public void robotInit() {
    ErrorCode error; // CTRE specific error codes

    if ((error = leftMotor.configFactoryDefault()) != ErrorCode.OK) // factory default the motors
      System.out.println("error setting leftMotor to defaults = " + error);
    if ((error = rightMotor.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting rightMotor to defaults = " + error);
    if ((error = leftFollower.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting leftFollower to defaults = " + error);
    if ((error = rightFollower.configFactoryDefault()) != ErrorCode.OK)
      System.out.println("error setting rightFollower to defaults = " + error);

    if ((error = leftMotor.configContinuousCurrentLimit(40)) != ErrorCode.OK) // set max continous current to 40 Amps
      System.out.println("error setting leftMotor current limit = " + error);
    if ((error = rightMotor.configContinuousCurrentLimit(40)) != ErrorCode.OK)
      System.out.println("error setting rightMotor current limit = " + error);
    if ((error = leftFollower.configContinuousCurrentLimit(40)) != ErrorCode.OK)
      System.out.println("error setting leftFollower current limit = " + error);
    if ((error = rightFollower.configContinuousCurrentLimit(40)) != ErrorCode.OK)
      System.out.println("error setting rightFollower current limit = " + error);

    leftMotor.setNeutralMode(NeutralMode.Brake); // set all motors to electrical braking
    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftFollower.follow(leftMotor); // attach followers to leaders
    rightFollower.follow(rightMotor);
    leftMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
    rightMotor.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    leftMotor.setSensorPhase(true); // set up encoders directions
    rightMotor.setSensorPhase(false);

    try { // attempt to instantiate the NavX2. If it throws an exception, catch it and
          // report it.
      ahrs = new AHRS(SPI.Port.kMXP); // SPI is the protocol on the MXP connector that the navigator is plugged into
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void robotPeriodic() {
  }

  boolean stopped; // class variable used only in autonomous

  @Override
  public void autonomousInit() {
    ahrs.zeroYaw(); // set current heading to zero deg. If headed 179 deg may cause problems
    stopped = false; // so we can repeat autonomous
  }

  @Override
  public void autonomousPeriodic() {
    if (!stopped) { // still turning
      double currentAngle = ahrs.getYaw();
      double error = Math.abs(currentAngle - STOPANGLE);
      if (error <= TOLERANCE) { // at target - stop turning
        stopped = true;
        drive.arcadeDrive(0.0, 0.0, false); // stop
        System.out.println("stopping - angle = " + currentAngle);
      } else { // keep turning
        double rate = (SLOPE * error) + MINROTSPEED; // rate is always positive
        if (rate > MAXROTSPEED) // clip anything above maxrate or below -maxrate
          rate = MAXROTSPEED;
        drive.arcadeDrive(0.0, SIGN * rate, false); // only rotate, don't square the input
        System.out.println("driving currentAng = " + currentAngle + "  rate = " + rate);
      }
    } else { // stopped - feed the motor safety monster
      drive.arcadeDrive(0.0, 0.0, false);
      System.out.println("stopped - angle = " + ahrs.getYaw());
    }
  }

  @Override
  public void teleopInit() {
  }

  double currentY; // class variable for this method so they don't reset
  double currentRot;

  @Override
  public void teleopPeriodic() {
    double demandY = joy.getY();
    double demandRot = joy.getZ();
    double volts = PDP.getVoltage();
    if (Math.abs(demandY) < DEAD_ZONE) // get rid of dead zone noise
      demandY = 0.0;
    if (Math.abs(demandRot) < DEAD_ZONE)
      demandRot = 0.0;
    // make sure the current and demand are on the same side of zero
    if (Math.signum(demandY) != Math.signum(currentY))
      currentY = 0.0;
    if (Math.signum(demandRot) != Math.signum(currentRot))
      currentRot = 0.0;
    // if driver is slowing, let him
    if (Math.abs(currentY) > Math.abs(demandY))
      currentY = demandY;
    if (Math.abs(currentRot) > Math.abs(demandRot))
      currentRot = demandRot;
    if (volts < VOLTAGE_TRIGGER) { // about to brown out, decrease drive
      currentY *= DECEL_MULTIPLIER; // one second to be at max of 1/3 power
      currentRot *= DECEL_MULTIPLIER;
    } else { // not in voltage trouble so ramp up speed
      if (demandY < 0.0)
        currentY -= ACCEL_INCREMENT;
      else
        currentY += ACCEL_INCREMENT;
      if (demandRot < 0.0)
        currentRot -= ACCEL_INCREMENT;
      else
        currentRot += ACCEL_INCREMENT;
    }
    System.out.println("V = " + volts + "  dY = " + demandY + "  cY = " + currentY + "  drot = " + demandRot
        + "  cRot = " + currentRot);
    drive.arcadeDrive(currentY, -0.5 * currentRot); // note that this squares the inputs
    // and arcade drive clamps the inputs to -1.0 to 1.0
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
