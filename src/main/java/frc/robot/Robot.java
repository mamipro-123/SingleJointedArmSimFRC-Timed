package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Robot extends TimedRobot {

  private int pressedButton;
  private int topSetpoint, bottomSetpoint;

  // The arm gearbox represents a gearbox containing two Falcon500 motors.
  private final DCMotor mArmFalcon = DCMotor.getFalcon500(2);
  private final ProfiledPIDController topController = new ProfiledPIDController(Constants.ArmConstants.kArmKp,
      Constants.ArmConstants.kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController bottomController = new ProfiledPIDController(Constants.ArmConstants.kArmKp,
      Constants.ArmConstants.kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final Encoder topEncoder = new Encoder(Constants.ArmConstants.kEncoderAChannel,
      Constants.ArmConstants.kEncoderBChannel);
  private final Encoder bottomEncoder = new Encoder(Constants.ArmConstants.kEncoderAChannel + 2,
      Constants.ArmConstants.kEncoderBChannel + 2);

  private final WPI_TalonFX topMotor = new WPI_TalonFX(Constants.ArmConstants.kMotorPort);
  private final WPI_TalonFX bottomMotor = new WPI_TalonFX(Constants.ArmConstants.kMotorPort + 1);
  private final Joystick m_joystick = new Joystick(Constants.ArmConstants.kJoystickPort);

  private final EncoderSim m_topEncoderSim = new EncoderSim(topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(bottomEncoder);
  
  //SingleJointedArmSim
  private final SingleJointedArmSim m_arm_topSim = new SingleJointedArmSim(
      mArmFalcon,
      Constants.ArmConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(Constants.ArmConstants.m_arm_topLength, 
      Constants.ArmConstants.m_arm_topMass),                                     
     Constants.ArmConstants.m_arm_topLength, 
      Units.degreesToRadians(Constants.ArmConstants.m_arm_top_min_angle), 
      Units.degreesToRadians(Constants.ArmConstants.m_arm_top_max_angle), 
      false,
      VecBuilder.fill(Constants.ArmConstants.kArmEncoderDistPerPulse));

  private final SingleJointedArmSim m_arm_bottomSim = new SingleJointedArmSim(
      mArmFalcon,
      Constants.ArmConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(Constants.ArmConstants.m_arm_bottomLength,
          Constants.ArmConstants.m_arm_bottomMass),
      Constants.ArmConstants.m_arm_bottomLength,
      Units.degreesToRadians(Constants.ArmConstants.m_arm_bottom_min_angle),
      Units.degreesToRadians(Constants.ArmConstants.m_arm_bottom_max_angle),
      false,
      VecBuilder.fill(Constants.ArmConstants.kArmEncoderDistPerPulse));

  //Mechanism2d
  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 45, 21.75);
  private final MechanismLigament2d m_arm_tower = m_armPivot
      .append(new MechanismLigament2d("ArmTower",
          18, -90,
          10,
          new Color8Bit(Color.kDarkGray)));
  private final MechanismLigament2d m_arm_bottom = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Bottom",
          20, -90,
          10,
          new Color8Bit(Color.kDarkTurquoise)));
  private final MechanismLigament2d m_arm_top = m_arm_bottom.append(
      new MechanismLigament2d(
          "Arm Top",
          28.5,
          Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
          10,
          new Color8Bit(Color.kDarkSeaGreen)));

          
  @Override
  public void robotInit() {
    topEncoder.setDistancePerPulse(Constants.ArmConstants.kArmEncoderDistPerPulse);
    bottomEncoder.setDistancePerPulse(Constants.ArmConstants.kArmEncoderDistPerPulse);
    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  
  @Override
  public void teleopPeriodic() {

    double pidOutputTop = topController.calculate(topEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(
        SmartDashboard.getNumber("Setpoint top (degrees)", 0) - MathUtil.clamp(
            SmartDashboard.getNumber("Setpoint bottom (degrees)", 150), Constants.ArmConstants.m_arm_bottom_min_angle,
            Constants.ArmConstants.m_arm_bottom_max_angle),
        Constants.ArmConstants.m_arm_top_min_angle, Constants.ArmConstants.m_arm_top_max_angle)));
    topMotor.setVoltage(pidOutputTop);
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            Constants.ArmConstants.m_arm_bottom_min_angle, Constants.ArmConstants.m_arm_bottom_max_angle)));
    bottomMotor.setVoltage(pidOutputBottom);


    switch (pressedButton) {
      case 1:
        topSetpoint = Constants.setPoint.positionOneTop;
        bottomSetpoint = Constants.setPoint.positionOneBottom;
        break;
      case 2:
        topSetpoint = Constants.setPoint.positionTwoTop;
        bottomSetpoint = Constants.setPoint.positionTwoBottom;
        break;
      case 3:
        topSetpoint = Constants.setPoint.positionThreeTop;
        bottomSetpoint = Constants.setPoint.positionThreeBottom;
        break;
      case 4:
        topSetpoint = Constants.setPoint.positionFourTop;
        bottomSetpoint = Constants.setPoint.positionFourBottom;
        break;

      default:
        topSetpoint = Constants.setPoint.defaultTopPosition;
        bottomSetpoint = Constants.setPoint.defaultBottomPosition;
        break;

    }

    if (m_joystick.getRawButton(1)) {
      pressedButton = 1;
    } else if (m_joystick.getRawButton(2)) {
      pressedButton = 2;

    } else if (m_joystick.getRawButton(3)) {
      pressedButton = 3;

    } else if (m_joystick.getRawButton(4)) {
      pressedButton = 4;

    }

    pidOutputTop = topController.calculate(topEncoder.getDistance(),
        Units.degreesToRadians(topSetpoint - bottomSetpoint));
    topMotor.setVoltage(pidOutputTop);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
    pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    bottomMotor.setVoltage(pidOutputBottom);

  }

  @Override
  public void disabledInit() {
    topMotor.set(0.0);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_topSim.setInput(topMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(bottomMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }

}
