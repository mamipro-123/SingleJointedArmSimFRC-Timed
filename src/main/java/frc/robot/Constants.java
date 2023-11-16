// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ArmConstants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    // The P gain for the PID controller that drives this arm.
    public static final double kArmKp = 40.0;
    public static final double kArmKi = 0.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    // Simulation classes help us simulate what's going on, including gravity.
    public static final double m_armGravity = 60;
    public static final double m_arm_topMass = 10.0; // Kilograms
    public static final double m_arm_topLength = Units.inchesToMeters(38.5);
    public static final double m_arm_bottomMass = 4.0; // Kilograms
    public static final double m_arm_bottomLength = Units.inchesToMeters(27);

    public static final int m_arm_top_min_angle = -75;
    public static final int m_arm_top_max_angle = 260;
    public static final int m_arm_bottom_min_angle = 30;
    public static final int m_arm_bottom_max_angle = 150;

  }

  public static class setPoint {
    // SETPOINTS
    public static final int defaultBottomPosition = 90;
    public static final int defaultTopPosition = 260;

    public static final int positionOneBottom = 140;
    public static final int positionOneTop = 200;

    public static final int positionThreeBottom = 135;
    public static final int positionThreeTop = 160;

    public static final int positionFourBottom = 120;
    public static final int positionFourTop = 255;

    public static final int positionTwoBottom = 60;
    public static final int positionTwoTop = 195;
  }

}
