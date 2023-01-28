// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

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

    public enum DriveID {
        FRONT_RIGHT(16), FRONT_LEFT(10), BACK_LEFT(12), BACK_RIGHT(15);

        private final int id;

        DriveID(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }

    public enum SpinID {
        FRONT_RIGHT(17), FRONT_LEFT(11), BACK_LEFT(13), BACK_RIGHT(14);

        private final int id;

        SpinID(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }

    // The port on the Driver Station that the joystick is connected to.
    public static int driveStickPort = 0;
    // The Driver Station joystick used for driving the robot.
    public static XboxController driverController = new XboxController(Constants.driveStickPort);
    public static Joystick driverJoystick = new Joystick(Constants.driveStickPort);

    /* Gear ratio between the spin motor and wheel rotation */
    public static double gearRatio = 26.0 + 2.0 / 3.0;

    /* Motor maximum speed */
    public static double motorMaxSpeedMetersPerSecond = 5.0;
    public static double spinMotorMaxSpeedMetersPerSecond = 100.0;
    public static double anglePIDDefaultValue = 0.7;

    /* Robot Width and Length Constants */
    public static double motorWidthApartInches = 24.5625;
    public static double motorLengthApartInches = 20.875;

    /* Offset calculations */
    public static double motorXOffsetInches = motorWidthApartInches / 2;
    public static double motorYOffsetInches = motorLengthApartInches / 2;

    /* Origin */
    public static final Translation2d origin = new Translation2d(0, 0);

    /* Motor offsets */
    public static final Translation2d frontRightOffsetMeters = new Translation2d(
            Units.inchesToMeters(motorXOffsetInches), /* X */
            -Units.inchesToMeters(motorYOffsetInches)); /* Y */
    public static final Translation2d frontLeftOffsetMeters = new Translation2d(
            -Units.inchesToMeters(motorXOffsetInches), /* X */
            -Units.inchesToMeters(motorYOffsetInches)); /* Y */
    public static final Translation2d backLeftOffsetMeters = new Translation2d(
            -Units.inchesToMeters(motorXOffsetInches), /* X */
            Units.inchesToMeters(motorYOffsetInches)); /* Y */
    public static final Translation2d backRightOffsetMeters = new Translation2d(
            Units.inchesToMeters(motorXOffsetInches), /* X */
            Units.inchesToMeters(motorYOffsetInches)); /* Y */

}
