// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

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
    public static int driveStickPort = 0;
    public static Joystick driverStick = new Joystick(Constants.driveStickPort);

    public static double motorWidthApartInches = 24.5625;
    public static double motorLengthApartInches = 20.875;

    public static double motorXOffsetInches = motorWidthApartInches / 2;
    public static double motorYOffsetInches = motorLengthApartInches / 2;
    /* Origin */

    public static final Translation2d origin = new Translation2d(0, 0);
    /* Motor offsets */
    public static final Translation2d frontRightOffsetInches = new Translation2d(motorXOffsetInches,
            motorYOffsetInches);
    public static final Translation2d frontLeftOffsetInches = new Translation2d(-motorXOffsetInches,
            motorYOffsetInches);
    public static final Translation2d backLeftOffsetInches = new Translation2d(-motorXOffsetInches,
            -motorYOffsetInches);
    public static Translation2d backRightOffsetInches = new Translation2d(motorXOffsetInches, -motorYOffsetInches);

}
