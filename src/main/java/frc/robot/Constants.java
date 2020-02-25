/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1_id = 11;
        public static final int kLeftMotor2_id = 12;
        public static final int kRightMotor1_id = 13;
        public static final int kRightMotor2_id = 14;

        public static final double kDriveGearRatio = 10.71;
        public static final double kWheelDiameterMeters = 0.1524;

        public static final int kNeoBuiltinCPR = 42;
        //Convert the Rotations reported by built-in neo encoder to Meters as needed by the 
        // WPILIB's Ramsette controller
        public static final double kNeoPositionConversionFactor = (1/kDriveGearRatio) * Math.PI * kWheelDiameterMeters;
        // Convert RPM reported by built-in neo encoder to Meters/sec as needed by the 
        // WPILIB's Ramsette controller
        public static final double kNeoVelocityConversionFactor = (1/60) * (1/kDriveGearRatio) * Math.PI * kWheelDiameterMeters;
        // public static final int kEncoderCPR = kNeoBuiltinCPR;
        // public static final double kWheelDiameterInches = 6;
        // public static final double kEncoderDistancePerPulse =
        //     // Assumes the encoders are NOT directly mounted on the wheel shafts
        //     (kWheelDiameterInches * Math.PI) / (double) (kEncoderCPR * kDriveGearRatio);

        public static final double kTrackwidthMeters = 0.6713;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int[] kLeftEncoderPorts = new int[]{0, 1};
        public static final int[] kRightEncoderPorts = new int[]{2, 3};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;
        
        public static final double kLeftEncoderPulsesPerRev = kNeoBuiltinCPR * 4 * kDriveGearRatio;
        public static final double kRightEncoderPulsesPerRev = kNeoBuiltinCPR * 4 * kDriveGearRatio;
        // public static final double kLeftEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        // public static final double kRightEncoderPulsesPerRev = 8192; // Rev Throughbore encoder
        public static final double kLeftMetersPerPulse = Math.PI * kWheelDiameterMeters / kLeftEncoderPulsesPerRev;
        public static final double kRightMetersPerPulse = Math.PI * kWheelDiameterMeters / kRightEncoderPulsesPerRev;
    
        public static final int kNeoEncoderPulsesPerRev = kNeoBuiltinCPR * 4;
        public static final double kNeoEncoderMetersPerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) (kNeoEncoderPulsesPerRev * kDriveGearRatio);
    
        public static final boolean kGyroReversed = true;
    
        // kS, kV, kA values from frc-characterization tool
        public static final double ksVolts = 0.15;
        public static final double kvVoltSecondsPerMeter = 2.71;
        public static final double kaVoltSecondsSquaredPerMeter = 0.367;
        // public static final double kTrackwidthMeters = 0.6713;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.34; // value from characterization tool is 13.4!
        public static final double kD = 0.0;
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
		public static int kOtherControllerPort = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class AutoPathsConstants {
        public static final int kPos3Path1_numSegments = 4;
        public static final String[] kPos3Path1 = 
            new String[] {"paths/Auto_pos3_path1_segment1.wpilib.json", "paths/Auto_pos3_path1_segment2.wpilib.json",
                "paths/Auto_pos3_path1_segment3.wpilib.json", "paths/Auto_pos3_path1_segment4.wpilib.json"};
    }
}
