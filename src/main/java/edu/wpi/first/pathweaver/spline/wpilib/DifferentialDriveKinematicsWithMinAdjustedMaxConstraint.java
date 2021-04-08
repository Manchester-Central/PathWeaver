package edu.wpi.first.pathweaver.spline.wpilib;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class DifferentialDriveKinematicsWithMinAdjustedMaxConstraint extends DifferentialDriveKinematicsConstraint {
    private double m_minAdjustedMax;

    public DifferentialDriveKinematicsWithMinAdjustedMaxConstraint(DifferentialDriveKinematics kinematics, double maxVelocity, double minAdjustedMax) {
        super(kinematics, maxVelocity);
        m_minAdjustedMax = minAdjustedMax;
    }
    
    @Override
    public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
        return Math.max(super.getMaxVelocityMetersPerSecond(poseMeters, curvatureRadPerMeter, velocityMetersPerSecond), m_minAdjustedMax);
    }
}
