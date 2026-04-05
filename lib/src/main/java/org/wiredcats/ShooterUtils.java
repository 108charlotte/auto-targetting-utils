
package org.wiredcats;

import org.wiredcats.TargetCalculations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShooterUtils {
    private final TargetCalculations targetCalculations;

    public ShooterUtils(TargetCalculations targetCalculations) {
        this.targetCalculations = targetCalculations;
    }

    private Translation3d getRobotVector(Pose2d robotPose, Translation3d fieldVelocity) {
        return new Translation3d(fieldVelocity.getX(), fieldVelocity.getY(), 0.0);
    }

    private double calculateTurretAngle(Pose2d robotPose, Translation3d shooterSpeedVector) {
        double fieldRelative = getTurretAngleInFieldFrame(shooterSpeedVector);  
        double robotHeadingAngle = robotPose.getRotation().getRadians(); 
        return fieldRelative - robotHeadingAngle; 
    }

    public Translation3d getVectorFromRobotToTarget(Pose2d robotPose, Pose2d target, double shooterHeightFromGround) {
        Translation3d ballPositionVector = new Translation3d(robotPose.getX(), robotPose.getY(), shooterHeightFromGround);
        Translation3d targetGoalPositionVector = new Translation3d(target.getX(), target.getY(), targetCalculations.getTargetHeight(target));
        Translation3d vectorFromRobotToTarget = targetGoalPositionVector.minus(ballPositionVector);
        return vectorFromRobotToTarget; 
    }

    private double getTurretAngleInFieldFrame(Translation3d shooterSpeedVector) {
        return Math.atan2(shooterSpeedVector.getY(), shooterSpeedVector.getX());
    }

    /* returns a double[] which looks like this: 
     * result[0] = pitch angle used in calculations (determined from TargetCalculations)
     * result[1] = calculated initial necessary speed to shoot ball to target accounting for velocity
     * result[2] = calculated horizontal angle necessary to shoot ball to target accounting for velocity
     * result[3] = calculated angle for turret
     * result[4] = calculated flight time used for scaling speed to calculate compensation
     */
    public double[] calculationsForProjectileLaunch(Pose2d robotPose, Translation2d fieldVelocity, double shooterHeightFromGround) {
        double[] toReturn = new double[4];

        double pitchAngle = targetCalculations.getAngle(robotPose);
        Pose2d target = targetCalculations.getTarget(robotPose);

        Translation3d robotVelocity3d = getRobotVector(robotPose, fieldVelocity); // m/s
        Translation3d vectorFromRobotToTarget = getVectorFromRobotToTarget(robotPose, target, shooterHeightFromGround); // meters
        double[] times = calcTimeForBallToHitGoal(pitchAngle, robotVelocity3d.getX(), robotVelocity3d.getY(), vectorFromRobotToTarget.getX(), vectorFromRobotToTarget.getY(), vectorFromRobotToTarget.getZ());

        if (times.length == 0) {
            // set defaults if nothing found (could be impossible shot)
            toReturn[0] = pitchAngle;
            toReturn[1] = 0.0;
            toReturn[2] = 0.0; 
            toReturn[3] = 0.0; 
            toReturn[4] = 0.0; 

            return toReturn;
        }

        // use first time for calculations
        double time = times[0];
        Translation3d robotDisplacementTime = new Translation3d(robotVelocity3d.getX() * time, robotVelocity3d.getY() * time, robotVelocity3d.getZ() * time);

        Translation3d shooterSpeedVector = vectorFromRobotToTarget.minus(robotDisplacementTime); 

        double dx = shooterSpeedVector.getX();
        double dy = shooterSpeedVector.getY();

        double horizontalDist = Math.sqrt(dx * dx + dy * dy);
        double speed = horizontalDist / (time * Math.cos(pitchAngle));
        double turretAngle = calculateTurretAngle(robotPose, shooterSpeedVector);

        toReturn[0] = pitchAngle;
        toReturn[1] = speed;
        toReturn[2] = getTurretAngleInFieldFrame(shooterSpeedVector);
        toReturn[3] = turretAngle;
        toReturn[4] = time; 

        /* 
         * example of how to call this with ball simulator: 
         *  projectile.throwBall(
         *      new Pose3d(robotPose.getX(), robotPose.getY(), shooterHeightFromGround, new Rotation3d()),
         *      new Rotation3d(0, pitchAngle, turretAngleInFieldFrame),
         *      speed, robotVelocity
         *  );
         */

        return toReturn; 
    }

    // Source - https://stackoverflow.com/a/37960741
    // Posted by Salix alba
    // Retrieved 2026-04-03, License - CC BY-SA 3.0

    // then refactored by copilot
    public static double[] solveRealQuarticRoots(double a, double b, double c, double d, double e) {
        double s1 = 2 * c * c * c - 9 * b * c * d + 27 * (a * d * d + b * b * e) - 72 * a * c * e;
        double q1 = c * c - 3 * b * d + 12 * a * e;
        double discrim1 = -4 * q1 * q1 * q1 + s1 * s1;
        if (discrim1 > 0) {
        double s2 = s1 + Math.sqrt(discrim1);
        double q2 = Math.cbrt(s2 / 2);
        double s3 = q1 / (3 * a * q2) + q2 / (3 * a);
        double discrim2 = (b * b) / (4 * a * a) - (2 * c) / (3 * a) + s3;
        if (discrim2 > 0) {
            double s4 = Math.sqrt(discrim2);
            double s5 = (b * b) / (2 * a * a) - (4 * c) / (3 * a) - s3;
            double s6 = (-(b * b * b) / (a * a * a) + (4 * b * c) / (a * a) - (8 * d) / a) / (4 * s4);
            double discrim3 = (s5 - s6);
            double discrim4 = (s5 + s6);
            // actual root values, may not be set
            double r1 = Double.NaN, r2 = Double.NaN, r3 = Double.NaN, r4 = Double.NaN;

            if (discrim3 > 0) {
            double sqrt1 = Math.sqrt(s5 - s6);
            r1 = -b / (4 * a) - s4 / 2 + sqrt1 / 2;
            r2 = -b / (4 * a) - s4 / 2 - sqrt1 / 2;
            } else if (discrim3 == 0) {
            // repeated root case
            r1 = -b / (4 * a) - s4 / 2;
            }
            if (discrim4 > 0) {
            double sqrt2 = Math.sqrt(s5 + s6);
            r3 = -b / (4 * a) + s4 / 2 + sqrt2 / 2;
            r4 = -b / (4 * a) + s4 / 2 - sqrt2 / 2;
            } else if (discrim4 == 0) {
            r3 = -b / (4 * a) + s4 / 2;
            }
            if (discrim3 > 0 && discrim4 > 0)
            return new double[] { r1, r2, r3, r4 };
            else if (discrim3 > 0 && discrim4 == 0)
            return new double[] { r1, r2, r3 };
            else if (discrim3 > 0 && discrim4 < 0)
            return new double[] { r1, r2 };
            else if (discrim3 == 0 && discrim4 > 0)
            return new double[] { r1, r3, r4 };
            else if (discrim3 == 0 && discrim4 == 0)
            return new double[] { r1, r3 };
            else if (discrim3 == 0 && discrim4 < 0)
            return new double[] { r1 };
            else if (discrim3 < 0 && discrim4 > 0)
            return new double[] { r3, r4 };
            else if (discrim3 < 0 && discrim4 == 0)
            return new double[] { r3 };
            else if (discrim3 < 0 && discrim4 < 0)
            return new double[0];
        }
        }
        return new double[0];
    }

    public Pose3d[] makeAdvantageScopeLine(Translation3d vector, Pose2d robotPose, double heightOffset) {
        Pose3d[] final_array = new Pose3d[9]; 
        double robotX = robotPose.getX(); 
        double robotY = robotPose.getY(); 
        for (int i = 0; i < 9; i++) {
            Pose3d to_add = new Pose3d(vector.getX()/(i+1) + robotX, vector.getY()/(i+1) + robotY, heightOffset + vector.getZ()/(i+1), new Rotation3d(0,0,0)); 
            final_array[i] = to_add; 
        }
        return final_array; 
    }

    private double[] calcTimeForBallToHitGoal(double pitchAngle, double xSpeed, double ySpeed, double xDist, double yDist, double zDist) {
        double a = -(4.9*4.9); 
        double b = 0; 
        double c = -9.8 * zDist + ((Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xSpeed * xSpeed) + (ySpeed * ySpeed))); 
        double d = -2 * (Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xSpeed * xDist) + (ySpeed * yDist)); 
        double e = -zDist + ((Math.tan(pitchAngle) * Math.tan(pitchAngle)) * ((xDist * xDist) + (yDist * yDist))); 

        // use function from stack overflow to get roots for a quartic polynomial
        double[] rawResults = solveRealQuarticRoots(a, b, c, d, e); 

        // filter roots
        java.util.List<Double> validRoots = new java.util.ArrayList<>();
        for (int i = 0; i < rawResults.length; i++) {
            double t = rawResults[i];
            // needs to exist and be positive (neg time isn't a thing)
            if (!Double.isNaN(t) && !Double.isInfinite(t) && t > 0) {
            validRoots.add(t);
            }
        }

        // convert valid values to array to return
        double[] filtered = new double[validRoots.size()];
        for (int i = 0; i < validRoots.size(); i++) {
            filtered[i] = validRoots.get(i);
        }
        return filtered;
    }
}
