package org.rivierarobotics.mathutil;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * 
 * contains swerve-specific math operations
 *
 */
public class SwerveCalculator {

    /**
     * normalize a batch of vectors so that the longest vector is capped at a
     * magnitude of limit, while the other vectors are rescaled proportionately.
     * Important for achieving desired motion when wheels would normally go over
     * max velocity
     */
    public static Vector2d[] batchNormalize(double limit, Vector2d... vecs) {
        double maxMag = 0;
        for (Vector2d v : vecs) {
            if (v.getMagnitude() > maxMag) {
                maxMag = v.getMagnitude();
            }
        }
        if (limit >= maxMag) {
            return vecs;
        }
        Vector2d[] normed = new Vector2d[vecs.length];
        for (int i = 0; i < vecs.length; i++) {
            normed[i] = vecs[i].scale(limit / maxMag);
        }
        return normed;
    }

    /**
     * 
     * @param gyroHeading
     *            - robot heading
     * @param rotVal
     *            - "power" of rotation (-1.0 - 1.0)
     * @param transVecF
     *            - Field-centered translation (joystick)
     * @param modulePos
     *            - position vector of module relative to rotation center
     * @return Vector representing wheel angle and speed
     * 
     *         Works by calculating a "rotation vector" which is perpendicular
     *         to the module's position vector and has a magnitude proportional
     *         to the desired rotation speed. This vector is added to the
     *         translation vector to calculate the drive vector.
     */
    public static Vector2d calculateDriveVector(double gyroHeading, double rotVal, Vector2d transVecF,
            Vector2d modulePos) {
        Vector2d transVecR = transVecF.rotate(gyroHeading);
        Vector2d rotVec = modulePos.normalize(rotVal);
        Vector2d driveVec = transVecR.add(rotVec);
        return driveVec;
    }

    /**
     * 
     * Calculates all the modules you could ever want.
     */
    public static Vector2d[] calculateAllModules(double gyroHeading, double rotVal, Vector2d transVecF,
            Vector2d... modulePos) {
        Vector2d[] driveVecs = new Vector2d[modulePos.length];
        for (int i = 0; i < modulePos.length; i++) {
            driveVecs[i] = calculateDriveVector(gyroHeading, rotVal, transVecF, modulePos[i]);
        }
        Vector2d[] driveVecsNormed = batchNormalize(1.0, driveVecs);
        return driveVecsNormed;
    }

}
