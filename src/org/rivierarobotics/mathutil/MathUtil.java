package org.rivierarobotics.mathutil;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Generic utility for math.
 */
public class MathUtil {

    /**
     * Limits {@code v} to be between {@code -limit} and {@code +limit}.
     * 
     * @param v
     *            - the value to limit
     * @param limit
     *            - the limit (should be positive)
     * @return a value between {@code -limit} and {@code +limit}
     */
    public static double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }

    /**
     * Returns the value closest to zero.
     * 
     * @param a
     *            - the first value
     * @param b
     *            - the second value
     * @return the value closest to zero
     */
    public static double absMin(double a, double b) {
        if (Math.abs(a) <= Math.abs(b)) {
            return a;
        } else {
            return b;
        }
    }

    /**
     * Checks if either axis of the joystick is outside of the band.
     * 
     * @param joy
     *            - the joystick to check
     * @param band
     *            - the deadband for this joystick
     * @return {@code true} if any axis is out of the band
     */
    public static boolean outOfDeadband(Joystick joy, Vector2d band) {
        if (Math.abs(joy.getX()) > band.getX() || Math.abs(joy.getY()) > band.getY()) {
            return true;
        }
        return false;
    }

    /**
     * Maps joystick output so that it is either zero if inside the deadband, or
     * between {@code 0} and {@code 1} if outside of it. It also adjusts the
     * zero point so that it starts at the edge of the deadband.
     * 
     * @param joy
     *            - the joystick
     * @param band
     *            - the deadband
     * @return the mapped axis values
     * @see #adjustDeadband(Vector2d, Vector2d)
     */
    public static Vector2d adjustDeadband(Joystick joy, Vector2d band, boolean invertThrottle, boolean invertX) {
        int xMult = 1;
        int yMult = 1;
        if(invertThrottle) {
            yMult = -1;
        }
        if(invertX) {
            xMult = -1;
        }
        return adjustDeadband(new Vector2d(joy.getX()*xMult, joy.getY()*yMult), band);
    }

    /**
     * Maps joystick output so that it is either zero if inside the deadband, or
     * between {@code 0} and {@code 1} if outside of it. It also adjusts the
     * zero point so that it starts at the edge of the deadband.
     * 
     * @param joy
     *            - the joystick output
     * @param band
     *            - the deadband
     * @return the mapped axis values
     */
    public static Vector2d adjustDeadband(Vector2d joy, Vector2d band) {
        return new Vector2d(adjustBand(joy.getX(), band.getX()), adjustBand(joy.getY(), band.getY()));
    }

    private static double adjustBand(double jVal, double min) {
        double abs = Math.abs(jVal);
        if (abs < min) {
            return 0;
        }
        double sign = Math.signum(jVal);
        return sign * map(abs, min, 1, 0, 1);
    }

    private static double map(double in, double lowIn, double highIn, double lowOut, double highOut) {
        double percentIn = (in - lowIn) / (highIn - lowIn);
        return percentIn * (highOut - lowOut) + lowOut;
    }

    /**
     * Adjusts {@code angle} so that it is between {@code 0} and {@code 2pi}.
     * 
     * @param angle
     *            - the angle to adjust, in radians
     * @return the adjusted value
     */
    public static double wrapAngleRad(double angle) {
        while (angle >= 2*Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < 0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
    
    /**
     * Adjusts {@code angle} so that it is between {@code -pi} and {@code pi}.
     * 
     * @param angle
     *            - the angle to adjust, in radians
     * @return the adjusted value
     */
    public static double boundHalfAngleRad(double angle) {
        while (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
    
    /**
     * Adjusts {@code angle} so that it is between {@code -180} and {@code 180}.
     * 
     * @param angle
     *            - the angle to adjust, in degrees
     * @return the adjusted value
     */
    public static double boundHalfAngleDeg(double angle) {
        while (angle >= 180.0) {
            angle -= 360.0;
        }
        while (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }
    
    /**
     * Adjusts {@code ticks} so that it is between {@code -ticksPerRev/2} and {@code ticksPerRev/2}.
     * 
     * @param ticks
     *            - the encoder count to adjust in native ticks
     * @param ticksPerRev
     *            - number of native encoder ticks per revolution
     * @return the adjusted value
     */
    public static int boundHalfAngleNative(int ticks, int ticksPerRev) {
        while (ticks >= ticksPerRev / 2) {
            ticks -= ticksPerRev;
        }
        while (ticks < -ticksPerRev / 2) {
            ticks += ticksPerRev;
        }
        return ticks;
    }
    

}
