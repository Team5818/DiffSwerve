package org.rivierarobotics.mathutil;

/**
 * A simple 2D vector class.
 */
public class Vector2d {

    private final double x;
    private final double y;

    /**
     * Creates a new vector from an x and y.
     * 
     * @param x
     *            - x value
     * @param y
     *            - y value
     */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the x value.
     * 
     * @return The x value.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value.
     * 
     * @return The y value
     */
    public double getY() {
        return y;
    }

    /**
     * 
     * @return length of vector
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }
    
    /**
     * return angle of vector
     */
    public double getAngle() {
        return MathUtil.wrapAngleRad(Math.atan2(y, x));
    }

    /**
     * 
     * @param other
     *            -vector to add
     * @return combination of 2 vectors
     */
    public Vector2d add(Vector2d other) {
        return new Vector2d(x + other.getX(), y + other.getY());
    }

    /**
     * 
     * @param scale
     *            -scaling factor
     * @return scales vector equally by all components
     */
    public Vector2d scale(double scale) {
        return new Vector2d(getX() * scale, getY() * scale);
    }

    /**
     * 
     * @return vector of unit length in same direction
     */
    public Vector2d normalize() {
        return scale(1.0 / getMagnitude());
    }

    /**
     * @return vector of length target in same direction
     */
    public Vector2d normalize(double target) {
        return scale(target / getMagnitude());
    }

    /**
     * @return vector rotated by ang radians
     */
    public Vector2d rotate(double ang) {
        return new Vector2d(x * Math.cos(ang) - y * Math.sin(ang), x * Math.sin(ang) + y * Math.cos(ang));
    }

    /**
     * compute dot product
     */
    public double dot(Vector2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    /**
     * 
     * @return vector reflected into 1st quadrant
     */
    public Vector2d abs() {
        return new Vector2d(Math.abs(x), Math.abs(y));
    }
    
    public Vector2d clone() {
        return new Vector2d(x,y);
    }

    @Override
    public String toString() {
        return String.format("(%s, %s)", x, y);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2d)) {
            return false;
        }
        Vector2d other = (Vector2d) obj;
        if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x)) {
            return false;
        }
        if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y)) {
            return false;
        }
        return true;
    }

    // TODO: more vector math!

}