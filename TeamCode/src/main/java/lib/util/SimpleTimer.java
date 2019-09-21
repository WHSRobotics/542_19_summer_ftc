package lib.util;

/**
 * Simplified Timer Class
 *
 * @see Timer
 */

public class SimpleTimer
{

    public double expirationTime; //in seconds

    public SimpleTimer() {

    }

    public void set(double timerDuration) {
        double currentTime = System.currentTimeMillis() / 1000.0; //time in seconds
        expirationTime = currentTime + timerDuration;
    }

    public boolean isExpired() {
        double currentTime = System.currentTimeMillis() / 1000.0; //time in seconds
        return (currentTime > expirationTime);
    }

}
