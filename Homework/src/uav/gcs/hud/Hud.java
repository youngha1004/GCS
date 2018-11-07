package uav.gcs.hud;

public interface Hud {
    void setRoll(double roll);
    void setPitch(double pitch);
    void setYaw(double yaw);

    void setSystemStatus(String systemStatus);
    void setArmed(boolean armed);
    void setAltitude(double altitude);
    void setBattery(double voltage, double current, double level);
    void setGpsFixed(boolean gpsFixed);
    void setMode(String mode);
    void setAirSpeed(double airSpeed);
    void setGroundSpeed(double groundSpeed);
    void setInfo(String info);
    void setTime(String time);
}
