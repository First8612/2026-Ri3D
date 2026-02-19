package frc.robot.utils;

public class AimData {
    private double distance;
    private double hoodPos;
    private double shootSpeed;

    public AimData(double distance, double hoodPos, double shootSpeed, double hangTime) {
        this.distance = distance;
        this.hoodPos = hoodPos;
        this.shootSpeed = shootSpeed;
    }

    public double getDist() {
        return distance;
    }
    
    public double getHood() {
        return hoodPos;
    }

    public double getSpeed() {
        return shootSpeed;
    }
    public AimData lerp(AimData aimdata2, double amount) {
        return new AimData(this.distance + amount * (aimdata2.distance - this.distance), 
                            this.hoodPos + amount * (aimdata2.hoodPos - this.hoodPos), 
                            this.shootSpeed + amount * (aimdata2.shootSpeed - this.shootSpeed), 
                            1);
    }
}