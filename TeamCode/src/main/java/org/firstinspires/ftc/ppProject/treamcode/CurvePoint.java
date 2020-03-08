package org.firstinspires.ftc.ppProject.treamcode;

import org.firstinspires.ftc.ppProject.core.Point;


public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    
    
    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnAmount, double slowDownTurnRadians) {
        
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        //this.pointLength = pointLength;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.slowDownTurnRadians = slowDownTurnRadians;
        
    }
    
    
    public CurvePoint (CurvePoint thisPoint) {
        this.x =thisPoint. x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.turnSpeed = thisPoint.turnSpeed;
        this.followDistance = thisPoint.followDistance;
        this.slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        this.slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        this.pointLength = thisPoint.pointLength;
        
    }
    
    
    public Point toPoint(){
        return new Point(x, y);

    }


    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
        
    }
    
    
}
