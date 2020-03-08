package org.firstinspires.ftc.ppProject.treamcode;

import org.firstinspires.ftc.ppProject.core.Point;

import java.util.ArrayList;
import static java.lang.Math.*;


public class MathFunctions {
    
    
     //Keeps angle within -180 to 180 degrees while preserving angle measure
    public static double angleWrap(double rad) {
        while (rad < -PI)
            rad += 2*PI;
        
        while (rad > PI)
            rad -= 2*PI;
        
        return rad;
        
    }
    
    
    
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoint1, Point linePoint2) {
        
        // If slope is pretty much horizontal or vertical, just don't
        if (abs(linePoint1.y - linePoint2.y) < 0.003)
            linePoint1.y = linePoint2.y + 0.003;

        if (abs(linePoint1.x - linePoint2.x) < 0.003)
            linePoint1.y = linePoint2.x + 0.003;
        
        // Rise over run to calculate slope
        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y = circleCenter.y;

        double quadraticB = (2.0 * m1 * y1 - (2.0 * pow(m1, 2) * x1));
        double quadraticC = ((pow(m1, 2) * pow(x1, 2)) - (2.0*y1*m1*x1) + pow(y1, 2) - pow(radius, 2));
        
        ArrayList<Point> allPoints = new ArrayList<>();
        
        
        try {
            double xRoot1 = (-quadraticB+sqrt(pow(quadraticB, 2) - 4*quadraticA*quadraticC))/(2*quadraticA);
            double yRoot1 = (m1 * (xRoot1 - x1)) + y1;

            // Shift back to circle center
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
    
    
            if (xRoot1 > minX && xRoot1 < maxX)
                allPoints.add(new Point(xRoot1, yRoot1));
                
            
            
            double xRoot2 = (-quadraticB-sqrt(pow(quadraticB, 2) - 4*quadraticA*quadraticC))/(2*quadraticA);
            double yRoot2 = (m1 * (xRoot2 - x1)) + y1;
    
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
                
            }
            
            //double xRoot2 = (quadraticB+sqrt(pow(quadraticB, 2) - 4*quadraticA*quadraticC))/(2*quadraticA);

        } catch(Exception e) {
            // No roots (No intersection)
            
        }
        
        return allPoints;
    }
    

}
