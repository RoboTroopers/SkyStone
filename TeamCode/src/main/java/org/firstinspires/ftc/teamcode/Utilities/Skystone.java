package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.*;

public class Skystone {
    
    public double ordinalPosition;
    public AllianceSides allianceSide; 
    public FieldPosition fieldPosition;
    
    
    public Skystone(AllianceSides allianceSide, double ordinalPosition) {
        
        double yInches;
        
        if (allianceSide == AllianceSides.BLUE) {
            yInches = BLUE_SKYSTONE_Y;
        } else {
            yInches = RED_SKYSTONE_Y;
        }
        
        this.ordinalPosition = ordinalPosition;
        
        FieldPosition fieldPosition = new FieldPosition(ordinalPosition*STONE_WIDTH, yInches);
        
    }
    
    
}
