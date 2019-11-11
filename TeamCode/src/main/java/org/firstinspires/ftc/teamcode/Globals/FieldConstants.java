package org.firstinspires.ftc.teamcode.Globals;

import org.firstinspires.ftc.teamcode.Utilities.Skystone;


public class FieldConstants {
    
    //All linear measurements use inches
    
    public static final double TILE_LENGTH = 24;
    public static final double TILES_PER_SIDE = 6;
    
    public static final double FIELD_LENGTH = TILE_LENGTH*TILES_PER_SIDE;

    
    public double tileToInches(double tile){
        return tile*TILE_LENGTH;
    }
    
    
    public static final double STONE_WIDTH = 8.0;
    public static final double STONE_DEPTH = 4;
    
    
    public enum AllianceSides {
        RED,
        BLUE
        
    }
    
    
    // Specific key field positions
    public static final double BRIDGE_X = FIELD_LENGTH/2;
    public static final double BLUE_SKYSTONE_Y = FIELD_LENGTH-(TILE_LENGTH*2);
    public static final double RED_SKYSTONE_Y = TILE_LENGTH*2;
    
    
    // All possible positions for skystones, not all actually are skystones
    public static final Skystone SKYSTONE_B1 = new Skystone(AllianceSides.BLUE, 1);
    public static final Skystone SKYSTONE_B2 = new Skystone(AllianceSides.BLUE, 2);
    public static final Skystone SKYSTONE_B3 = new Skystone(AllianceSides.BLUE, 3);
    public static final Skystone SKYSTONE_B4 = new Skystone(AllianceSides.BLUE, 4);
    public static final Skystone SKYSTONE_B5 = new Skystone(AllianceSides.BLUE, 5);
    public static final Skystone SKYSTONE_B6 = new Skystone(AllianceSides.BLUE, 6);
    
    public static final Skystone SKYSTONE_R1 = new Skystone(AllianceSides.RED, 1);
    public static final Skystone SKYSTONE_R2 = new Skystone(AllianceSides.RED, 2);
    public static final Skystone SKYSTONE_R3 = new Skystone(AllianceSides.RED, 3);
    public static final Skystone SKYSTONE_R4 = new Skystone(AllianceSides.RED, 4);
    public static final Skystone SKYSTONE_R5 = new Skystone(AllianceSides.RED, 5);
    public static final Skystone SKYSTONE_R6 = new Skystone(AllianceSides.RED, 6);
    
    
    // Gets the instance of the other skystone, as skystones are always 3 stones apart 
    public Skystone getOtherSkystone (Skystone ascertainedSkystone, AllianceSides allianceSide) {
        Skystone otherSkystone;
        if (allianceSide == AllianceSides.BLUE) {
            otherSkystone = SKYSTONE_B2; // Defaults to B2 in case any random skystone instance used
            if (ascertainedSkystone == SKYSTONE_B1)
                otherSkystone = SKYSTONE_B4;
            else if (ascertainedSkystone == SKYSTONE_B2)
                otherSkystone = SKYSTONE_B5;
            else if (ascertainedSkystone == SKYSTONE_B3)
                otherSkystone = SKYSTONE_B6;
            else if (ascertainedSkystone == SKYSTONE_B4)
                otherSkystone = SKYSTONE_B1;
            else if (ascertainedSkystone == SKYSTONE_B5)
                otherSkystone = SKYSTONE_B2;
            else if (ascertainedSkystone == SKYSTONE_B6)
                otherSkystone = SKYSTONE_B3;
        } else {
            otherSkystone = SKYSTONE_R2;
            if (ascertainedSkystone == SKYSTONE_R1)
                otherSkystone = SKYSTONE_R4;
            else if (ascertainedSkystone == SKYSTONE_R2)
                otherSkystone = SKYSTONE_R5;
            else if (ascertainedSkystone == SKYSTONE_R3)
                otherSkystone = SKYSTONE_R6;
            else if (ascertainedSkystone == SKYSTONE_R4)
                otherSkystone = SKYSTONE_R1;
            else if (ascertainedSkystone == SKYSTONE_R5)
                otherSkystone = SKYSTONE_R2;
            else if (ascertainedSkystone == SKYSTONE_R6)
                otherSkystone = SKYSTONE_R3;
        }
        return otherSkystone;
    }


}
