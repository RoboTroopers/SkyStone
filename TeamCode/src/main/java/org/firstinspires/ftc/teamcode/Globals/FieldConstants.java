package org.firstinspires.ftc.teamcode.Globals;

import org.firstinspires.ftc.teamcode.Utilities.Stone;


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
    public static final Stone STONE_B_1 = new Stone(AllianceSides.BLUE, 1);
    public static final Stone STONE_B_2 = new Stone(AllianceSides.BLUE, 2);
    public static final Stone STONE_B_3 = new Stone(AllianceSides.BLUE, 3);
    public static final Stone STONE_B_4 = new Stone(AllianceSides.BLUE, 4);
    public static final Stone STONE_B_5 = new Stone(AllianceSides.BLUE, 5);
    public static final Stone STONE_B_6 = new Stone(AllianceSides.BLUE, 6);
    
    public static final Stone STONE_R_1 = new Stone(AllianceSides.RED, 1);
    public static final Stone STONE_R_2 = new Stone(AllianceSides.RED, 2);
    public static final Stone STONE_R_3 = new Stone(AllianceSides.RED, 3);
    public static final Stone STONE_R_4 = new Stone(AllianceSides.RED, 4);
    public static final Stone STONE_R_5 = new Stone(AllianceSides.RED, 5);
    public static final Stone STONE_R_6 = new Stone(AllianceSides.RED, 6);
    
    
    // Gets the instance of the other skystone, as skystones are always 3 stones apart 
    public Stone getOtherSkystone (Stone ascertainedSkystone, AllianceSides allianceSide) {
        Stone otherSkystone;
        if (allianceSide == AllianceSides.BLUE) {
            otherSkystone = STONE_B_2; // Defaults to B2 in case any random skystone instance used
            if (ascertainedSkystone == STONE_B_1)
                otherSkystone = STONE_B_4;
            else if (ascertainedSkystone == STONE_B_2)
                otherSkystone = STONE_B_5;
            else if (ascertainedSkystone == STONE_B_3)
                otherSkystone = STONE_B_6;
            else if (ascertainedSkystone == STONE_B_4)
                otherSkystone = STONE_B_1;
            else if (ascertainedSkystone == STONE_B_5)
                otherSkystone = STONE_B_2;
            else if (ascertainedSkystone == STONE_B_6)
                otherSkystone = STONE_B_3;
        } else {
            otherSkystone = STONE_R_2;
            if (ascertainedSkystone == STONE_R_1)
                otherSkystone = STONE_R_4;
            else if (ascertainedSkystone == STONE_R_2)
                otherSkystone = STONE_R_5;
            else if (ascertainedSkystone == STONE_R_3)
                otherSkystone = STONE_R_6;
            else if (ascertainedSkystone == STONE_R_4)
                otherSkystone = STONE_R_1;
            else if (ascertainedSkystone == STONE_R_5)
                otherSkystone = STONE_R_2;
            else if (ascertainedSkystone == STONE_R_6)
                otherSkystone = STONE_R_3;
        }
        return otherSkystone;
    }


}
