package org.firstinspires.ftc.teamcode.Utilities;

class Memes {


    public final static String NEIL_SPEECH =
            " Explanation of why we have to create booleans that we write to when we get inputs instead of writing to the actual used vars ( movementX & Y) " +
                    "                                  ------------------------------------------------------------------------ " +
                    " So previously, movementX and Y were changed directly from the inputs that we get from the keyboard. This was fine, but because of how key events are " +
                    " taken as 1 input, previous inputs are NOT saved; only the current input(1) input is read. Because of this, when you hold down a key, and then press another key, the 2nd key will be " +
                    " read, but because the 1st key is no longer the one who was pressed down. To solve this normal keyboards have this thing called rollover, but we have to program " +
                    " it in since KeyAdapter sucks that way. So we just assign values to booleans and have a logic method that sorts out movementX and movementY. Kinda dumb but whatever lol." +
                    " I'm pretty sure im over complicating it since it actually doesn't really matter that much since the key that is held down after the 2nd key is released is actually read by KeyAdapter, " +
                    " but there is so much lag it annoys me. - Neil";


    public static long spedness = Long.MAX_VALUE;


}
