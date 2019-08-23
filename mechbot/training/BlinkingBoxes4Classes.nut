// This script draws blinking boxes on enemy heads

// Settings
targetFPS <- 120;

boxTime <- 0.1;
refireTime <- 0.5;
boxDelay <- 1 / targetFPS;
correctionForward <- 4.5;
correctionMoving <- -0.02;

headBoxMax <- Vector(4.5, 4.5, 8.5);
headBoxMin <- Vector(-4.5, -4.5, -3);
hColor <- Vector(255, 0, 255);
hColor2 <- Vector(255, 255, 0);

bodyBoxMax <- Vector(11, 11, 9);
bodyBoxMin <- Vector(-11, -11, -65);
bColor <- Vector(0, 255, 255);
bColor2 <- Vector(0, 255, 0);

borderBoxMax <- bodyBoxMax + Vector(0.7, 0.7, 0.7);
borderBoxMin <- bodyBoxMin + Vector(-0.7, -0.7, -0.7);


// Reference to the player to get his coordinates
player <- Entities.FindByClassname(null, "player"); 

// Finds all the enemy bots ingame and store them in an array
last <- null;
bots <- [];
for(;;) {
    last = Entities.FindByClassname(last, "cs_bot");
    if (last == null)
        break;
    bots.append(last);
}
printl("Found " + bots.len() + " bots.");

// Mainly copied from https://developer.valvesoftware.com/wiki/CSGO_Vscript_Examples, 20.03.2019

// create timer, ensure it's scope is created and get the scope 
thinkTimer <- Entities.CreateByClassname("logic_timer");
thinkTimer.ValidateScriptScope();
local timerScope = thinkTimer.GetScriptScope();

// Function to be executed every tick
timerScope.DoThink <- function() {
    foreach (i, bot in bots) {
        // Raytrace between the player and the bot to ignore bots behind walls
        if (TraceLine(player.EyePosition(), bot.EyePosition(), null) != 1)
            continue;

        // Ignore dead bots, since their EyePosition is completetly wrong
        if (bot.GetHealth() == 0)
            continue;

        // Add an experimentally found value to the Eyeposition to match the head better  
        pos <- bot.EyePosition()
               + bot.GetForwardVector() * correctionForward
               + bot.GetVelocity() * correctionMoving;

        // Draws a colored box over every head so we can easily find them
        vecStr <- "Vector(" + pos.x + ", " + pos.y + ", " + pos.z + ")"
        if (bot.GetTeam() == 2) {
            code <- "DebugDrawBox(" + vecStr + ", headBoxMax, headBoxMin, hColor.x, hColor.y, hColor.z,"
                + "255, boxTime);\n"
                + "DebugDrawBox(" + vecStr + ", bodyBoxMax, bodyBoxMin, bColor.x, bColor.y, bColor.z,"
                + "255, boxTime);\n"
                + "DebugDrawBox(" + vecStr + ", borderBoxMax, borderBoxMin, 0, 0, 0,"
                + "255, boxTime);";
        } else {
            code <- "DebugDrawBox(" + vecStr + ", headBoxMax, headBoxMin, hColor2.x, hColor2.y, hColor2.z,"
                + "255, boxTime);\n"
                + "DebugDrawBox(" + vecStr + ", bodyBoxMax, bodyBoxMin, bColor2.x, bColor2.y, bColor2.z,"
                + "255, boxTime);\n"
                + "DebugDrawBox(" + vecStr + ", borderBoxMax, borderBoxMin, 0, 0, 0,"
                + "255, boxTime);"     
        }

        EntFireByHandle(thinkTimer, "RunScriptCode", code, boxDelay, null, null)
    }
};

// Run DoThink on every timer tick
thinkTimer.ConnectOutput("OnTimer", "DoThink");

// Set the frequency of the timer and start it
thinkTimer.__KeyValueFromFloat("RefireTime", refireTime);
EntFireByHandle(thinkTimer, "Enable", "", 0, null, null);

