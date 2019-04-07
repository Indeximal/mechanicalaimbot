
// Reference to the player to get his coordinates
player <- Entities.FindByClassname(null, "player"); 

// Finds all the enemy bots ingame and store them in an array
last <- null;
bots <- [];

for(;;) {
    last = Entities.FindByClassname(last, "cs_bot");
    if (last == null)
        break;
    if (last.GetTeam() != player.GetTeam())
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
        pos <- bot.EyePosition() + bot.GetForwardVector() * 6;

        // Draws a colored box over every head so we can easily find them
        DebugDrawBox(pos, Vector(2, 2, 2), Vector(-2, -2, -2), 255, 0, 255, 255, 0.03);
    }
};

// Run DoThink on every timer tick
thinkTimer.ConnectOutput("OnTimer", "DoThink");

// Set the frequency of the timer and start it
thinkTimer.__KeyValueFromFloat("RefireTime", 0.06);
EntFireByHandle(thinkTimer, "Enable", "", 0, null, null);

