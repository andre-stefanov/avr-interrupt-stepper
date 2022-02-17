#include "Arduino.h"

#include "Mount.h"

Mount mount;

void setup()
{
    mount.setup();
    mount.track(true);
}

void loop()
{
    delay(500);
    mount.guide<Mount::Ra>(true, 500);
    delay(750);
    mount.guide<Mount::Ra>(false, 500);
    delay(750);
    mount.slewTo<Mount::Ra>(Angle::deg(-1.0f));
    delay(1000);
    mount.startSlewing<Mount::Ra>(true);
    delay(500);
    mount.stopSlewing<Mount::Ra>();

    do
    {
        // finish
    } while (1);
}