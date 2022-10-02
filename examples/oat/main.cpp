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
    mount.guide<Mount::Ra>(false, 500);
    mount.guide<Mount::Dec>(false, 500);

    delay(750);
    mount.guide<Mount::Ra>(true, 500);
    mount.guide<Mount::Dec>(true, 500);

    delay(750);
    mount.slewTo<Mount::Ra>(Angle::deg(-1.0f));
    mount.slewTo<Mount::Dec>(Angle::deg(-1.0f));

    delay(1000);
    mount.startSlewing<Mount::Ra>(true);
    mount.startSlewing<Mount::Dec>(true);

    delay(500);
    mount.stopSlewing<Mount::Ra>();
    mount.stopSlewing<Mount::Dec>();

    do
    {
        // finish
    } while (1);
}