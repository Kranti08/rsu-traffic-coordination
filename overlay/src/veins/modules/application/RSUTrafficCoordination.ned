package org.car2x.veins.modules.application;

import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple RSUTrafficCoordination extends DemoBaseApplLayer
{
    parameters:
        @class(RSUTrafficCoordination);
        double failureTime @unit(s) = default(-1s);
        double byzantineTime @unit(s) = default(-1s);
        double biasTime @unit(s) = default(-1s);
        double driftAttackTime @unit(s) = default(-1s);
        bool forgeAuth = default(false);
        double biasTarget = default(-1);           // if >=0, liar proposes this fixed value
        bool enableCaching = default(false);
        bool enableValidation = default(true);   // Layer 1B on/off
        double maxQueueChangePerSec = default(20); // Layer 1B plausibility bound (lights-network: normal jumps ~13, attack ~987)
        double driftThreshold = default(35);       // Weakness #2: sustained-drift bound (data-driven: 0 false positives across all scenarios)
        int driftPersistenceK = default(3);        // consecutive steps to flag
        double xcheckThreshold = default(15);      // beacon vs sumo disagreement
        bool enableVoting = default(true);       // Layer 3 on/off
        bool enableDegradation = default(true);  // Layer 6 on/off
        bool enableRerouting = default(true);    // Layer 4 on/off
        double reroutePenalty = default(30.0);    // failed-zone road cost for rerouting
        @display("i=block/app2");
}
