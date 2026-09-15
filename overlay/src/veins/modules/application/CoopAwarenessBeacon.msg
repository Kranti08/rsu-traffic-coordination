import veins.modules.messages.DemoSafetyMessage;

namespace veins;

// ETSI CAM-aligned beacon (structure follows EN 302 637-2 core
// fields; stationType codes from ETSI TS 102 894-2).
// Implemented within the VEINS messaging framework.
message CoopAwarenessBeacon extends DemoSafetyMessage
{
    long stationID;               // sender identity
    int stationType;              // 5 = passengerCar, 15 = roadSideUnit
    double speedValue;            // m/s
    double headingValue;          // degrees (SUMO convention)
    simtime_t generationTime;     // CAM generationDeltaTime analogue
    uint64_t authTag;             // keyed auth tag (Change 4)

    // Application payload (traffic coordination protocol)
    double queueLength;           // RSU: zone queue
    double proposedGreen;         // RSU: green-time proposal (L3)
    double destX = -1;            // Vehicle: destination (-1 = unknown)
    double destY = -1;
}
