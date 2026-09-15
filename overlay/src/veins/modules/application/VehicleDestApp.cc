#include "VehicleDestApp.h"
#include "veins/modules/application/CoopAwarenessBeacon_m.h"
#include "veins/modules/application/BeaconAuth.h"

Define_Module(VehicleDestApp);

void VehicleDestApp::handleSelfMsg(cMessage* msg) {
    if (msg->getKind() == SEND_BEACON_EVT) {
        veins::CoopAwarenessBeacon* cab = new veins::CoopAwarenessBeacon();
        populateWSM(cab);
        sendDown(cab);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
    }
    else {
        veins::DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

void VehicleDestApp::populateWSM(veins::BaseFrame1609_4* wsm,
                                 veins::LAddress::L2Type rcvId, int serial) {
    veins::DemoBaseApplLayer::populateWSM(wsm, rcvId, serial);

    veins::CoopAwarenessBeacon* cab = dynamic_cast<veins::CoopAwarenessBeacon*>(wsm);
    if (!cab) return;

    // Lazily resolve my destination from my SUMO route (once)
    if (!dest_known && traciVehicle && traci) {
        std::list<std::string> route = traciVehicle->getPlannedRoadIds();
        std::string dest_edge;
        for (auto it = route.rbegin(); it != route.rend(); ++it) {
            if (!it->empty() && (*it)[0] != ':') { dest_edge = *it; break; }
        }
        if (!dest_edge.empty()) {
            std::list<veins::Coord> shape =
                traci->lane(dest_edge + "_0").getShape();
            if (!shape.empty()) {
                my_dest = shape.back();
                dest_known = true;
            }
        }
    }

    // ETSI CAM-aligned fields (stationType 5 = passengerCar)
    cab->setStationID(myId);
    cab->setStationType(5);
    cab->setGenerationTime(simTime());
    cab->setAuthTag(computeAuthTag(myId, simTime()));
    if (traciVehicle) {
        cab->setSpeedValue(traciVehicle->getSpeed());     // m/s
        cab->setHeadingValue(traciVehicle->getAngle());   // degrees
    }
    if (dest_known) {
        cab->setDestX(my_dest.x);
        cab->setDestY(my_dest.y);
    } // else defaults (-1,-1) = "destination not known yet"
}
