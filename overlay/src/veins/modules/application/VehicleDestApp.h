#ifndef __VEINS_VEHICLEDESTAPP_H_
#define __VEINS_VEHICLEDESTAPP_H_

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

class VehicleDestApp : public veins::DemoBaseApplLayer {
protected:
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void populateWSM(veins::BaseFrame1609_4* wsm,
                             veins::LAddress::L2Type rcvId = veins::LAddress::L2BROADCAST(),
                             int serial = 0) override;
private:
    bool dest_known = false;
    veins::Coord my_dest;
};

#endif
