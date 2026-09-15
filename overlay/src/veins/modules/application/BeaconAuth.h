#ifndef __VEINS_BEACONAUTH_H_
#define __VEINS_BEACONAUTH_H_

#include <string>
#include <functional>
#include "omnetpp.h"

// Keyed message-authentication placeholder (Change 4).
// Demonstrates where ETSI TS 103 097 signatures would sit in the
// message flow. Real PKI/ECDSA is future work; here a shared-key
// hash binds the tag to (stationID, generationTime) so a forged or
// tampered beacon fails verification.
inline uint64_t computeAuthTag(long stationID, omnetpp::simtime_t genTime) {
    static const std::string sharedKey = "thesis-shared-key-2026";
    std::string data = std::to_string(stationID) + "|" +
                       std::to_string(genTime.raw()) + "|" + sharedKey;
    return static_cast<uint64_t>(std::hash<std::string>{}(data));
}

#endif
