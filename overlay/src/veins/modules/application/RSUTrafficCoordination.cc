#include "RSUTrafficCoordination.h"
#include "veins/modules/application/CoopAwarenessBeacon_m.h"
#include "veins/modules/application/BeaconAuth.h"
#include "veins/base/utils/Coord.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <chrono>

Define_Module(RSUTrafficCoordination);

RSUTrafficCoordination::RSUTrafficCoordination() {
    measurement_timer = nullptr;
    failure_timer = nullptr;
    manager = nullptr;
}

RSUTrafficCoordination::~RSUTrafficCoordination() {
    cancelAndDelete(measurement_timer);
    cancelAndDelete(failure_timer);
}

void RSUTrafficCoordination::initialize(int stage) {
    veins::DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        zone_radius = 300.0;
        queue_speed_threshold = 2.0;
        // each RSU controls the signals in its territory (closest-RSU partition)
        switch (getParentModule()->getIndex()) {
            case 0: my_lights = {"C2","C3","D2"}; break;
            case 1: my_lights = {"A3","B3","B4","C4"}; break;
            case 2: my_lights = {"D3","D4","E3"}; break;
            case 3: my_lights = {"A1","A2","B0","B1","B2","C0","C1"}; break;
            case 4: my_lights = {"D0","D1","E1","E2"}; break;
            default: break;
        }
        vehicles_in_zone = 0;
        raw_queue_length = 0;
        queue_length = 0;
        incoming_vehicles = 0;
        outgoing_vehicles = 0;
        pressure = 0.0;
        measurement_interval = 1.0;

        last_valid_queue = 0;
        max_queue_change_per_sec = par("maxQueueChangePerSec").doubleValue();
        drift_threshold = par("driftThreshold").doubleValue();
        drift_persistence_k = par("driftPersistenceK").intValue();
        xcheck_threshold = par("xcheckThreshold").doubleValue();
        byzantine_detections = 0;
        valid_measurements = 0;
        byzantine_time = par("byzantineTime").doubleValue();
        bias_time = par("biasTime").doubleValue();
        drift_attack_time = par("driftAttackTime").doubleValue();
        bias_factor = 1.4; // smart lie: +40% on the true proposal
        bias_target = par("biasTarget").doubleValue();
        bias_active = false;
        sum_bias_shift = 0.0;
        bias_shift_samples = 0;
        timing_sum_ms = 0.0;
        timing_max_ms = 0.0;
        timing_min_ms = 1e18;
        timing_samples = 0;

        enable_caching = par("enableCaching").boolValue();
        cache_hits = 0;
        cache_cross_hits = 0;
        cache_misses = 0;
        sum_staleness = 0.0;
        cache_lookup_ms_sum = 0.0;
        cache_compute_ms_sum = 0.0;
        byzantine_injected = false;

        proposed_green_time = 30.0;
        traffic_state = "light";
        l2_decisions = 0;
        sum_green_time = 0.0;
        count_light = count_moderate = count_heavy = count_congested = 0;

        voted_green_time = 30.0;
        l3_votes_collected = 0;
        l3_voting_rounds = 0;
        sum_voted_green = 0.0;

        enable_rerouting = par("enableRerouting").boolValue();
        reroute_penalty = par("reroutePenalty").doubleValue();
        rerouting_active = false;
        l4_reroute_commands_sent = 0;
        l4_vehicles_rerouted = 0;
        failed_rsu_x = 0; failed_rsu_y = 0;
        failed_rsu_known = false;
        w_dist = 1.0; w_cong = 2.0; w_heur = 0.5;
        junction_capacity = 30.0;
        l4_greedy_evaluations = 0;
        l4_actuations = 0;
        gt_network_waiting = 0.0;
        l4_dest_from_v2x = 0;
        l4_dest_inside_skipped = 0;
        l4_dest_fallback = 0;
        sum_greedy_score = 0.0;

        active_failures = 0;
        quality_level = 100;
        green_time_cap = 50.0;
        speed_guidance = 100.0;
        degradation_activations = 0;
        coordination_mode = "normal";

        last_pressure = 0.0;
        spike_threshold = 5.0;
        pressure_spikes = 0;

        heartbeat_timeout = 3.0;
        failures_detected = 0;
        first_detection_time = -1;
        false_alarm_recoveries = 0;
        i_am_failed = false;
        failure_time = par("failureTime").doubleValue();

        gt_waiting_seconds = 0.0;
        real_wait_seconds = 0.0;
        total_measurements = 0;
        sum_queue = 0.0;
        sum_pressure = 0.0;
        max_queue_seen = 0;
        my_x = 0; my_y = 0;

        measurement_timer = new cMessage("measurementTimer");
        scheduleAt(simTime() + measurement_interval, measurement_timer);

        if (failure_time >= 0) {
            failure_timer = new cMessage("failureTimer");
            scheduleAt(failure_time, failure_timer);
        }
    }
    else if (stage == 1) {
        manager = veins::TraCIScenarioManagerAccess().get();
        cModule* mob = getParentModule()->getSubmodule("mobility");
        if (mob) {
            my_x = mob->par("x").doubleValue();
            my_y = mob->par("y").doubleValue();
        }
    }
}

void RSUTrafficCoordination::populateWSM(veins::BaseFrame1609_4* wsm,
                                         veins::LAddress::L2Type rcvId,
                                         int serial) {
    veins::DemoBaseApplLayer::populateWSM(wsm, rcvId, serial);
    if (veins::CoopAwarenessBeacon* cab = dynamic_cast<veins::CoopAwarenessBeacon*>(wsm)) {
        cab->setStationID(myId);
        cab->setStationType(15);  // RSU
        cab->setGenerationTime(simTime());
        if (par("forgeAuth").boolValue())
            cab->setAuthTag(0xDEADBEEF);
        else
            cab->setAuthTag(computeAuthTag(myId, simTime()));
        cab->setQueueLength(queue_length);
        cab->setProposedGreen(proposed_green_time);
    }
}

void RSUTrafficCoordination::handleSelfMsg(cMessage* msg) {
    if (msg == failure_timer) {
        i_am_failed = true;
        EV << "[RSU] *** INJECTED FAILURE t=" << simTime() << " ***" << endl;
        return;
    }
    if (msg == measurement_timer) {
        // GROUND TRUTH: experimenter observes real traffic ALWAYS,
        // even when this RSU's protocol has failed. Never sees injections.
        gt_waiting_seconds += countRealQueue();
        gt_network_waiting += countNetworkQueue();

        if (!i_am_failed) {
            auto t_start = std::chrono::high_resolution_clock::now();
            measureTraffic();
            validateMeasurement();
            {
                double gap = std::abs((double)queue_length - (double)beacon_queue_count);
                sum_xcheck_gap += gap;
                xcheck_samples++;
                if (gap > max_xcheck_gap) max_xcheck_gap = gap;
                // sustained disagreement between own count and vehicle reports
                if (gap > xcheck_threshold) {
                    consecutive_xcheck_gap++;
                    if (consecutive_xcheck_gap >= drift_persistence_k) {
                        xcheck_rejections++;
                        EV << "[RSU] *** L1C XCHECK: sumo=" << queue_length
                           << " beacon=" << beacon_queue_count
                           << " disagree for " << consecutive_xcheck_gap
                           << " steps ***" << endl;
                    }
                } else {
                    consecutive_xcheck_gap = 0;
                }
            }
            beacon_queue_count = 0;
            checkNeighborHealth();
            applyDegradation();
            rerouteVehicles();
            computeSignalTiming();
            byzantineVoting();
            trackBackpressure();

            auto t_end = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            timing_sum_ms += ms;
            if (ms > timing_max_ms) timing_max_ms = ms;
            if (ms < timing_min_ms) timing_min_ms = ms;
            timing_samples++;

            total_measurements++;
            sum_queue += queue_length;
            sum_pressure += pressure;
            if (queue_length > max_queue_seen) max_queue_seen = queue_length;
        }

        scheduleAt(simTime() + measurement_interval, measurement_timer);
        return;
    }
    if (i_am_failed) return;  // dead RSU sends no beacons
    if (msg->getKind() == SEND_BEACON_EVT) {
        veins::CoopAwarenessBeacon* cab = new veins::CoopAwarenessBeacon();
        populateWSM(cab);
        sendDown(cab);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        return;
    }
    veins::DemoBaseApplLayer::handleSelfMsg(msg);
}

void RSUTrafficCoordination::onBSM(veins::DemoSafetyMessage* bsm) {
    if (i_am_failed) return;

    veins::CoopAwarenessBeacon* cab = dynamic_cast<veins::CoopAwarenessBeacon*>(bsm);
    if (!cab) return;  // unknown beacon type: ignore completely

    // reject stale/replayed beacons
    simtime_t age = simTime() - cab->getGenerationTime();
    if (age > 2.0) {
        replay_rejections++;
        EV_INFO << "[RSU] beacon REJECTED (stationType=" << cab->getStationType()
                << "): age=" << age << "s exceeds freshness window" << endl;
        return;
    }

    // verify auth tag
    if (cab->getAuthTag() != computeAuthTag(cab->getStationID(), cab->getGenerationTime())) {
        auth_rejections++;
        EV_INFO << "[RSU] beacon REJECTED (stationType=" << cab->getStationType()
                << "): auth tag mismatch" << endl;
        return;
    }

    // vehicle
    if (cab->getStationType() == 5) {
        EV_INFO << "[RSU] CAM(vehicle): speed=" << cab->getSpeedValue()
                << " m/s heading=" << cab->getHeadingValue()
                << " deg ts=" << cab->getGenerationTime() << endl;
        // cross-check: count self-reported slow vehicles in zone
        {
            veins::Coord vpos = bsm->getSenderPos();
            double dx = vpos.x - my_x, dy = vpos.y - my_y;
            if (std::sqrt(dx*dx + dy*dy) <= zone_radius &&
                cab->getSpeedValue() < queue_speed_threshold) {
                beacon_queue_count++;
            }
        }
        if (cab->getDestX() < 0) return;  // destination not known yet
        DestBeacon db;
        db.pos = bsm->getSenderPos();
        db.dest = veins::Coord(cab->getDestX(), cab->getDestY(), 0);
        db.t = simTime();
        dest_beacons.push_back(db);
        if (dest_beacons.size() > 2000)
            dest_beacons.erase(dest_beacons.begin(), dest_beacons.begin()+1000);
        return;
    }

    // RSU heartbeat
    if (cab->getStationType() == 15) {
        veins::Coord sp = bsm->getSenderPos();
        char key[64];
        std::snprintf(key, sizeof(key), "RSU_at_%.0f_%.0f", sp.x, sp.y);
        std::string k(key);
        // beacon from a suspected-dead rsu -> it's alive, forgive
        if (neighbor_failed[k]) {
            neighbor_failed[k] = false;
            if (active_failures > 0) active_failures--;
            false_alarm_recoveries++;
            if (active_failures == 0) rerouting_active = false;
        }
        neighbor_last_seen[k] = simTime();
        double nb_queue = cab->getQueueLength();
        double nb_green = cab->getProposedGreen();
        neighbor_queues[k] = nb_queue;
        // File the neighbor's answer on the borrowed shelf, by bucket.
        // (Bucket approximated from their queue - honest simplification.)
        if (enable_caching && nb_green >= 10.0 && nb_green <= 50.0) {
            std::string nb_bucket;
            if (nb_queue < 5)       nb_bucket = "light";
            else if (nb_queue < 15) nb_bucket = "moderate";
            else if (nb_queue < 30) nb_bucket = "heavy";
            else                    nb_bucket = "congested";
            neighbor_shared_cache[nb_bucket] = nb_green;
        }
        if (nb_green >= 10.0 && nb_green <= 50.0)
            neighbor_proposals[k] = nb_green;
        else
            neighbor_proposals[k] = 30.0;
        return;
    }

    // any other station type: ignore
}

void RSUTrafficCoordination::checkNeighborHealth() {
    for (auto const& entry : neighbor_last_seen) {
        if (!neighbor_failed[entry.first] &&
            (simTime() - entry.second) > heartbeat_timeout) {
            neighbor_failed[entry.first] = true;
            failures_detected++;
            active_failures++;
            if (first_detection_time < 0) first_detection_time = simTime();

            rerouting_active = true;

            EV << "[RSU] *** L5 FAILURE DETECTED: " << entry.first
               << " at t=" << simTime() << " ***" << endl;
        }
    }

    // rebuild list of ALL failed positions (multi-failure support)
    failed_positions.clear();
    for (auto const& nf : neighbor_failed) {
        if (nf.second) {
            double fx, fy;
            std::sscanf(nf.first.c_str(), "RSU_at_%lf_%lf", &fx, &fy);
            failed_positions.push_back(std::make_pair(fx, fy));
        }
    }
    failed_rsu_known = !failed_positions.empty();
    if (failed_rsu_known) {
        // keep legacy single vars pointing at first (dest-exception uses list below)
        failed_rsu_x = failed_positions[0].first;
        failed_rsu_y = failed_positions[0].second;
    }
}

void RSUTrafficCoordination::applyDegradation() {
    // VERSION SWITCH: Layer 6 off -> stay at full quality (Version A/B)
    if (!par("enableDegradation").boolValue()) {
        quality_level = 100; green_time_cap = 50.0;
        speed_guidance = 100.0; coordination_mode = "normal";
        return;
    }

    int prev_quality = quality_level;

    if (active_failures == 0) {
        quality_level = 100; green_time_cap = 50.0;
        speed_guidance = 100.0; coordination_mode = "normal";
    } else if (active_failures == 1) {
        quality_level = 80; green_time_cap = 40.0;
        speed_guidance = 90.0; coordination_mode = "normal";
    } else if (active_failures == 2) {
        quality_level = 60; green_time_cap = 30.0;
        speed_guidance = 70.0; coordination_mode = "normal";
    } else {
        quality_level = 40; green_time_cap = 20.0;
        speed_guidance = 60.0; coordination_mode = "time-triggered";
    }

    if (quality_level < prev_quality) {
        degradation_activations++;
        EV << "[RSU] *** L6 DEGRADATION: quality=" << quality_level
           << "% speed=" << speed_guidance << "km/h mode="
           << coordination_mode << " ***" << endl;
    }
}

void RSUTrafficCoordination::rerouteVehicles() {
    if (!enable_rerouting || !rerouting_active || !manager || !failed_rsu_known) return;

    std::vector<std::string> candidates;
    for (auto const& entry : neighbor_last_seen) {
        if (!neighbor_failed[entry.first]) candidates.push_back(entry.first);
    }
    if (candidates.empty()) return;

    veins::TraCICommandInterface* traci = manager->getCommandInterface();
    const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();

    // PASS 1: roads currently inside the failed RSU zone = roads to avoid
    std::set<std::string> avoid_roads;
    for (auto const& entry : hosts) {
        veins::TraCIMobility* mob =
            dynamic_cast<veins::TraCIMobility*>(
                entry.second->getSubmodule("veinsmobility"));
        if (!mob) continue;
        veins::Coord pos = mob->getPositionAt(simTime());
        for (auto const& fp : failed_positions) {
            double dx = pos.x - fp.first, dy = pos.y - fp.second;
            if (std::sqrt(dx*dx + dy*dy) <= 300.0) {
                avoid_roads.insert(mob->getRoadId());
                break;
            }
        }
    }

    // PASS 2: Greedy selection + ACTUATION via changeRoute
    for (auto const& entry : hosts) {
        const std::string& vid = entry.first;
        if (rerouted_vehicles.find(vid) != rerouted_vehicles.end()) continue;

        veins::TraCIMobility* mob =
            dynamic_cast<veins::TraCIMobility*>(
                entry.second->getSubmodule("veinsmobility"));
        if (!mob) continue;

        veins::Coord pos = mob->getPositionAt(simTime());
        double dist_to_failed = 1e18;
        for (auto const& fp : failed_positions) {
            double dxf = pos.x - fp.first, dyf = pos.y - fp.second;
            double d = std::sqrt(dxf*dxf + dyf*dyf);
            if (d < dist_to_failed) dist_to_failed = d;
        }

        if (dist_to_failed < 400.0 && dist_to_failed > 100.0) {
            veins::Coord dest_proxy = pos;
            bool found_v2x = false;
            double best_d = 30.0; // match beacon within 30m, last 2s
            for (auto const& db : dest_beacons) {
                if (simTime() - db.t > 2.0) continue;
                double bdx = db.pos.x - pos.x, bdy = db.pos.y - pos.y;
                double d = std::sqrt(bdx*bdx + bdy*bdy);
                if (d < best_d) { best_d = d; dest_proxy = db.dest; found_v2x = true; }
            }
            if (found_v2x) l4_dest_from_v2x++;
            else {
                l4_dest_fallback++;
                auto it = vehicle_last_pos.find(vid);
                if (it != vehicle_last_pos.end()) {
                    dest_proxy.x = pos.x + (pos.x - it->second.x) * 10.0;
                    dest_proxy.y = pos.y + (pos.y - it->second.y) * 10.0;
                }
            }

            // dest inside ANY dead zone -> car must enter, skip penalty
            double dmin = 1e18;
            for (auto const& fp : failed_positions) {
                double ddx = dest_proxy.x - fp.first;
                double ddy = dest_proxy.y - fp.second;
                double d = std::sqrt(ddx*ddx + ddy*ddy);
                if (d < dmin) dmin = d;
            }
            if (dmin <= 300.0) {
                l4_dest_inside_skipped++;
                rerouted_vehicles.insert(vid); // don't re-check every round
                continue;
            }

            std::string best_cand;
            double best_metric = 1e18;
            for (auto const& cand : candidates) {
                double cx, cy;
                std::sscanf(cand.c_str(), "RSU_at_%lf_%lf", &cx, &cy);
                double d1 = std::sqrt((pos.x-cx)*(pos.x-cx) + (pos.y-cy)*(pos.y-cy));
                double dh = std::sqrt((dest_proxy.x-cx)*(dest_proxy.x-cx) +
                                      (dest_proxy.y-cy)*(dest_proxy.y-cy));
                double nq = neighbor_queues.count(cand) ? neighbor_queues[cand] : 0.0;
                double score = w_dist*(d1/100.0) + w_cong*(nq/junction_capacity)
                             + w_heur*(dh/100.0);
                l4_greedy_evaluations++;
                sum_greedy_score += score;
                double util = (candidate_assignments[cand] + 1.0) * score;
                if (util < best_metric) { best_metric = util; best_cand = cand; }
            }

            candidate_assignments[best_cand]++;
            rerouted_vehicles.insert(vid);
            l4_vehicles_rerouted++;
            l4_reroute_commands_sent++;

            // === ACTUATION: make SUMO avoid the failed zone's roads ===
            if (traci) {
                std::string own_road = mob->getRoadId();
                for (auto const& road : avoid_roads) {
                    if (road == own_road) continue; // cannot avoid own road
                    traci->vehicle(vid).changeRoute(road, reroute_penalty);
                    l4_actuations++;
                }
            }
        }
    }
}

void RSUTrafficCoordination::measureTraffic() {
    vehicles_in_zone = 0;
    raw_queue_length = 0;
    ns_queue = 0;
    ew_queue = 0;
    std::set<std::string> vehicles_this_cycle;

    if (manager) {
        const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();
        for (auto const& entry : hosts) {
            veins::TraCIMobility* mob =
                dynamic_cast<veins::TraCIMobility*>(
                    entry.second->getSubmodule("veinsmobility"));
            if (!mob) continue;
            veins::Coord pos = mob->getPositionAt(simTime());

            double dx = pos.x - my_x, dy = pos.y - my_y;
            if (std::sqrt(dx*dx + dy*dy) <= zone_radius) {
                vehicles_in_zone++;
                vehicles_this_cycle.insert(entry.first);
                if (mob->getSpeed() < queue_speed_threshold) {
                    raw_queue_length++;
                    veins::Heading h = mob->getHeading();
                    double ang = h.getRad() * 180.0 / M_PI;
                    ang = std::fmod(ang + 360.0, 360.0);
                    bool ns = (ang < 45 || ang > 315 || (ang > 135 && ang < 225));
                    if (ns) ns_queue++; else ew_queue++;
                }
            }
            vehicle_last_pos[entry.first] = pos;
        }
    }

    if (byzantine_time >= 0 && !byzantine_injected && simTime() >= byzantine_time) {
        raw_queue_length = 999;
        byzantine_injected = true;
        EV << "[RSU] *** BYZANTINE INJECTION queue=999 t="
           << simTime() << " ***" << endl;
    }
    if (drift_attack_time >= 0 && simTime() >= drift_attack_time) {
        drift_creep += 4;
        raw_queue_length += drift_creep;
    }

    incoming_vehicles = 0;
    for (auto const& id : vehicles_this_cycle)
        if (vehicles_last_cycle.find(id) == vehicles_last_cycle.end()) incoming_vehicles++;
    outgoing_vehicles = 0;
    for (auto const& id : vehicles_last_cycle)
        if (vehicles_this_cycle.find(id) == vehicles_this_cycle.end()) outgoing_vehicles++;
    vehicles_last_cycle = vehicles_this_cycle;
}

void RSUTrafficCoordination::validateMeasurement() {
    // VERSION SWITCH: Layer 1B off -> accept raw always (Version A)
    if (!par("enableValidation").boolValue()) {
        queue_length = raw_queue_length;
        last_valid_queue = queue_length;
        valid_measurements++;
        pressure = queue_length + incoming_vehicles - outgoing_vehicles;
        return;
    }

    int change = raw_queue_length - last_valid_queue;
    if (std::abs(change) <= max_queue_change_per_sec) {
        queue_length = raw_queue_length;
        last_valid_queue = queue_length;
        valid_measurements++;
    } else {
        byzantine_detections++;
        queue_length = last_valid_queue;
        EV << "[RSU] *** L1B BYZANTINE: raw=" << raw_queue_length
           << " keeping " << last_valid_queue << " ***" << endl;
    }

    // track drift from moving average
    if (!queue_history.empty()) {
        double avg = 0.0;
        for (int v : queue_history) avg += v;
        avg /= queue_history.size();
        double drift = std::abs(queue_length - avg);
        sum_drift_observed += drift;
        drift_samples++;
        if (drift > max_drift_observed) max_drift_observed = drift;

        // sustained drift over K steps => slow poisoning
        if (drift > drift_threshold) {
            consecutive_high_drift++;
            if (consecutive_high_drift >= drift_persistence_k) {
                drift_rejections++;
                EV << "[RSU] *** L1B SUSTAINED DRIFT: drift=" << drift
                   << " for " << consecutive_high_drift
                   << " steps -> slow-poisoning suspected ***" << endl;
                // fall back to the moving-average baseline
                queue_length = (int)avg;
            }
        } else {
            consecutive_high_drift = 0;  // transient spike -> reset
        }
    }
    queue_history.push_back(queue_length);
    if (queue_history.size() > 10) queue_history.pop_front();

    pressure = queue_length + incoming_vehicles - outgoing_vehicles;
}

void RSUTrafficCoordination::computeSignalTiming() {
    if (pressure < 5)       { traffic_state = "light";     count_light++; }
    else if (pressure < 15) { traffic_state = "moderate";  count_moderate++; }
    else if (pressure < 30) { traffic_state = "heavy";     count_heavy++; }
    else                    { traffic_state = "congested"; count_congested++; }

    // Fresh computation (always done - it IS the answer when caching
    // is off, and it is the STALENESS REFERENCE when caching is on)
    double fresh_answer;
    if (coordination_mode == "time-triggered") {
        fresh_answer = green_time_cap;
    } else {
        fresh_answer = 30.0 + (pressure - 10.0);
        if (fresh_answer < 10.0) fresh_answer = 10.0;
        if (fresh_answer > green_time_cap) fresh_answer = green_time_cap;
    }

    if (!enable_caching) {
        proposed_green_time = fresh_answer;
    } else {
        // CACHING EXPERIMENT: bucket -> saved answer
        auto t0 = std::chrono::high_resolution_clock::now();
        auto found = decision_cache.find(traffic_state);
        auto t1 = std::chrono::high_resolution_clock::now();
        cache_lookup_ms_sum +=
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (found != decision_cache.end()) {
            // HIT: reuse the saved answer
            proposed_green_time = found->second;
            cache_hits++;
            // measure staleness: saved vs what fresh would have said
            sum_staleness += std::abs(proposed_green_time - fresh_answer);
        } else {
            // MISS in my own cache - can I borrow a neighbor's answer?
            auto borrowed = neighbor_shared_cache.find(traffic_state);
            if (borrowed != neighbor_shared_cache.end()) {
                // CROSS-REGION REUSE: another RSU already solved this!
                proposed_green_time = borrowed->second;
                decision_cache[traffic_state] = borrowed->second;
                cache_cross_hits++;
                cache_hits++;
                sum_staleness += std::abs(proposed_green_time - fresh_answer);
            } else {
                // True MISS: compute fresh and save
                proposed_green_time = fresh_answer;
                decision_cache[traffic_state] = fresh_answer;
                cache_misses++;
            }
        }
    }

    l2_decisions++;
    sum_green_time += proposed_green_time;

    // ===== SMART LIE INJECTION (plausible-value Byzantine test) =====
    // Unlike the queue=999 test, this value stays BELIEVABLE (within
    // 10-50s range), so Layer 1B validation will NOT catch it.
    if (bias_time >= 0 && simTime() >= bias_time) {
        bias_active = true;
        double honest_value = proposed_green_time;
        if (bias_target >= 0.0)
            proposed_green_time = bias_target;      // fixed lie for BFT test
        else
            proposed_green_time = honest_value * bias_factor;
        if (proposed_green_time > 50.0) proposed_green_time = 50.0;
        if (proposed_green_time < 10.0) proposed_green_time = 10.0;
        EV << "[RSU] *** SMART LIE: honest=" << honest_value
           << " lying=" << proposed_green_time
           << " t=" << simTime() << " ***" << endl;
    }
}

void RSUTrafficCoordination::byzantineVoting() {
    // VERSION SWITCH: Layer 3 off -> use own proposal (Version A/B)
    if (!par("enableVoting").boolValue()) {
        voted_green_time = proposed_green_time;
        l3_voting_rounds++;
        sum_voted_green += voted_green_time;
        return;
    }

    std::vector<double> proposals;
    proposals.push_back(proposed_green_time);

    for (auto const& entry : neighbor_proposals) {
        if (!neighbor_failed[entry.first]) {
            proposals.push_back(entry.second);
            l3_votes_collected++;
        }
    }

    std::sort(proposals.begin(), proposals.end());
    voted_green_time = proposals[proposals.size() / 2];

    // direction-aware max-pressure: more green to the busier axis
    if (!i_am_failed && manager) {
        veins::TraCICommandInterface* tc = manager->getCommandInterface();
        if (tc) {
            double total = ns_queue + ew_queue;
            double ns_share = total > 0 ? ns_queue / total : 0.5;
            double lo = 15.0, hi = 50.0;
            double ns_green = lo + (hi - lo) * ns_share;
            double ew_green = lo + (hi - lo) * (1.0 - ns_share);
            for (auto const& light : my_lights) {
                int ph = tc->trafficlight(light).getCurrentPhaseIndex();
                if (ph == last_phase[light]) continue;   // only act on phase change
                last_phase[light] = ph;
                std::string st = tc->trafficlight(light).getCurrentState();
                if (st.find('y') != std::string::npos) continue;
                bool ns_phase = st.substr(0, 2).find('G') != std::string::npos;
                tc->trafficlight(light).setPhaseDuration(ns_phase ? ns_green : ew_green);
                l2_light_actuations++;
            }
        }
    }

    // Measure how much a smart lie (if active anywhere in the network)
    // shifted MY final decision compared to my own honest computation
        double my_honest = 30.0 + (pressure - 10.0);
        if (my_honest < 10.0) my_honest = 10.0;
        if (my_honest > green_time_cap) my_honest = green_time_cap;
        double shift = std::abs(voted_green_time - my_honest);
        sum_bias_shift += shift;
        bias_shift_samples++;

    l3_voting_rounds++;
    sum_voted_green += voted_green_time;
}

void RSUTrafficCoordination::trackBackpressure() {
    double dp = pressure - last_pressure;
    if (dp > spike_threshold) {
        pressure_spikes++;
    }
    last_pressure = pressure;
}

// Ground-truth queue count: real vehicles only, no injection possible
int RSUTrafficCoordination::countRealQueue() {
    int q = 0;
    if (manager) {
        const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();
        for (auto const& entry : hosts) {
            veins::TraCIMobility* mob =
                dynamic_cast<veins::TraCIMobility*>(
                    entry.second->getSubmodule("veinsmobility"));
            if (!mob) continue;
            veins::Coord pos = mob->getPositionAt(simTime());
            double dx = pos.x - my_x, dy = pos.y - my_y;
            if (std::sqrt(dx*dx + dy*dy) <= zone_radius) {
                if (mob->getSpeed() < queue_speed_threshold) q++;
                real_wait_seconds += mob->getVehicleCommandInterface()->getWaitingTime();
                real_distance += mob->getVehicleCommandInterface()->getDistanceTravelled();
            }
        }
    }
    return q;
}

int RSUTrafficCoordination::countNetworkQueue() {
    int q = 0;
    if (manager) {
        const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();
        for (auto const& entry : hosts) {
            veins::TraCIMobility* mob =
                dynamic_cast<veins::TraCIMobility*>(
                    entry.second->getSubmodule("veinsmobility"));
            if (mob && mob->getSpeed() < queue_speed_threshold) q++;
        }
    }
    return q;
}

void RSUTrafficCoordination::finish() {
    if (bias_shift_samples > 0)
        recordScalar("L3_bias_avg_shift_seconds", sum_bias_shift / bias_shift_samples);
    if (enable_caching) {
        recordScalar("CACHE_hits", cache_hits);
        recordScalar("CACHE_misses", cache_misses);
        recordScalar("CACHE_cross_hits", cache_cross_hits);
        int total = cache_hits + cache_misses;
        if (total > 0)
            recordScalar("CACHE_hit_rate_percent", 100.0 * cache_hits / total);
        if (cache_hits > 0)
            recordScalar("CACHE_avg_staleness_seconds", sum_staleness / cache_hits);
    }
    if (timing_samples > 0) {
        recordScalar("TIMING_avg_ms", timing_sum_ms / timing_samples);
        recordScalar("TIMING_max_ms", timing_max_ms);
        recordScalar("TIMING_min_ms", timing_min_ms);
    }
    recordScalar("GT_network_waiting", gt_network_waiting);
    recordScalar("L0_replay_rejections", replay_rejections);
    recordScalar("L0_auth_rejections", auth_rejections);
    recordScalar("L1B_max_drift_observed", max_drift_observed);
    if (drift_samples > 0)
        recordScalar("L1B_avg_drift_observed", sum_drift_observed / drift_samples);
    recordScalar("L1B_drift_rejections", drift_rejections);
    recordScalar("L1C_max_xcheck_gap", max_xcheck_gap);
    if (xcheck_samples > 0)
        recordScalar("L1C_avg_xcheck_gap", sum_xcheck_gap / xcheck_samples);
    recordScalar("L1C_xcheck_rejections", xcheck_rejections);
    recordScalar("L4_dest_from_v2x", l4_dest_from_v2x);
    recordScalar("L4_dest_inside_skipped", l4_dest_inside_skipped);
    recordScalar("L4_dest_fallback", l4_dest_fallback);
    recordScalar("GT_waiting_seconds", gt_waiting_seconds);
    recordScalar("REAL_wait_seconds", real_wait_seconds);
    recordScalar("REAL_distance", real_distance);
    recordScalar("L1A_total_measurements", total_measurements);
    recordScalar("L1A_max_queue", max_queue_seen);
    if (total_measurements > 0) {
        recordScalar("L1A_avg_queue_length", sum_queue / total_measurements);
        recordScalar("L1A_avg_pressure", sum_pressure / total_measurements);
        recordScalar("L1B_valid_rate_percent",
                     100.0 * valid_measurements / total_measurements);
    }
    // KEY OUTCOME METRIC: total vehicle-seconds spent queued in my zone
    // (each queued vehicle contributes 1 second per measurement cycle)
    recordScalar("VEH_total_waiting_seconds", sum_queue);

    recordScalar("L1B_byzantine_detections", byzantine_detections);
    recordScalar("L2_decisions", l2_decisions);
    if (l2_decisions > 0)
        recordScalar("L2_avg_green_time", sum_green_time / l2_decisions);
    recordScalar("L2_state_light", count_light);
    recordScalar("L2_state_moderate", count_moderate);
    recordScalar("L2_state_heavy", count_heavy);
    recordScalar("L2_state_congested", count_congested);
    recordScalar("L3_voting_rounds", l3_voting_rounds);
    recordScalar("L2_light_actuations", l2_light_actuations);
    recordScalar("L3_votes_collected", l3_votes_collected);
    if (l3_voting_rounds > 0)
        recordScalar("L3_avg_voted_green", sum_voted_green / l3_voting_rounds);
    recordScalar("L4_reroute_commands_sent", l4_reroute_commands_sent);
    recordScalar("L4_vehicles_rerouted", l4_vehicles_rerouted);
    recordScalar("L4_greedy_evaluations", l4_greedy_evaluations);
    recordScalar("L4_actuations", l4_actuations);
    if (l4_greedy_evaluations > 0)
        recordScalar("L4_avg_greedy_score", sum_greedy_score / l4_greedy_evaluations);
    recordScalar("L4_candidates_used", (double)candidate_assignments.size());
    int max_assigned = 0;
    for (auto const& e : candidate_assignments)
        if (e.second > max_assigned) max_assigned = e.second;
    if (l4_vehicles_rerouted > 0)
        recordScalar("L4_distribution_max_share",
                     (double)max_assigned / l4_vehicles_rerouted);
    recordScalar("L6_quality_level", quality_level);
    recordScalar("L6_degradation_activations", degradation_activations);
    recordScalar("L6_speed_guidance_kmh", speed_guidance);
    recordScalar("L6_coordination_mode",
                 coordination_mode == "time-triggered" ? 1.0 : 0.0);
    recordScalar("L5_i_am_failed", i_am_failed ? 1 : 0);
    recordScalar("L5_neighbors_tracked", (double)neighbor_last_seen.size());
    recordScalar("L5_failures_detected", failures_detected);
    recordScalar("L5_false_alarm_recoveries", false_alarm_recoveries);
    recordScalar("L5_first_detection_time",
                 first_detection_time >= 0 ? first_detection_time.dbl() : -1);
    recordScalar("L5_pressure_spikes", pressure_spikes);

    veins::DemoBaseApplLayer::finish();
}
