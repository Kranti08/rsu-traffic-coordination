#ifndef __VEINS_RSUTRAFFICCOORDINATION_H_
#define __VEINS_RSUTRAFFICCOORDINATION_H_

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include <set>
#include <map>
#include <vector>
#include <deque>
#include <string>

using namespace omnetpp;

class RSUTrafficCoordination : public veins::DemoBaseApplLayer {

public:
    RSUTrafficCoordination();
    virtual ~RSUTrafficCoordination();

protected:
    virtual void initialize(int stage) override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void onBSM(veins::DemoSafetyMessage* bsm) override;
    virtual void populateWSM(veins::BaseFrame1609_4* wsm,
                             veins::LAddress::L2Type rcvId = veins::LAddress::L2BROADCAST(),
                             int serial = 0) override;
    virtual void finish() override;

private:
    void measureTraffic();
    void validateMeasurement();
    void computeSignalTiming();
    void byzantineVoting();
    void applyDegradation();
    void checkNeighborHealth();
    void trackBackpressure();
    void rerouteVehicles();         // Layer 4: Greedy + Weighted distribution

    veins::TraCIScenarioManager* manager;
    double my_x, my_y;
    double zone_radius;
    double queue_speed_threshold;

    // Layer 1A
    int vehicles_in_zone;
    int raw_queue_length;
    int queue_length;
    int incoming_vehicles;
    int outgoing_vehicles;
    double pressure;
    std::set<std::string> vehicles_last_cycle;
    std::map<std::string, veins::Coord> vehicle_last_pos; // for heading proxy

    // Layer 1B
    int last_valid_queue;
    int max_queue_change_per_sec;
    int byzantine_detections;
    std::deque<int> queue_history;      // Weakness #2: rolling window
    double max_drift_observed = 0.0;    // observe-only stats
    double sum_drift_observed = 0.0;
    int drift_samples = 0;
    int consecutive_high_drift = 0;     // Weakness #2: persistence counter
    int drift_rejections = 0;           // sustained-drift attacks caught
    double drift_threshold = 0.0;       // from param (data-driven)
    int drift_persistence_k = 0;        // consecutive steps required
    int beacon_queue_count = 0;         // Weakness #3: queue from vehicle self-reports
    double max_xcheck_gap = 0.0;        // observe-only: max |SUMO - beacon| gap
    double sum_xcheck_gap = 0.0;
    int xcheck_samples = 0;
    int consecutive_xcheck_gap = 0;
    int xcheck_rejections = 0;
    double xcheck_threshold = 0.0;
    int valid_measurements;
    double byzantine_time;
    bool byzantine_injected;
    double bias_time;         // when the SMART (plausible) lie starts
    double bias_factor;
    double bias_target = -1.0;       // e.g. 1.4 = +40% biased proposal
    std::vector<std::string> my_lights;   // traffic lights this RSU controls
    long l2_light_actuations = 0;         // times we set a signal
    int ns_queue = 0;
    int ew_queue = 0;
    double real_wait_seconds = 0.0;
    double real_distance = 0.0;
    std::map<std::string,int> last_phase;
    double drift_attack_time = -1.0;
    int drift_creep = 0;
    bool bias_active;
    double sum_bias_shift;
    int bias_shift_samples;
    // Decision-cycle stopwatch (real wall-clock time per cycle)
    double timing_sum_ms;
    double timing_max_ms;
    double timing_min_ms;
    int timing_samples;
    // Caching feasibility experiment (professor's question)
    // bucket name -> saved green time. Tiny: max 4 entries.
    bool enable_caching;
    std::map<std::string, double> decision_cache;
    std::map<std::string, double> neighbor_shared_cache; // borrowed answers
    int cache_hits;
    int cache_cross_hits;   // reused a NEIGHBOR's shared answer
    int cache_misses;
    double sum_staleness;   // |cached answer - fresh answer|
    double cache_lookup_ms_sum;
    double cache_compute_ms_sum;

    // Layer 2
    double proposed_green_time;
    std::string traffic_state;
    int l2_decisions;
    double sum_green_time;
    int count_light, count_moderate, count_heavy, count_congested;

    // Layer 3 (now with REAL shared data via beacons)
    std::map<std::string, double> neighbor_proposals;
    std::map<std::string, double> neighbor_queues;   // Greedy congestion input
    double voted_green_time;
    int l3_votes_collected;
    int l3_voting_rounds;
    double sum_voted_green;

    // Layer 4: Greedy + Weighted
    bool enable_rerouting;
    double reroute_penalty;
    bool rerouting_active;
    int l4_reroute_commands_sent;
    int l4_vehicles_rerouted;
    std::set<std::string> rerouted_vehicles;
    std::map<std::string, int> candidate_assignments; // weighted distribution
    double failed_rsu_x, failed_rsu_y;
    bool failed_rsu_known;
    double w_dist, w_cong, w_heur;   // Greedy weights 1.0 / 2.0 / 0.5
    double junction_capacity;         // 30 vehicles
    int l4_greedy_evaluations;
    int l4_actuations;
    double sum_greedy_score;

    // Layer 6
    int active_failures;
    int quality_level;
    double green_time_cap;
    double speed_guidance;
    int degradation_activations;
    std::string coordination_mode;

    // Layer 5b
    double last_pressure;
    double spike_threshold;
    int pressure_spikes;

    // Timing / failure injection
    double measurement_interval;
    cMessage* measurement_timer;
    double failure_time;
    bool i_am_failed;
    cMessage* failure_timer;

    // Layer 5a
    std::map<std::string, simtime_t> neighbor_last_seen;
    std::map<std::string, bool> neighbor_failed;
    double heartbeat_timeout;
    int failures_detected;
    simtime_t first_detection_time;

    // Ground truth (experimenter's view - immune to failure & injection)
    int countRealQueue();
    double gt_waiting_seconds;
    int countNetworkQueue();
    double gt_network_waiting;

    // V2X vehicle destination beacons: (pos, dest, time)
    struct DestBeacon { veins::Coord pos; veins::Coord dest; simtime_t t; };
    std::vector<DestBeacon> dest_beacons;
    int l4_dest_from_v2x;
    int l4_dest_inside_skipped;
    std::vector<std::pair<double,double>> failed_positions;
    int false_alarm_recoveries;
    int l4_dest_fallback;

    // Stats
    int total_measurements;
    int replay_rejections = 0;   // Change 2b: stale/replayed beacons rejected
    int auth_rejections = 0;     // Change 4: failed auth-tag verification
    double sum_queue;
    double sum_pressure;
    int max_queue_seen;
};

#endif
