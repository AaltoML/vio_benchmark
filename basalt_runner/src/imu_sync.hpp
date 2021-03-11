#ifndef IMU_SYNC_H
#define IMU_SYNC_H

#include <functional>
#include <deque>
#include <iostream>

// _Not_ thread safe imu sample synchronization
class ImuSync {
public:
    std::function<void(double time, double leaderX, double leaderY, double leaderZ, double followerX, double followerY, double followerZ)> onSyncedLeader;

    struct Sample {
        double time;
        double x;
        double y;
        double z;
    };

    void process() {
        if (leaderSamples.empty() || followerSamples.size() < 2) return;
        while(!leaderSamples.empty() && leaderSamples.back().time < followerSamples.back().time) {
            // If we don't have a follower sample before leader samples timestamp, drop them
            leaderSamples.pop_back();
        }
        auto prevFollower = followerSamples.rbegin();
        auto nextFollower = followerSamples.rbegin() + 1;
        while (
            !leaderSamples.empty()
            && leaderSamples.back().time <= followerSamples.front().time) {
            while (leaderSamples.back().time > nextFollower->time) {
                prevFollower++;
                nextFollower++;
                followerSamples.pop_back();
            }
            Sample current = leaderSamples.back();
            leaderSamples.pop_back();
            double t0 = (nextFollower->time - current.time) / (nextFollower->time - prevFollower->time);
            double t1 = 1. - t0;
            if (onSyncedLeader)
                onSyncedLeader(
                    current.time,
                    current.x,
                    current.y,
                    current.z,
                    prevFollower->x * t0 + nextFollower->x * t1,
                    prevFollower->y * t0 + nextFollower->y * t1,
                    prevFollower->z * t0 + nextFollower->z * t1
                );
        }
    }

    void addLeader(double time, double x, double y, double z) {
        // Not strictly necessary in scope of this class, but leads to more nicely behaved data
        if (!this->leaderSamples.empty() && time <= this->leaderSamples.front().time)
            return;
        this->leaderSamples.emplace_front(Sample {time, x, y, z});
        process();
    }

    void addFollower(double time, double x, double y, double z) {
        // Follower samples timestamps must be increasing to avoid divided by zero
        if (!this->followerSamples.empty() && time <= this->followerSamples.front().time)
            return;
        this->followerSamples.emplace_front(Sample {time, x, y, z});
        process();
    }

private:
    std::deque<Sample> leaderSamples;
    std::deque<Sample> followerSamples;
};

#endif // IMU_SYNC_H
