#include<bits/stdc++.h>
using namespace std;

namespace wpi {

/**
 * Implements a table of key-value pairs with linear interpolation between
 * values.
 *
 * If there's no matching key, the value returned will be a linear interpolation
 * between the keys before and after the provided one.
 *
 * @tparam Key   The key type.
 * @tparam Value The value type.
 */
template <typename Key, typename Value>
class InterpolatingMap {
    public:
    /**
     * Inserts a key-value pair.
     *
     * @param key   The key.
     * @param value The value.
     */
    void insert(const Key& key, const Value& value) {
        m_container.insert(std::make_pair(key, value));
    }

    /**
     * Inserts a key-value pair.
     *
     * @param key   The key.
     * @param value The value.
     */
    void insert(Key&& key, Value&& value) {
        m_container.insert(std::make_pair(key, value));
    }

    /**
     * Returns the value associated with a given key.
     *
     * If there's no matching key, the value returned will be a linear
     * interpolation between the keys before and after the provided one.
     *
     * @param key The key.
     */
    Value operator[](const Key& key) const {
        using const_iterator = typename std::map<Key, Value>::const_iterator;
        // Get iterator to upper bound key-value pair for the given key
        const_iterator upper = m_container.upper_bound(key);
        // If key > largest key in table, return value for largest table key
        if (upper == m_container.end()) {
          return (--upper)->second;
        }
        // If key <= smallest key in table, return value for smallest table key
        if (upper == m_container.begin()) {
          return upper->second;
        }
        // Get iterator to lower bound key-value pair
        const_iterator lower = upper;
        --lower;
        // Perform linear interpolation between lower and upper bound
        const double delta = (key - lower->first) / (upper->first - lower->first);
        return delta * upper->second + (1.0 - delta) * lower->second;
    }

    /**
     * Clears the contents.
     */
    void clear() { m_container.clear(); }

    private:
    std::map<Key, Value> m_container;
};

}  // namespace wpi

wpi::InterpolatingMap<double, double> velMap;

struct TrajectoryPoint{
    double position, velocity, acceleration, time;
};

struct VelocityLimit{
    double dStart,  dEnd,  velocity;
};

const double MAX_VELOCITY = 4.5;
const double MAX_ACCELERATION = 9;
const double MAX_DECELERATION = 4.5;

wpi::InterpolatingMap<double, double> generateProfile(double distance, double startVel = 0, double endVel = 0, std::vector<VelocityLimit> limit = {}){
    std::vector<TrajectoryPoint> profile;
    for(double i = 0; i <= distance; i += 0.01){
        profile.push_back(TrajectoryPoint{.position = i, .velocity = MAX_VELOCITY});
    }

    for(int i = 0; i < limit.size(); i++){
        for(double p = limit[i].dStart; p <= limit[i].dEnd; p += 0.01){
            profile[p/0.01].velocity = limit[i].velocity;
        }
    }

    profile[0].velocity = startVel;
    profile[profile.size()-1].velocity = endVel;

    for(int i = 1; i < profile.size(); i++){
        profile[i].velocity = min(profile[i].velocity, sqrt(profile[i-1].velocity * profile[i-1].velocity + 2 * MAX_ACCELERATION * 0.01));
    }

    for(int i = profile.size()-2; i >= 0; i--){
        profile[i].velocity = min(profile[i].velocity, sqrt(profile[i+1].velocity * profile[i+1].velocity + 2 * MAX_DECELERATION * 0.01));
    }

    for(int i = 0; i < profile.size()-1; i++){
        profile[i].acceleration = (profile[i+1].velocity * profile[i+1].velocity - profile[i].velocity * profile[i].velocity) / (2 * 0.01);
    }

    for(int i = 1; i < profile.size(); i++){
        if(profile[i-1].acceleration != 0){
            profile[i].time = profile[i-1].time + (profile[i].velocity - profile[i-1].velocity) / profile[i-1].acceleration;
        }
        else{
            profile[i].time = profile[i-1].time + 0.01 / profile[i].velocity;
        }
    }

    wpi::InterpolatingMap<double, double> ret;

    for(int i = 0; i < profile.size(); i++){
        ret.insert(profile[i].time, profile[i].velocity);
    }

    return ret;
}

int main(){
    auto profile = generateProfile(8, 0, 0, {{1, 3, 2}, {6, 7, 1}});
    ofstream os;
    os.open("Output.txt");
    for(double i = 0; i < 4.3; i += 0.01){
        os << profile[i] << endl;
    }
    os.close();
}