#include <iostream>

#include "dedalus/world_model/world_snapshot.hpp"

int main() {
    dedalus::WorldSnapshot snapshot;

    snapshot.timestamp = dedalus::TimePoint{123456789};
    snapshot.active_map_frame_id = dedalus::MapFrameId{"map_local_0001"};

    snapshot.appearance_condition.timestamp = snapshot.timestamp;
    snapshot.appearance_condition.lighting_mode = dedalus::LightingMode::Day;
    snapshot.appearance_condition.weather_mode = dedalus::WeatherMode::Clear;
    snapshot.appearance_condition.sensor_mode = dedalus::SensorMode::Rgb;
    snapshot.appearance_condition.confidence = 0.75F;

    snapshot.ego.timestamp = snapshot.timestamp;
    snapshot.ego.map_frame_id = snapshot.active_map_frame_id;
    snapshot.ego.local_T_body.position = dedalus::Vec3{0.0, 0.0, -12.0};
    snapshot.ego.velocity_local = dedalus::Vec3{1.2, 0.0, 0.0};

    dedalus::AgentState person;
    person.agent_id = dedalus::AgentId{"agent_0001"};
    person.identity_id = dedalus::IdentityId{"identity_unknown_0001"};
    person.source_track_id = dedalus::TrackId{"track_0001"};
    person.last_seen = snapshot.timestamp;
    person.position_local = dedalus::Vec3{18.2, -3.1, -10.5};
    person.velocity_local = dedalus::Vec3{2.0, 0.4, 0.0};
    person.map_frame_id = snapshot.active_map_frame_id;
    person.class_label = dedalus::ClassLabel::Person;
    person.faction = dedalus::FactionLabel::Unknown;
    person.lifecycle = dedalus::AgentLifecycle::Active;
    person.confidence = 0.82F;
    snapshot.agents.push_back(person);

    dedalus::ContainerState car;
    car.container_id = dedalus::AgentId{"agent_car_0001"};
    car.type = dedalus::ContainerType::Car;
    car.local_T_container.position = dedalus::Vec3{20.0, -4.0, -10.5};
    car.velocity_local = dedalus::Vec3{0.0, 0.0, 0.0};
    car.map_frame_id = snapshot.active_map_frame_id;
    car.capacity_estimate = 4.0F;
    car.confidence = 0.65F;
    snapshot.containers.push_back(car);

    dedalus::MapFrame map_frame;
    map_frame.map_frame_id = snapshot.active_map_frame_id;
    map_frame.scale_confidence = 0.5F;
    map_frame.orientation_confidence = 0.5F;
    map_frame.created_at = snapshot.timestamp;
    map_frame.last_used = snapshot.timestamp;
    snapshot.map_frames.push_back(map_frame);

    std::cout << dedalus::to_json(snapshot);
    return 0;
}
