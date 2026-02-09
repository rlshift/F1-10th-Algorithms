from speed import optimize_racing_line
from map import track_map, test_map
from identify_turn import analyze_track_from_grid, get_waypoints, generate_racing_line

track = track_map

results = analyze_track_from_grid(track)
# waypoints = generate_racing_line(results)
waypoints = get_waypoints(results)
speeds, controls = optimize_racing_line(
    track, 
    results,
    base_speed=20,      # Max speed on straights
    min_corner_speed=3, # Min speed in tight corners
    max_accel=2,        # Max acceleration per step
    max_decel=4         # Max braking per step
)


print(waypoints)
# print("\n\n",results)


# print(f"Speeds: {speeds}")
# print(f"Controls: {controls}")