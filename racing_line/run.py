from speed import optimize_racing_line
from map import track_map, test_map, racing_map
from identify_turn import analyze_track_from_grid, get_waypoints, generate_racing_line

track = racing_map
# track = track_map
if track == racing_map:
    for i in range(len(track)):
        for n in range(len(track[i])):
            if (track[i][n] == 0):
                track[i][n] = 1
            else:
                track[i][n] = 0


# results = analyze_track_from_grid(track, 7)
results = analyze_track_from_grid(track, 3)
# waypoints = generate_racing_line(results)
waypoints = get_waypoints(results)
# waypoints = waypoints[0:len(waypoints)//2]
speeds, controls = optimize_racing_line(
    waypoints, 
    results,
    base_speed=20,      # Max speed on straights
    min_corner_speed=3, # Min speed in tight corners
    max_accel=2,        # Max acceleration per step
    max_decel=4         # Max braking per step
)

print("---------------")
# print(waypoints)
print("\n\n",results)
# print("\n\n", results[0])
# print("\n\n", results[0]['coord'])
# print("\n\n", results[0]['type'])
# print(len(waypoints), len(results))

print("---------------")

print(f"Speeds: {speeds}")
print(f"Controls: {controls}")
# print(len(speeds), len(controls))