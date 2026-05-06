from speed import optimize_racing_line
from map import track_map, test_map, racing_map, new_map, racing_map2
from identify_turn import analyze_track_from_grid, get_waypoints, chaikin_smooth, downsample_waypoints, measure_track_width

track = racing_map2
# track = racing_map2
# track = track_map
# track = new_map
if track == racing_map or track == racing_map2:
    for i in range(len(track)):
        for n in range(len(track[i])):
            if (track[i][n] == 0):
                track[i][n] = 1
            else:
                track[i][n] = 0


track_width = measure_track_width(track)
print("\n\n \t Track Width = ", track_width)
results = analyze_track_from_grid(track, track_width=6)

# results = analyze_track_from_grid(track, 3)
# waypoints = generate_racing_line(results)

waypoints = get_waypoints(results)
waypoints = chaikin_smooth(waypoints, iterations=3)
waypoints = downsample_waypoints(waypoints, step=8) 
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