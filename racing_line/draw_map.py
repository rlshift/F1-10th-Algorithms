from run import controls, waypoints, track, test_map
from speed import Controls
# DONE Holding and not accelerating for a majority of the track 
# 0 is the road, 1 is the wall, 'F' is the start finish line

def draw_map(map_array):
    first = False
    isCell = False
    print(f"Waypoints: {waypoints}")
    for i in range(len(map_array)):
        for j in range(len(map_array[0])):
            cell = map_array[i][j]

            for n in range(len(waypoints)):
                isCell = waypoints[n][0] == i and waypoints[n][1] == j
                if (isCell):
                    if (first == False):
                        print("🟩", end=" ")
                        first = True
                    elif (controls[n] == Controls.ACCEL):
                        print("🟩", end=" ")
                    elif (controls[n] == Controls.BRAKE):
                        print("🟥", end=" ")
                    elif (controls[n] == Controls.HOLD):
                        print("🟨", end=" ")
                    break
            if (not isCell):
                if cell == 0:
                    print("⬛", end=" ")
                elif cell == 1:
                    print("⬜", end=" ")

        print("")


draw_map(track)
