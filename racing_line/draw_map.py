racing_map = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 1, 1, 1, 1, 1, 0, 1],
    [1, 0, 0, 1, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 'F', 0, 0, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1]
]
# 0 is the road, 1 is the wall, 'F' is the start finish line

def draw_map(map_array):
    for i in range(len(map_array)):
        for j in range(len(map_array[0])):
            cell = map_array[i][j]
            if cell == 0:
                print("⬛", end=" ")
            elif cell == 1:
                print("⬜", end=" ")
            elif cell == 'F':
                print("🏁", end=" ")
        print("\n")

draw_map(racing_map)