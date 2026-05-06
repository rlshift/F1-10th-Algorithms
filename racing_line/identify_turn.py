# ---------------------------------------------------------------------
# NOTE This analysis is for a track that goes clockwise, swap right for left if going anti-clockwise
# ---------------------------------------------------------------------
def measure_track_width(grid):
    """Sample the grid to find the median track width, returns a good track_width value."""
    rows = len(grid)
    cols = len(grid[0])
    widths = []

    for r in range(0, rows, 3):
        for c in range(0, cols, 3):
            if grid[r][c] != 1:  # 1 is road
                continue
            # Cast horizontally and vertically
            for dr, dc in [(0, 1), (1, 0)]:
                dist = 0
                nr, nc = r, c
                while 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 1:
                    nr += dr
                    nc += dc
                    dist += 1
                if dist > 1:
                    widths.append(dist)

    if not widths:
        return 6  # fallback

    widths.sort()
    median = widths[len(widths) // 2]
    # track_width step should be roughly half the track width
    return max(2, median // 2)


def analyze_track_from_grid(grid, track_width):
    centers = []
    for r in range(1, len(grid) - 1):
        for c in range(1, len(grid[0]) - 1):
            # If a pixel and all its neighbors are 1, it's a center
            if all(grid[r+i][c+j] == 1 for i in [-1,0,1] for j in [-1,0,1]):
                # To avoid duplicate centers for the same block, 
                # we only take every 3rd coordinate
                if r % track_width== 1 and c % track_width == 1:
                    centers.append([r, c])

    # Sort centers into a continuous path (Nearest Neighbor)
    if not centers: return []
    ordered_path = [centers.pop(0)]
    while centers:
        last = ordered_path[-1]
        next_pt = min(centers, key=lambda p: (p[0]-last[0])**2 + (p[1]-last[1])**2)
        ordered_path.append(next_pt)
        centers.remove(next_pt)

    # Analyze Curvature and Directions
    analysis = []
    n = len(ordered_path)
    for i in range(n):
        A = ordered_path[i-1]
        B = ordered_path[i]
        C = ordered_path[(i+1) % n]
        
        v1 = (B[0] - A[0], B[1] - A[1])
        v2 = (C[0] - B[0], C[1] - B[1])
        
        # Cross Product
        cp = (v1[0] * v2[1]) - (v1[1] * v2[0])
        
        # Identify type
        if cp > 0:
            t = "L"
        elif cp < 0:
            t = "R"
        else:
            # DS is Diagonal Straight 
            # S is Straight
            t = "DS" if v1[0] != 0 and v1[1] != 0 else "S"
            
        analysis.append({"coord": B, "cp": cp, "type": t})
        
    return analysis

def get_waypoints(results):
    ret = []
    for i in results:
        ret.append(i['coord'])
    return ret
# WIP Work in progress
def generate_racing_line(analysis, offset_distance=0.4):
    """
    Generate racing line waypoints offset from center:
    - Move RIGHT for LEFT turns (cp > 0) 
    - Move LEFT for RIGHT turns (cp < 0)
    - Stay center for straights (cp == 0)
    """
    racing_waypoints = []
    
    for i, entry in enumerate(analysis):
        coord = entry['coord'][:]  # [r, c]
        cp = entry['cp']
        
        if cp > 0:  # LEFT turn - move RIGHT
            # Previous and next vectors to get track direction at this point
            prev_pt = analysis[(i-1) % len(analysis)]['coord']
            next_pt = analysis[(i+1) % len(analysis)]['coord']
            
            # Track direction vector 
            track_dir = [
                next_pt[1] - prev_pt[1],  # x direction
                -(next_pt[0] - prev_pt[0]) # y direction 
            ]
            
            # Normalize
            length = (track_dir[0]**2 + track_dir[1]**2)**0.5
            if length > 0:
                track_dir = [track_dir[0]/length, track_dir[1]/length]
            
            # Offset RIGHT by distance
            coord[0] += track_dir[1] * offset_distance  
            coord[1] += track_dir[0] * offset_distance  
            
        elif cp < 0:  # RIGHT turn - move LEFT  
            prev_pt = analysis[(i-1) % len(analysis)]['coord']
            next_pt = analysis[(i+1) % len(analysis)]['coord']
            
            track_dir = [
                next_pt[1] - prev_pt[1],
                -(next_pt[0] - prev_pt[0])
            ]
            
            length = (track_dir[0]**2 + track_dir[1]**2)**0.5
            if length > 0:
                track_dir = [track_dir[0]/length, track_dir[1]/length]
            
            # Offset LEFT 
            coord[0] -= track_dir[1] * offset_distance
            coord[1] -= track_dir[0] * offset_distance
        coord[0] = round(coord[0])
        coord[1] = round(coord[1])
        
        racing_waypoints.append(coord)
    
    return racing_waypoints

def chaikin_smooth(waypoints, iterations=4):
    """
    Chaikin's corner cutting algorithm.
    Each iteration replaces each segment with two new points
    at 25% and 75% along the segment, producing a smooth curve.
    """
    pts = [pt[:] for pt in waypoints]
    
    for _ in range(iterations):
        new_pts = []
        n = len(pts)
        for i in range(n):
            p0 = pts[i]
            p1 = pts[(i + 1) % n]
            
            # Q: 25% along the segment
            q = [
                round(0.75 * p0[0] + 0.25 * p1[0]),
                round(0.75 * p0[1] + 0.25 * p1[1])
            ]
            # R: 75% along the segment
            r = [
                round(0.25 * p0[0] + 0.75 * p1[0]),
                round(0.25 * p0[1] + 0.75 * p1[1])
            ]
            new_pts.append(q)
            new_pts.append(r)
        pts = new_pts
    
    return pts
def downsample_waypoints(waypoints, step=4):
    """
    Keep every nth point after smoothing.
    step=4 undoes one Chaikin iteration (which doubles points),
    so for 3 iterations use step=8, for 2 use step=4.
    """
    return waypoints[::step]
