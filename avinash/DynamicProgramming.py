import random

def generate_traffic_grid(n, m):
    return [[random.choice([0, 1]) for _ in range(m)] for _ in range(n)]

def adaptive_traffic_signal_dp_with_path(grid, start, destination):
    n, m = len(grid), len(grid[0])
    dp = [[float('inf')] * m for _ in range(n)]
    parent = [[None for _ in range(m)] for _ in range(n)]  # To track path
    dp[start[0]][start[1]] = 0

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for i in range(n):
        for j in range(m):
            if dp[i][j] == float('inf'):
                continue
            for di, dj in directions:
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < m:
                    signal_cost = 1 if grid[ni][nj] == 1 else 0
                    new_cost = dp[i][j] + signal_cost
                    if new_cost < dp[ni][nj]:
                        dp[ni][nj] = new_cost
                        parent[ni][nj] = (i, j)  # Track where we came from

    # Reconstruct path from destination to start
    path = []
    curr = destination
    while curr is not None:
        path.append(curr)
        curr = parent[curr[0]][curr[1]]
    path.reverse()  # Reverse the path to get start -> destination

    print("dp:")
    for row in dp:
        print(row)

    return dp[destination[0]][destination[1]], path

# -------------------------
# Example usage
n, m = 5, 5
grid = generate_traffic_grid(n, m)
start = (0, 0)
destination = (4, 4)

print("Initial grid (0 = green, 1 = red):")
for row in grid:
    print(row)

# Compute minimum delay and path
cost, path = adaptive_traffic_signal_dp_with_path(grid, start, destination)
print(f"\n[DP] Minimum delay to reach destination: {cost}")
print("[DP] Optimal path:")
print(" -> ".join(map(str, path)))
