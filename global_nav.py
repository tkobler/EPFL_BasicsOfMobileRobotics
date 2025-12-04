import numpy as np
import heapq
from math import sqrt
import constants as cst
import matplotlib.pyplot as plt
import cv2

#obstacle minimum 3 points no lines

class GlobalNav:

    def __init__(self, arena_width=1189, arena_height=841):
        self.width = arena_width
        self.height = arena_height
        self.nodes = []
        self.edges = {}

    def dist(self, a, b):
        return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def ccw(self, A, B, C): #are the points in counter-clockwise order? return True if > 0, False for cw < 0 and colinear = 0
        return (C[1] - A[1]) * (B[0] - A[0]) - (B[1] - A[1]) * (C[0] - A[0]) > 0 #https://www.geeksforgeeks.org/dsa/orientation-3-ordered-points/

    def segments_intersect(self, p1, p2, p3, p4): #return True if they cross
        return (self.ccw(p1, p3, p4) != self.ccw(p2, p3, p4)) and (self.ccw(p1, p2, p3) != self.ccw(p1, p2, p4)) #https://www.geeksforgeeks.org/dsa/check-if-two-given-line-segments-intersect/
        #two segments AB and CD intersect if and only if: C and D are on different sides of AB and A and B are on different sides of CD

    def segment_hits_polygon(self, A, B, poly): #true if segment AB intersects any edge of polygon poly
        n = len(poly)
        for i in range(n):
            C = poly[i]
            D = poly[(i + 1) % n] #pair of edges, loops around

            #if AB shares an endpoint with the edge CD -> allowed
            if A == C or A == D or B == C or B == D:
                continue #skip next if

            #check edge intersection
            if self.segments_intersect(A, B, C, D):
                return True  
        return False

    def point_in_polygon(self, point, poly):
        x, y = point
        inside = False
        n = len(poly)

        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]

            #cross product == 0 -> collinear
            cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
            if abs(cross) < cst.EPSILON: #accept tiny tolerance because never 0
                # dot product <= 0 -> point is between endpoints
                dot = (x - x1) * (x - x2) + (y - y1) * (y - y2)
                if dot <= cst.EPSILON:
                    return False  #on edge = considered outside

            #Ray casting : check if the horizontal ray crosses edge (x1,y1)-(x2,y2)
            if (y1 > y) != (y2 > y): #y is between y1 and y2; compute intersection x
                x_intersect = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < x_intersect:
                    inside = not inside
        return inside #True = inside, False = outside

    def segment_visible(self, A, B, polygon_list):
        mid = ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)

        for poly in polygon_list:
            if self.segment_hits_polygon(A, B, poly): #AB must not intersect any polygon edge
                return False
            if self.point_in_polygon(mid, poly): #midpoint inside polygon (for segments between the polygon's points)
                return False
        return True

    def inflate_polygon(self, poly): #inflate a polygon by moving each corner away from its centroid
        cx = np.mean([p[0] for p in poly]) #finfing centroid
        cy = np.mean([p[1] for p in poly])

        inflated = []
        for (x, y) in poly:
            vx = x - cx #dist from centroid to corner
            vy = y - cy
            L = sqrt(vx * vx + vy * vy) + 1e-6 #avoid div by 0, when center is on a point(like a 3 point line)
            scale = (L + cst.INFLATION_R) / L #scale factor to push vertex outward by inflation radius
            inflated.append((cx + vx * scale, cy + vy * scale)) #apply scaling to get inflated vertex
        return inflated

    def build_visibility_graph(self, start, goal, obstacles): #build the visibility graph on inflated obstacles
        """
        Inputs:
            start, goal: (x, y) points in mm
            obstacles: list of polygons [[(x,y),...], [...]]
        Returns:
            nodes: list of all points (start, goal, obstacle vertices)
            edges: dict {idx: [(neighbor_idx, distance), ...]}
        """
        inflated = [self.inflate_polygon(poly) for poly in obstacles] #inflate obstacles

        nodes = [start, goal] #idx 0 = start, 1 = goal
        for poly in inflated: #collect all nodes
            nodes.extend(poly)

        #build edges between all visible pairs
        N = len(nodes)
        edges = {i: [] for i in range(N)}

        for i in range(N):
            for j in range(i + 1, N):
                A = nodes[i]
                B = nodes[j]

                if self.segment_visible(A, B, inflated):
                    d = self.dist(A, B)
                    edges[i].append((j, d))
                    edges[j].append((i, d)) #adds to list with [(point, distance),...]

        return nodes, edges

    def dijkstra(self, edges, start_idx, goal_idx): #help from Exercise 5 part 1
        pq = [(0.0, start_idx)] #(cost_so_far, node_index), priority queue implemented with heapq
        cost = {start_idx: 0.0} #dictionary: node -> best known cost
        parent = {start_idx: None} #keeps track of how we reached each node

        while pq:
            c, u = heapq.heappop(pq) #pops the element with smallest cost
            if u == goal_idx:
                break

            for v, w in edges[u]: #explore neighbors of u
                newc = c + w #total cost to reach v via u
                if v not in cost or newc < cost[v]: #update if first time visiting v or found better path
                    cost[v] = newc
                    parent[v] = u
                    heapq.heappush(pq, (newc, v))

        if goal_idx not in parent: #if goal has no parent, no path exists
            return None  # no path

        #reconstruct path, go backwards using the parent dictionary
        path = []
        cur = goal_idx
        while cur is not None:
            path.append(cur)
            cur = parent[cur]
        return path[::-1] #returns list of node indices for the shortest path.

    def find_optimal_path(self, thymio, goal, obstacles):
        """
        Inputs:
            thymio: (x, y, theta) in mm, rad
            goal: (x, y) in mm
            obstacles: list of polygons from vision
        Returns:
            path_world: list of (x, y) waypoints in mm.
        """
        start_pt = [thymio[0], thymio[1]]
        goal_pt = [goal[0], goal[1]]

        self.nodes, self.edges = self.build_visibility_graph(start_pt, goal_pt, obstacles)
        self.nodes = self.nodes  #store for plotting later

        path_idx = self.dijkstra(self.edges, start_idx=0, goal_idx=1) #goal idx = 1 bc of build_visibility
        if path_idx is None:
            print("GlobalNav (visibility): No path found!")
            return []

        path_world = [[int(self.nodes[i][0]), int(self.nodes[i][1])] for i in path_idx] #convert node indices to (x,y) coordinates
        return path_world  #removes start point to avoid stopping immediately

    def is_goal_reached(self, current_pos, goal_pos): #tol = 70mm bc R of the Thymio is ~55, so a bit more 
        return self.dist(current_pos, goal_pos) < cst.GOAL_TOL
    
    def plot_map(self, obstacles, path, start, goal): #generated with chatGPT

        fig, ax = plt.subplots(figsize=(9, 7))
        ax.set_title("Visibility Graph + Shortest Path", fontsize=16)
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.grid(True)

        #plot raw obstacles
        for poly in obstacles:
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            ax.fill(xs, ys, color="black", alpha=0.4)

        #plot visibility edges
        for u in self.edges:
            for v, _ in self.edges[u]:
                x1, y1 = self.nodes[u]
                x2, y2 = self.nodes[v]
                ax.plot([x1, x2], [y1, y2], color="gray", linewidth=1, alpha=0.35)

        #plot nodes
        for (x, y) in self.nodes:
            ax.plot(x, y, "bo", markersize=5)

        #plot optimal path
        if len(path) >= 2:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, "r-", linewidth=2, label="Shortest Path")

        #Start + Goal
        ax.plot(start[0], start[1], "go", markersize=10, label="Start")
        ax.plot(goal[0], goal[1], "yo", markersize=10, label="Goal")

        #axis settings
        ax.set_xlim(0, self.width)
        ax.set_ylim(self.height, 0)
        ax.invert_yaxis()
        ax.legend()
        plt.show()

        print("Press 'p' to close the plot.")
        plot_graph = True
        key = -1
        while plot_graph:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('p'):
                plot_graph = False
                plt.close('all')
                break


if __name__ == "__main__":
    nav = GlobalNav()

    obstacles = [
        [(140, 260), (200, 400), (300, 180)],
        [(400, 500), (600, 380), (900, 460), (900, 540), (600, 620)],
    ]

    start = (80, 100)
    goal  = (920, 400)

    #build graph
    nodes, edges = nav.build_visibility_graph(start, goal, obstacles)

    #compute optimal path
    path_world = nav.find_optimal_path(start, goal, obstacles)
    path = [(int(x), int(y)) for (x, y) in path_world]

    #call the internal plot function
    nav.plot_map(obstacles, nodes, edges, path, start, goal)