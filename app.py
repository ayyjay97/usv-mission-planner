import streamlit as st
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import random

# --- A* Algorithm Implementation ---
# No changes needed here logic-wise, provided the grid treats anything != 0 as an obstacle.

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0 
        self.h = 0 
        self.f = 0 

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def astar_search(grid, start, end):
    """
    Executes A* Search to find the optimal path for the USV.
    :param grid: 2D numpy array (0=Navigable, 1=Land, 2=Contact)
    """
    rows, cols = grid.shape
    if not (0 <= start[0] < rows and 0 <= start[1] < cols): return None
    if not (0 <= end[0] < rows and 0 <= end[1] < cols): return None
    
    # Check if start/end are obstacles (Land or Contact)
    if grid[start] != 0 or grid[end] != 0: return None

    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)
    closed_set = set()

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)

        if current_node == end_node:
            path = []
            current = current_node
            while current:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0)] # Left, Right, Up, Down
        for new_position in neighbors:
            node_position = (current_node.position[0] + new_position[0], 
                             current_node.position[1] + new_position[1])

            if (node_position[0] > (rows - 1) or node_position[0] < 0 or 
                node_position[1] > (cols - 1) or node_position[1] < 0): continue

            # OBSTACLE CHECK: 
            # 0 = Water (Navigable)
            # 1 = Land (Obstacle)
            # 2 = Surface Contact (Obstacle)
            if grid[node_position] != 0: continue
            
            if node_position in closed_set: continue

            new_node = Node(current_node, node_position)
            new_node.g = current_node.g + 1
            new_node.h = abs(new_node.position[0] - end_node.position[0]) + \
                         abs(new_node.position[1] - end_node.position[1])
            new_node.f = new_node.g + new_node.h

            if any(open_node for open_node in open_list 
                   if new_node == open_node and new_node.g > open_node.g):
                continue

            heapq.heappush(open_list, new_node)

    return None

# --- Streamlit Application ---

st.set_page_config(page_title="USV Mission Planner", layout="wide")

st.title("⚓ USV Mission Planner")
st.markdown("""
**Operational View:**
* **Water:** Light Blue
* **Landmass:** Dark Grey
* **Surface Contacts:** Cyan Circles
* **USV Track:** Gold
""")

# 1. Initialize Grid in Session State
GRID_SIZE = 20
if 'grid' not in st.session_state:
    st.session_state.grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

# 2. Sidebar Controls
st.sidebar.header("Mission Parameters")

# -- Deployment & Objective --
st.sidebar.subheader("📍 Deployment Point")
col1, col2 = st.sidebar.columns(2)
with col1: start_x = st.number_input("Lat (Row)", 0, GRID_SIZE-1, 0)
with col2: start_y = st.number_input("Long (Col)", 0, GRID_SIZE-1, 0)
start_pos = (start_x, start_y)

st.sidebar.divider()

st.sidebar.subheader("🎯 Mission Objective")
col3, col4 = st.sidebar.columns(2)
with col3: end_x = st.number_input("Obj Lat (Row)", 0, GRID_SIZE-1, GRID_SIZE-1)
with col4: end_y = st.number_input("Obj Long (Col)", 0, GRID_SIZE-1, GRID_SIZE-1)
end_pos = (end_x, end_y)

st.sidebar.divider()

# -- Environment Generation --
st.sidebar.subheader("🗺️ Environment Settings")
num_contacts = st.sidebar.slider("Surface Contact Density (Count)", 0, 30, 10)

if st.sidebar.button("Generate Environment"):
    # 1. Generate Land (20% coverage)
    # We use 0 for water, 1 for land
    new_grid = np.random.choice([0, 1], size=(GRID_SIZE, GRID_SIZE), p=[0.8, 0.2])
    
    # 2. Generate Surface Contacts (Value = 2)
    # Find all coordinates that are currently Water (0)
    water_indices = np.argwhere(new_grid == 0)
    
    # Select random indices for contacts
    if len(water_indices) >= num_contacts:
        # Randomly choose indices
        contact_indices = water_indices[np.random.choice(len(water_indices), num_contacts, replace=False)]
        
        # Mark them as 2 (Contact)
        for r, c in contact_indices:
            new_grid[r, c] = 2
            
    st.session_state.grid = new_grid

# Ensure Start/End are navigable (Clear land/contacts from start/end)
st.session_state.grid[start_pos] = 0
st.session_state.grid[end_pos] = 0

# 3. Execution & Metrics
path = astar_search(st.session_state.grid, start_pos, end_pos)

st.sidebar.divider()
st.sidebar.subheader("📊 Mission Metrics")

if path:
    fuel_cost = len(path) - 1 
    st.sidebar.metric(label="Estimated Fuel Cost", value=f"{fuel_cost} Units")
    st.sidebar.success("Path solution found.")
else:
    st.sidebar.metric(label="Estimated Fuel Cost", value="N/A")
    if start_pos != end_pos:
        st.sidebar.error("No valid path. Objective is blocked.")

# 4. Visualization Logic
fig, ax = plt.subplots(figsize=(10, 10))

# -- Render Base Grid (Water and Land) --
# We mask the contacts (2) temporarily so imshow only deals with 0 and 1 for the background
# This prevents imshow from messing up the colors if we just want 2 colors for the map base
display_grid = st.session_state.grid.copy()
display_grid[display_grid == 2] = 0 # Treat contacts as water for the background layer

cmap = colors.ListedColormap(['#e0f7fa', '#424242']) # Light Blue, Dark Grey
bounds = [0, 0.5, 1]
norm = colors.BoundaryNorm(bounds, cmap.N)

ax.imshow(display_grid, cmap=cmap, norm=norm)

# -- Render Surface Contacts (Cyan Circles) --
# Find all coordinates where grid == 2
contact_rows, contact_cols = np.where(st.session_state.grid == 2)
# Scatter plot them. Note: Scatter takes (x, y), so we pass (cols, rows)
ax.scatter(contact_cols, contact_rows, s=200, c='#00BCD4', marker='o', edgecolors='black', linewidth=1.5, label='Surface Contact')

# -- Grid Styling --
ax.grid(which='major', axis='both', linestyle='-', color='white', linewidth=0.5, alpha=0.1)
ax.set_xticks(np.arange(-.5, GRID_SIZE, 1))
ax.set_yticks(np.arange(-.5, GRID_SIZE, 1))
ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

# -- Render Path --
if path:
    path_y = [p[1] for p in path] 
    path_x = [p[0] for p in path] 
    ax.plot(path_y, path_x, color='#FFD700', linewidth=4, marker='o', markersize=6, label='USV Track')

# -- Render Endpoints --
ax.plot(start_pos[1], start_pos[0], 'o', color='#2E7D32', markersize=15, markeredgecolor='white', label='Deployment Point')
ax.plot(end_pos[1], end_pos[0], 'X', color='#C62828', markersize=15, markeredgecolor='white', label='Mission Objective')

# -- Legend --
ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=4, frameon=False, fontsize=10)

st.pyplot(fig)