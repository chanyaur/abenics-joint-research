import pyvista as pv
import numpy as np

# 1. Load or create a base mesh
# mesh = pv.Sphere()
mesh = pv.read("cs_gear.stl")

# 2. Calculate the distance from the center along X and Y axes
dist_x = np.abs(mesh.points[:, 0])
dist_y = np.abs(mesh.points[:, 1])

# 3. Combine them. 
# Taking the maximum ensures that if a point is far along X OR far along Y, 
# it gets a high heat value.
combined_heat = np.maximum(dist_x, dist_y)

# 4. Attach the combined continuous data to the mesh
mesh.point_data["pole_heatmap"] = combined_heat

# 5. Plot the mesh with a heatmap colormap
plotter = pv.Plotter()

# 'hot' or 'jet' are perfect for this. 
# With 'hot' or 'jet', the highest values (the poles) will be bright red/white,
# while the center/neutral areas will be black/blue.
plotter.add_mesh(
    mesh, 
    scalars="pole_heatmap", 
    cmap="jet",            # Try "jet" if you want a blue-to-red gradient instead of black-to-red
    show_scalar_bar=True,
    scalar_bar_args={"title": "Pole Intensity"}
)

plotter.show()