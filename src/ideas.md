# States:
- Wait For Object + Placement Clicks
- Extract Click 3D
- Pickup Object
- Check correct + rotate 
- Place Object
- Stow Arm

OR callback order: click -> extract 3D -> arm move + stow on end

# TODO
- Refine, test vision node
    - Paramaters
    - Filters
    - Check when sending messages and where
    - RViz
- Task Manager state machine (separate node)
- Arm node + frame

# Processing / filters
- Current: 2D -> 3D, camera -> arm, crop, SOR, voxel, plane, cluster, (optimal_grasp) curvature
- New: 2D -> 3D, camera -> arm, crop (few cm around 3D), SOR/voxel??, cluster to check if piece is card/card stack based on width
    - OR: crop to board + SOR + voxel + plane as one-time filters to check ground / cards (this may vary if cards stacked AND/OR board thickness)
    - Check if obstacles around (i.e. if can be safely picked up)