# Exploration Metrics

This page describes the metrics used to track exploration progress in FUEL.

## Frontier Exploration Rate (`/exploration/frontier_rate`)

The frontier exploration rate provides a measure of how many discovered frontier cells have been "covered" or "cleared" by the UAV's sensors.

### Calculation

The rate is calculated as a percentage:

$$Rate = \frac{Cells_{total} - Cells_{current}}{Cells_{total}} \times 100$$

Where:
- **$Cells_{total}$**: The cumulative count of unique voxels that have been identified as frontiers since the start of the mission.
- **$Cells_{current}$**: The number of voxels currently identified as frontiers (active or dormant).

### Why it is not Monotonically Increasing

Unlike volume-based exploration metrics, the frontier exploration rate may **decrease** temporarily during the mission. This happens for several reasons:

1.  **New Discoveries**: As the UAV moves into unknown territory, it might discover a large number of new frontier cells simultaneously. This increases $Cells_{total}$ and $Cells_{current}$, but if a large cluster is newly found, the proportion of "cleared" cells relative to the new, larger total may drop.
2.  **Frontier Clustering/Splitting**: The `FrontierFinder` dynamically clusters and filters frontier cells. Changes in the map can lead to existing frontiers being re-evaluated, which might slightly fluctuate the count of active cells.
3.  **Dormant Frontiers**: Frontiers that cannot be reached or lack valid viewpoints are moved to a dormant list. They still count as $Cells_{current}$ until they are actually cleared by the sensor (e.g., if the UAV passes near them while going elsewhere).

### Comparison with Volumetric Rate (`/sdf_map/explored_volume`)

| Metric | Definition | Behavior |
|--------|------------|----------|
| **Volumetric Rate** | Fraction of the total bounding box volume that is "Known" (Free or Occupied). | Always increases as more space is observed. |
| **Frontier Rate** | Fraction of identified "Unknown-Free" boundaries that have been cleared. | Focuses on mission objectives; can fluctuate as new objectives (frontiers) are found. |
