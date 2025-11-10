# RouteStorm
A hybrid geospatial and meteorological TSP engine that integrates lat/lon stop data with live open-source weather feeds to produce dynamic, risk-aware routing optimized for speed and safety.

Overview

1. Compute a base travel-time matrix from coordinates (great-circle or road network).

2. Pull open weather for each stop - we are using Open-Meteo which does not require an API key
   
3. Convert weather to slow-down factors (rain/snow/wind).

4. Multiply the base times by those factors to get a weather-adjusted cost matrix.

5. Solve TSP with OR-Tools and return the route.

