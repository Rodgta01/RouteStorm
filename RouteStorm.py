from math import radians, sin, cos, asin, sqrt
from datetime import datetime, timezone
from dateutil import parser as dateparser
import requests

# ---------- 1) Input your stops ----------
# Format: list of dicts with name, lat, lon. 'when' is ISO8601 time you expect to be there (optional).
stops = [
    {"name": "Depot", "lat": 41.1176, "lon": -85.0689, "when": "2025-11-10T08:00:00-05:00"},
    {"name": "Stop A", "lat": 41.1802, "lon": -84.9960, "when": "2025-11-10T08:15:00-05:00"},
    {"name": "Stop B", "lat": 41.0953, "lon": -85.1394, "when": "2025-11-10T08:30:00-05:00"},
    {"name": "Stop C", "lat": 41.2281, "lon": -85.0111, "when": "2025-11-10T08:45:00-05:00"},
]

# ---------- 2) Great-circle distance (km) ----------
def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0088
    dlat, dlon = radians(lat2-lat1), radians(lon2-lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
    return 2*R*asin(sqrt(a))

# (Optional) If you have OSRM or OpenRouteService, replace with road distance/ETA here.

# ---------- 3) Base travel-time matrix (minutes) ----------
def build_base_time_matrix(stops, assumed_speed_kph=35.0):
    n = len(stops)
    m = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j: 
                continue
            km = haversine_km(stops[i]["lat"], stops[i]["lon"], stops[j]["lat"], stops[j]["lon"])
            minutes = (km / assumed_speed_kph) * 60.0
            m[i][j] = minutes
    return m

# ---------- 4) Pull open-source weather (Open-Meteo; no API key) ----------
# We’ll grab hourly precipitation, snowfall, and wind. You can swap to Meteostat if you prefer.
def get_weather_factor(lat, lon, when_iso):
    """
    Returns a multiplicative slow-down factor >= 1.0 based on forecast/nowcast.
    Simple rule set you can tune for your fleet.
    """
    dt = dateparser.parse(when_iso).astimezone(timezone.utc)
    date_str = dt.strftime("%Y-%m-%d")
    hour_str = dt.strftime("%H:00")

    url = (
        "https://api.open-meteo.com/v1/forecast"
        f"?latitude={lat}&longitude={lon}"
        "&hourly=precipitation,snowfall,wind_speed_10m,wind_gusts_10m"
        f"&start_date={date_str}&end_date={date_str}&timezone=UTC"
    )
    r = requests.get(url, timeout=20)
    r.raise_for_status()
    data = r.json().get("hourly", {})

    # Find index for our hour
    times = data.get("time", [])
    try:
        idx = times.index(f"{date_str}T{hour_str}")
    except ValueError:
        # If the exact hour isn't available, fall back to a neutral factor
        return 1.0

    precip = (data.get("precipitation") or [0])[idx] or 0  # mm
    snow   = (data.get("snowfall") or [0])[idx] or 0       # cm
    wind   = (data.get("wind_speed_10m") or [0])[idx] or 0 # km/h
    gust   = (data.get("wind_gusts_10m") or [0])[idx] or 0 # km/h

    # ----- 5) Convert weather to a factor (tune these as needed) -----
    factor = 1.0
    # Rain: modest slow-downs
    if precip >= 0.5:   factor += 0.10   # light/moderate rain
    if precip >= 5.0:   factor += 0.10   # heavy rain

    # Snow: larger slow-downs
    if snow >= 0.1:     factor += 0.20   # light snow
    if snow >= 1.0:     factor += 0.30   # heavier snow

    # Wind/Gusts: add some penalty
    if wind >= 30:      factor += 0.05
    if gust >= 50:      factor += 0.10

    return factor

def get_stop_weather_factors(stops):
    """
    Get a per-stop factor at the ETA you plan to be there.
    If 'when' missing, use departure of first stop as fallback.
    """
    default_when = stops[0].get("when")
    factors = []
    for s in stops:
        when_iso = s.get("when") or default_when
        factors.append(get_weather_factor(s["lat"], s["lon"], when_iso))
    return factors  # one factor per node

# ---------- 6) Build weather-adjusted time matrix ----------
def apply_weather_to_matrix(base_matrix, node_factors):
    """
    Simple approach: time(i->j) *= max(node_factor[i], node_factor[j])
    You can make this fancier (e.g., along-route weather sampling).
    """
    n = len(base_matrix)
    m = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            m[i][j] = base_matrix[i][j] * max(node_factors[i], node_factors[j])
    return m

# ---------- 7) Solve TSP with OR-Tools ----------
def solve_tsp(time_matrix, start_index=0):
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2

    n = len(time_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, start_index)  # single vehicle
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        i = manager.IndexToNode(from_index)
        j = manager.IndexToNode(to_index)
        return int(round(time_matrix[i][j] * 100))  # scale to integer

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.time_limit.FromSeconds(10)
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH

    solution = routing.SolveWithParameters(search_params)
    if not solution:
        raise RuntimeError("No TSP solution found")

    # Extract route
    idx = routing.Start(0)
    order = [manager.IndexToNode(idx)]
    while not routing.IsEnd(idx):
        idx = solution.Value(routing.NextVar(idx))
        order.append(manager.IndexToNode(idx))

    total_minutes = 0.0
    for a, b in zip(order[:-1], order[1:]):
        total_minutes += time_matrix[a][b]

    return order, total_minutes

# ---------- 8) Put it all together ----------
base = build_base_time_matrix(stops, assumed_speed_kph=35.0)
factors = get_stop_weather_factors(stops)          # calls Open-Meteo
weather_adj = apply_weather_to_matrix(base, factors)
order, total_min = solve_tsp(weather_adj, start_index=0)

print("Visit order:")
for k in order:
    print(k, stops[k]["name"])
print(f"Total travel time (weather-adjusted): {total_min:.1f} min")
print("Node weather factors:", [round(x, 2) for x in factors])

