import matplotlib.pyplot as plt

# Extract lat/lon from QGC WPL datapyth
# extract from mission_lkhd_0.txt 8th and 9th column
data = []
with open('mission_lkhd_0.txt', 'r') as file:
    lines = file.readlines() 


for line in lines:
    parts = line.strip().split('\t')
    if len(parts) > 9: 
      lat = float(parts[8]) 
      lon = float(parts[9]) 
      data.append((lat, lon))


# Separate latitudes and longitudes
lats = [lat for lat, lon in data]
lons = [lon for lat, lon in data]

# Plotting
plt.figure(figsize=(8, 8))
plt.plot(lons, lats, marker='o', linestyle='-')
plt.scatter(lons[0], lats[0], color='green', s=100, label='Start')
plt.scatter(lons[-1], lats[-1], color='red', s=100, label='End')

# Annotate waypoints
for i, (lat, lon) in enumerate(data):
    plt.text(lon, lat, str(i), fontsize=6, ha='right')

plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Plotted Path from QGC WPL Data")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()