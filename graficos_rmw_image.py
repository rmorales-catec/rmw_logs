import matplotlib.pyplot as plt
import os
import re

# 📁 Carpeta con los logs
log_dir = os.path.expanduser('~/rmw_logs/Resultados')
rmws = ['rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp', 'rmw_zenoh_cpp']

def parse_file(filepath, mode='hz'):
    """
    Mode 'hz': busca 'average rate:' y 'no new messages'
    Mode 'delay': busca 'average delay:' y 'no new messages'
    """
    values = []
    if not os.path.exists(filepath):
        print(f"⚠️  No existe el archivo {filepath}")
        return values
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if mode == 'hz':
                m = re.match(r'average rate:\s+([0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            else:  # mode == 'delay'
                m = re.match(r'average delay:\s+(-?[0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
    print(f"{os.path.basename(filepath)} → {len(values)} muestras")
    return values

# --- Preparar datos ---
data_hz = {}
data_delay = {}
for rmw in rmws:
    data_hz[rmw]    = parse_file(os.path.join(log_dir, f"{rmw}_image_hz.txt"), mode='hz')
    data_delay[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_image_delay.txt"), mode='delay')

# --- Gráfico de Frecuencia ---
plt.figure(figsize=(10, 5))
for rmw, hz_vals in data_hz.items():
    if not hz_vals:
        continue
    x = list(range(len(hz_vals)))
    plt.plot(x, hz_vals, label=rmw, marker='o', markersize=3)
    for i, v in enumerate(hz_vals):
        if v == 0:
            plt.plot(i, 0, 'ro')
plt.title("Evolución de la Frecuencia (Hz)")
plt.xlabel("Muestra")
plt.ylabel("Hz")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("frecuencia_por_tiempo.png")
plt.show()

# --- Gráfico de Delay ---
# Calculamos min/max globales para ajustar el eje Y
all_delays = [v for vals in data_delay.values() for v in vals]
if all_delays:
    mn, mx = min(all_delays), max(all_delays)
    margin = abs(0.1 * max(abs(mn), abs(mx)))
    y_min, y_max = mn - margin, mx + margin
else:
    y_min, y_max = -1, 1

plt.figure(figsize=(10, 5))
plt.ylim(y_min, y_max)

for rmw, d_vals in data_delay.items():
    if not d_vals:
        continue
    x = list(range(len(d_vals)))
    plt.plot(x, d_vals, label=rmw, marker='x', linestyle='--', markersize=3)
    for i, v in enumerate(d_vals):
        if v == 0:
            plt.plot(i, 0, 'ro')
plt.title("Evolución del Delay (s)")
plt.xlabel("Muestra")
plt.ylabel("s")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("delay_por_tiempo.png")
plt.show()
