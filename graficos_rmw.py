import matplotlib.pyplot as plt
import os
import re

# log_dir = os.path.expanduser('~/rmw_logs/Resultados_pruebas/Resultados/Resultados25-04_2')
log_dir = os.path.expanduser('~/rmw_logs/Resultados')
rmws = ['rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp', 'rmw_zenoh_cpp', 'zenoh-bridge']

def parse_file(filepath, mode='hz'):
    values = []
    if not os.path.exists(filepath):
        print(f"⚠️  No existe el archivo {filepath}")
        return values
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if mode == 'hz':
                m = re.search(r'average rate:\s+([0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)
            else:  # delay
                m = re.search(r'average delay:\s+(-?[0-9.]+)', line)
                if m:
                    values.append(float(m.group(1)))
                elif 'no new messages' in line:
                    values.append(0.0)

    print(f"{os.path.basename(filepath)} → {len(values)} muestras")
    return values

# --- Preparar datos ---
data_image_hz = {}
data_image_delay = {}
data_lidar_hz = {}
data_lidar_delay = {}
data_image_hz2 = {}
data_image_delay2 = {}
data_lidar_compressed_hz = {}
data_lidar_compressed_delay = {}

for rmw in rmws:
    data_image_hz[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_image_hz.txt"), mode='hz')
    data_image_delay[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_image_delay.txt"), mode='delay')
    data_lidar_hz[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_lidar_hz.txt"), mode='hz')
    data_lidar_delay[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_lidar_delay.txt"), mode='delay')
    data_image_hz2[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_image_hz2.txt"), mode='hz')
    data_image_delay2[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_image_delay2.txt"), mode='delay')
    data_lidar_compressed_hz[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_lidar_compressed_hz.txt"), mode='hz')
    data_lidar_compressed_delay[rmw] = parse_file(os.path.join(log_dir, f"{rmw}_lidar_compressed_delay.txt"), mode='delay')

# --- Función para graficar ---
def plot_metric(data, title, ylabel, filename, marker='o', linestyle='--'):
    plt.figure(figsize=(10, 5))
    all_vals = [v for vals in data.values() for v in vals]
    if all_vals:
        mn, mx = min(all_vals), max(all_vals)
        margin = abs(0.1 * max(abs(mn), abs(mx)))
        plt.ylim(mn - margin, mx + margin)
    for rmw, vals in data.items():
        if not vals:
            continue
        x = list(range(len(vals)))
        plt.plot(x, vals, label=rmw, marker=marker, linestyle=linestyle, markersize=3)
        for i, v in enumerate(vals):
            if v == 0:
                plt.plot(i, 0, 'ro')
    plt.title(title)
    plt.xlabel("Muestra")
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    plt.show()

# --- Gráficas image ---
plot_metric(data_image_hz, "Frecuencia - image topic", "Hz", "frecuencia_image.png")
plot_metric(data_image_delay, "Delay - image topic", "s", "delay_image.png", marker='x', linestyle='--')

# --- Gráficas lidar ---
plot_metric(data_lidar_hz, "Frecuencia - livox/lidar topic", "Hz", "frecuencia_lidar.png")
plot_metric(data_lidar_delay, "Delay - livox/lidar topic", "s", "delay_lidar.png", marker='x', linestyle='--')

# --- Gráficas image ---
plot_metric(data_image_hz2, "Frecuencia - image topic", "Hz", "frecuencia_image2.png")
plot_metric(data_image_delay2, "Delay - image topic", "s", "delay_image2.png", marker='x', linestyle='--')

# --- Gráficas lidar comprimido ---
plot_metric(data_lidar_compressed_hz, "Frecuencia - livox/lidar/compressed topic", "Hz", "frecuencia_lidar_compressed.png")
plot_metric(data_lidar_compressed_delay, "Delay - livox/lidar/compressed topic", "s", "delay_lidar_compressed.png", marker='x', linestyle='--')
