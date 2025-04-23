# import pyshark

# pcap_file = '/home/rmorales/rmw_logs/Resultados/rmw_fastrtps_cpp.pcap'
# cap = pyshark.FileCapture(pcap_file)

# # Verificamos los primeros paquetes y sus atributos
# for packet in cap:
#     print(packet)  # Imprime el paquete completo
#     print(f"Longitud del paquete: {packet.length}")  # Imprime el tamaño del paquete




# import pyshark

# pcap_file = '/home/rmorales/rmw_logs/Resultados/rmw_fastrtps_cpp.pcap'
# cap = pyshark.FileCapture(pcap_file, only_summaries=False)  # Captura toda la información

# # Imprime los primeros 10 paquetes y sus atributos
# for i, packet in enumerate(cap):
#     if i < 10:
#         print(f"Paquete #{i+1}: {packet}")
#         print(f"Longitud del paquete: {packet.length}")
#     else:
#         break




# import pyshark
# import os
# import matplotlib.pyplot as plt

# # Lista fija de archivos pcap (nombres conocidos)
# RMW_FILES = [
#     # "rmw_fastrtps_cpp.pcap"
#     "rmw_fastrtps_cpp.pcap",
#     "rmw_cyclonedds_cpp.pcap",
#     "rmw_zenoh_cpp.pcap"
# ]

# # Ruta base donde están guardados
# LOG_DIR = os.path.expanduser("~/rmw_logs/Resultados")

# # Función para extraer tamaños de paquetes
# def extract_packet_sizes(pcap_path):
#     print(f"📂 Abriendo {pcap_path}")
#     try:
#         cap = pyshark.FileCapture(pcap_path, display_filter="udp", only_summaries=False)
#         sizes = []
#         for pkt in cap:
#             try:
#                 print(f"Longitud del paquete: {pkt.length}")  # Imprime el tamaño del paquete
#                 sizes.append(int(pkt.length))
#             except AttributeError:
#                 continue
#         cap.close()
#         return sizes
#     except Exception as e:
#         print(f"❌ Error al procesar {pcap_path}: {e}")
#         return []

# # Crear gráfico
# plt.figure(figsize=(10, 6))
# any_data = False

# for filename in RMW_FILES:
#     full_path = os.path.join(LOG_DIR, filename)

#     if not os.path.exists(full_path):
#         print(f"⚠️ Archivo no encontrado: {full_path}")
#         continue

#     packet_sizes = extract_packet_sizes(full_path)

#     if packet_sizes:
#         plt.plot(packet_sizes, label=filename)
#         any_data = True
#     else:
#         print(f"⚠️ No se extrajeron datos de {filename}")

# # Mostrar gráfico si hay datos
# if any_data:
#     plt.title("📊 Tamaño de paquetes capturados por RMW")
#     plt.xlabel("Número de Paquete")
#     plt.ylabel("Tamaño (bytes)")
#     plt.legend()
#     plt.grid(True)
#     plt.tight_layout()
#     plt.show()
# else:
#     print("⚠️ No se pudo graficar: no hay datos válidos.")





# import pyshark
# import os
# import matplotlib.pyplot as plt

# # Lista fija de archivos pcap
# RMW_FILES = [
#     "rmw_fastrtps_cpp.pcap",
#     "rmw_cyclonedds_cpp.pcap",
#     "rmw_zenoh_cpp.pcap"
# ]

# # Ruta base
# LOG_DIR = os.path.expanduser("~/rmw_logs/Resultados")

# # Datos para graficar
# rmw_labels = []
# avg_sizes = []

# # Función para extraer tamaños
# def extract_packet_sizes(pcap_path):
#     try:
#         cap = pyshark.FileCapture(pcap_path, display_filter="udp", only_summaries=False, keep_packets=False)
#         sizes = []
#         for pkt in cap:
#             try:
#                 sizes.append(int(pkt.length))
#             except AttributeError:
#                 continue
#         cap.close()
#         return sizes
#     except Exception as e:
#         print(f"❌ Error al procesar {pcap_path}: {e}")
#         return []

# # Procesar cada archivo
# for filename in RMW_FILES:
#     full_path = os.path.join(LOG_DIR, filename)

#     if not os.path.exists(full_path):
#         print(f"⚠️ Archivo no encontrado: {full_path}")
#         continue

#     packet_sizes = extract_packet_sizes(full_path)

#     if packet_sizes:
#         promedio = sum(packet_sizes) / len(packet_sizes)
#         rmw_labels.append(filename.replace(".pcap", ""))
#         avg_sizes.append(promedio)
#         print(f"✅ {filename}: {len(packet_sizes)} paquetes, tamaño medio = {promedio:.2f} bytes")
#     else:
#         print(f"⚠️ No se extrajeron datos de {filename}")

# # Graficar resultados
# if avg_sizes:
#     plt.figure(figsize=(8, 5))
#     bars = plt.bar(rmw_labels, avg_sizes, color='cornflowerblue', edgecolor='black')
#     for bar, val in zip(bars, avg_sizes):
#         plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5, f"{val:.1f}", ha='center')
#     plt.title("📊 Tamaño medio de paquetes UDP por RMW")
#     plt.ylabel("Tamaño promedio (bytes)")
#     plt.grid(axis='y')
#     plt.tight_layout()
#     plt.show()
# else:
#     print("⚠️ No se pudo graficar: no hay datos válidos.")





# import pyshark
# import os
# import matplotlib.pyplot as plt
# import numpy as np

# RMW_FILES = [
#     "rmw_fastrtps_cpp.pcap",
#     "rmw_cyclonedds_cpp.pcap",
#     "rmw_zenoh_cpp.pcap"
# ]

# LOG_DIR = os.path.expanduser("~/rmw_logs/Resultados")

# rmw_labels = []
# avg_sizes = []
# total_packets = []
# estimated_loss = []

# def extract_packet_info(pcap_path):
#     try:
#         cap = pyshark.FileCapture(pcap_path, display_filter="udp", only_summaries=False, keep_packets=False)
#         sizes = []
#         timestamps = []

#         for pkt in cap:
#             try:
#                 sizes.append(int(pkt.length))
#                 timestamps.append(float(pkt.sniff_timestamp))
#             except AttributeError:
#                 continue
#         cap.close()
#         return sizes, timestamps
#     except Exception as e:
#         print(f"❌ Error al procesar {pcap_path}: {e}")
#         return [], []

# for filename in RMW_FILES:
#     full_path = os.path.join(LOG_DIR, filename)

#     if not os.path.exists(full_path):
#         print(f"⚠️ Archivo no encontrado: {full_path}")
#         continue

#     sizes, timestamps = extract_packet_info(full_path)

#     if sizes and len(timestamps) > 1:
#         promedio = sum(sizes) / len(sizes)
#         rmw_labels.append(filename.replace(".pcap", ""))
#         avg_sizes.append(promedio)
#         total_packets.append(len(sizes))

#         timestamps = np.array(sorted(timestamps))
#         deltas = np.diff(timestamps)
#         duration = timestamps[-1] - timestamps[0]
#         expected = int(duration / np.median(deltas))
#         loss = max(0, expected - len(sizes))
#         estimated_loss.append(loss)

#         print(f"✅ {filename}: {len(sizes)} paquetes, tamaño medio = {promedio:.2f} bytes, pérdida estimada = {loss}")
#     else:
#         print(f"⚠️ No se extrajeron datos de {filename}")

# # --- Gráfico 1: Tamaño medio ---
# if avg_sizes:
#     plt.figure(figsize=(8, 5))
#     bars = plt.bar(rmw_labels, avg_sizes, color='cornflowerblue', edgecolor='black')
#     for bar, val in zip(bars, avg_sizes):
#         plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5, f"{val:.1f}", ha='center')
#     plt.title("📊 Tamaño medio de paquetes UDP por RMW")
#     plt.ylabel("Tamaño promedio (bytes)")
#     plt.grid(axis='y')
#     plt.tight_layout()
#     plt.show()

# # --- Gráfico 2: Total de paquetes y pérdida estimada ---
# if total_packets:
#     x = np.arange(len(rmw_labels))
#     width = 0.35

#     plt.figure(figsize=(8, 5))
#     bar1 = plt.bar(x - width/2, total_packets, width, label='Capturados', color='lightgreen', edgecolor='black')
#     bar2 = plt.bar(x + width/2, estimated_loss, width, label='Perdidos (estimado)', color='salmon', edgecolor='black')

#     for i, val in enumerate(total_packets):
#         plt.text(x[i] - width/2, val + 10, str(val), ha='center')
#     for i, val in enumerate(estimated_loss):
#         plt.text(x[i] + width/2, val + 10, str(val), ha='center')

#     plt.xticks(x, rmw_labels)
#     plt.ylabel("Número de paquetes")
#     plt.title("📦 Paquetes capturados vs. pérdida estimada por RMW")
#     plt.legend()
#     plt.grid(axis='y')
#     plt.tight_layout()
#     plt.show()


import pyshark
import os
import matplotlib.pyplot as plt

RMW_FILES = [
    "rmw_fastrtps_cpp1.pcap",
    "rmw_cyclonedds_cpp1.pcap",
    "rmw_zenoh_cpp1.pcap",
    "rmw_fastrtps_cpp2.pcap",
    "rmw_cyclonedds_cpp2.pcap",
    "rmw_zenoh_cpp2.pcap"
]

LOG_DIR = os.path.expanduser("~/rmw_logs/Resultados")

# Métricas por RMW
rmw_names = []
avg_sizes = []
packet_counts = []
total_bytes = []

def extract_packet_metrics(pcap_path):
    try:
        cap = pyshark.FileCapture(pcap_path, display_filter="udp", only_summaries=False)
        sizes = []
        for pkt in cap:
            try:
                sizes.append(int(pkt.length))
            except AttributeError:
                continue
        cap.close()
        return sizes
    except Exception as e:
        print(f"❌ Error al procesar {pcap_path}: {e}")
        return []

# Recolectar métricas
for filename in RMW_FILES:
    full_path = os.path.join(LOG_DIR, filename)
    if not os.path.exists(full_path):
        print(f"⚠️ Archivo no encontrado: {full_path}")
        continue

    sizes = extract_packet_metrics(full_path)
    if sizes:
        avg = sum(sizes) / len(sizes)
        rmw_names.append(filename.replace(".pcap", ""))
        avg_sizes.append(avg)
        packet_counts.append(len(sizes))
        total_bytes.append(sum(sizes))  # Total en bytes
    else:
        print(f"⚠️ No se extrajeron datos de {filename}")

# Función para graficar
def plot_bar(values, labels, ylabel, title, fmt="int", filename=None):
    plt.figure(figsize=(9, 5))
    bars = plt.bar(labels, values, color='skyblue', edgecolor='black')
    for bar, val in zip(bars, values):
        label = f"{val:.0f}" if fmt == "int" else f"{val:.2f}"
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1, label, ha='center')
    plt.title(title)
    plt.ylabel(ylabel)
    plt.grid(axis='y')
    plt.tight_layout()
    if filename:
        plt.savefig(filename)
    plt.show()

# 📊 Graficar todo
plot_bar(avg_sizes, rmw_names, "Tamaño medio (bytes)", "Tamaño medio de paquetes", fmt="float", filename="avg_packet_size.png")
plot_bar(packet_counts, rmw_names, "Número de paquetes", "Total de paquetes enviados", fmt="int", filename="packet_count.png")
plot_bar(total_bytes, rmw_names, "Total de bytes", "Total de bytes transmitidos", fmt="int", filename="total_bytes.png")
