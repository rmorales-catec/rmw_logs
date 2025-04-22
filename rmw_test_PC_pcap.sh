#!/bin/zsh

# Lista de RMWs a probar
RMW_LIST=("rmw_zenoh_cpp" "rmw_fastrtps_cpp" "rmw_cyclonedds_cpp")
DURATION=60  # segundos que se escucha
LOG_DIR="$HOME/rmw_logs/Resultados"

mkdir -p "$LOG_DIR"
cd ~/ros2_ws
source install/setup.zsh

# Función que espera a que existan ambos topics
wait_for_topics() {
    echo "🔎 Esperando a que los topics estén disponibles..."

    for i in {1..60}; do
        HAS_IMAGE=$(ros2 topic list 2>/dev/null | grep -q "/image_compressed" && echo "1" || echo "0")
        HAS_LIVOX=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "1" || echo "0")

        if [[ "$HAS_IMAGE" == "1" && "$HAS_LIVOX" == "1" ]]; then
            echo "✅ Ambos topics disponibles"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ No se detectaron ambos topics tras 60 segundos"
    return 1
}

for RMW in "${RMW_LIST[@]}"; do
    echo "\n🔧 Probando RMW: $RMW"
    export RMW_IMPLEMENTATION=$RMW
    
    if [ "$RMW" = "rmw_cyclonedds_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/Desktop/cyclonedds.xml
        echo $CYCLONEDDS_URI
    fi

  if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Zenoh..."
        cd ~/ros2_ws
        ros2 run rmw_zenoh_cpp rmw_zenohd &
        ZENOH_PID=$!
        sleep 3
    fi

    # Parar el daemon para evitar caché anterior
    ros2 daemon stop
    sleep 1

    # Esperar a que ambos topics estén disponibles
    if ! wait_for_topics; then
        echo "⚠️ Saltando $RMW por timeout en topics"
        continue
    fi

    RVIZ_CONFIG_PATH="$HOME/rmw_logs/Config/livox-lidar.rviz"
    # Verifica que el archivo de configuración exista
    if [[ -f "$RVIZ_CONFIG_PATH" ]]; then
        echo "🚀 Lanzando RViz con configuración predefinida..."
        rviz2 -d "$RVIZ_CONFIG_PATH" &
        RVIZ_PID=$!
    else
        echo "⚠️ No se encontró el archivo de configuración de RViz en: $RVIZ_CONFIG_PATH"
    fi
    sleep 1

    # Lanzamos subscriber de imágenes
    echo "🚀 Lanzando nodo de imágenes..."
    ros2 run prueba_qos image_subscriber_QoS_compressed &
    SUB_PID=$!
    sleep 1

    # --- 🔴 INICIO DE CAPTURA DE PAQUETES ---
    echo "📡 Iniciando captura de paquetes con tcpdump..."
    # TCPDUMP_FILE="$LOG_DIR/${RMW}.pcap"
    # sudo tcpdump -i any -w "$TCPDUMP_FILE" port 7400 or port 7410 or udp > /dev/null 2>&1 &
    # En vez de guardar directamente:
    TCPDUMP_TMP="/tmp/${RMW}.pcap"
    sudo tcpdump -i any -w "$TCPDUMP_TMP" port 7400 or port 7410 or udp > /dev/null 2>&1 &
    TCPDUMP_PID=$!
    # Aseguramos que tcpdump arranque correctamente
    sleep 2
    # ----------------------------------------

    # Programamos detenerlo después del tiempo de duración
    (
        sleep 10
        echo "🛑 Deteniendo captura de paquetes (auto)"
        sudo kill -SIGINT $TCPDUMP_PID 2>/dev/null
    ) &

    echo "📏 Midiendo frecuencia y delay de la imagen..."
    timeout ${DURATION}s ros2 topic hz /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_hz.txt" &
    HZ_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic delay /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_delay.txt" &
    DELAY_IMAGE_PID=$!
    echo "📏 Midiendo frecuencia y delay del lidar..."
    timeout ${DURATION}s ros2 topic hz /livox/lidar -w 30 | tee "$LOG_DIR/${RMW}_lidar_hz.txt" &
    HZ_LIDAR_PID=$!
    timeout ${DURATION}s ros2 topic delay /livox/lidar -w 30 | tee "$LOG_DIR/${RMW}_lidar_delay.txt" &
    DELAY_LIDAR_PID=$!

    wait $HZ_IMAGE_PID
    wait $DELAY_IMAGE_PID
    wait $HZ_LIDAR_PID
    wait $DELAY_LIDAR_PID

    # kill $SUB_PID
    pkill -f image_subscriber_QoS_compressed
    sleep 1

    # Espera hasta que el proceso se cierre completamente
    while pgrep -f image_subscriber_QoS_compressed > /dev/null; do
        echo "⏳ Esperando que image_subscriber termine..."
        sleep 1
    done
    
    echo "✅ Nodo image_subscriber cerrado"
    kill $RVIZ_PID

    if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        # pkill -f rmw_zenohd
        kill $ZENOH_PID
    fi

    # Movemos los archivos pcap a la carpeta de resultados
    sudo mv "$TCPDUMP_TMP" "$LOG_DIR/"
    cd ~/rmw_logs/Resultados
    sudo chown rmorales:rmorales *.pcap

    echo "✅ Medición para $RMW completada"
    sleep 5
done

echo "\n📁 Pruebas completas. Resultados guardados en: $LOG_DIR"

# Graficamos los resultados
echo "📊 Graficando resultados..."
cd $HOME/rmw_logs
python3 graficos_rmw.py
