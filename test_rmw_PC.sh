#!/bin/zsh

# Lista de RMWs a probar
RMW_LIST=("rmw_zenoh_cpp" "zenoh-bridge" "rmw_cyclonedds_cpp" "rmw_fastrtps_cpp")
DURATION=20  # segundos que se escucha
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

wait_for_topics2() {
    echo "🔎 Esperando a que los topics estén disponibles..."

    for i in {1..60}; do
        HAS_IMAGE=$(ros2 topic list 2>/dev/null | grep -q "/image_compressed" && echo "1" || echo "0")
        HAS_LIVOX=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar" && echo "1" || echo "0")
        HAS_COMPRESSED=$(ros2 topic list 2>/dev/null | grep -q "/livox/lidar/compressed" && echo "1" || echo "0")
        HAS_DRACO=$(ros2 topic list 2>/dev/null | grep -q "/pointcloud_draco" && echo "1" || echo "0")


        if [[ "$HAS_IMAGE" == "1" && "$HAS_LIVOX" == "1" && "$HAS_COMPRESSED" == "1" && "$HAS_DRACO" == "1"  ]]; then
            echo "✅ Todos los topics disponibles"
            sleep 5
            return 0
        fi

        echo "⏳ Esperando ($i)..."
        sleep 1
    done

    echo "❌ No se detectaron los topics tras 60 segundos"
    return 1
}

mover_pcaps() {
    echo "📦 Moviendo archivos .pcap desde /tmp a $LOG_DIR"

    local files=(/tmp/*.pcap)

    if [[ ${#files[@]} -eq 0 ]]; then
        echo "⚠️ No se encontraron archivos .pcap en /tmp"
        return
    fi

    for file in "${files[@]}"; do
        echo "➡️  Moviendo $(basename "$file")"
        sudo mv "$file" "$LOG_DIR/"
    done

    # Aseguramos que el usuario tenga permisos sobre los archivos
    sudo chown $USER:$USER "$LOG_DIR"/*.pcap

    echo "✅ Todos los archivos .pcap fueron movidos"
}


for RMW in "${RMW_LIST[@]}"; do
    echo "\n🔧 Probando RMW: $RMW"
    export RMW_IMPLEMENTATION=$RMW
    # Cambiamos el dominio para evitar conflictos
    export ROS_DOMAIN_ID=80
    echo $ROS_DOMAIN_ID
    
    if [ "$RMW" = "rmw_cyclonedds_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/rmw_logs/Config/cyclonedds.xml
        echo $CYCLONEDDS_URI
        # Cambiamos el dominio para evitar conflictos
        export ROS_DOMAIN_ID=81
        echo $ROS_DOMAIN_ID
    fi

  if [ "$RMW" = "rmw_zenoh_cpp" ]; then
     # Antes de lanzar rmw_zenohd
        if lsof -i :7447 > /dev/null; then
            echo "⚠️ El puerto 7447 ya está en uso. Cerrando procesos antiguos..."
            sudo fuser -k 7447/tcp
            sleep 2
        fi
        # Cambiamos el dominio para evitar conflictos
        export ROS_DOMAIN_ID=82
        echo $ROS_DOMAIN_ID        
        echo "🔧 Cargando archivo de configuración para Zenoh..."
        export ZENOH_ROUTER_CONFIG_URI=$HOME/rmw_logs/Config/router_config.json5
        cd ~/ros2_ws
        source install/setup.zsh
        ros2 run rmw_zenoh_cpp rmw_zenohd &
        ZENOH_PID=$!
        sleep 5
    fi

    if [ "$RMW" = "zenoh-bridge" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/rmw_logs/Config/cyclonedds.xml
        echo $CYCLONEDDS_URI
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        # Cambiamos el dominio para evitar conflictos
        export ROS_DOMAIN_ID=83
        echo $ROS_DOMAIN_ID

        cd ~/ros2_ws
        source install/setup.zsh
        echo "🔧 Arrancando bridge Zenoh sobre rmw_cyclonedds_cpp..."
        zenoh-bridge-ros2dds > /dev/null 2>&1 &
        ZENOH_BRIDGE_PID=$!
        sleep 2
    fi

    # Parar el daemon para evitar caché anterior
    ros2 daemon stop
    sleep 2

    # Esperar a que ambos topics estén disponibles
    if ! wait_for_topics; then
        echo "⚠️ Saltando $RMW por timeout en topics"
        continue
    fi


    # Lanzamos subscriber de imágenes
    echo "🚀 Lanzando nodo de imágenes..."
    cd ~/ros2_ws
    source install/setup.zsh
    sleep 1
    ros2 run prueba_rmw image_subscriber_compressed &
    SUB_PID=$!
    sleep 2

    echo "Lanzamos RViz con la configuración de Lidar..."
    RVIZ_CONFIG_PATH="$HOME/rmw_logs/Config/livox-lidar.rviz"
    # Verifica que el archivo de configuración exista
    if [[ -f "$RVIZ_CONFIG_PATH" ]]; then
        echo "🚀 Lanzando RViz con configuración predefinida..."
        rviz2 -d "$RVIZ_CONFIG_PATH" &
        RVIZ_PID=$!
        sleep 1
    else
        echo "⚠️ No se encontró el archivo de configuración de RViz en: $RVIZ_CONFIG_PATH"
    fi
    sleep 2


    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${RMW}1.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"  # Archivo para guardar el PID

        # Ejecutar tcpdump en segundo plano y guardar su PID
        sudo tcpdump -i enxc84d44228958 -w "$TCPDUMP_TMP" &
        TCPDUMP_PID=$!
        echo "$TCPDUMP_PID" > "$TCPDUMP_PID_FILE"
        echo "Captura guardada en: $TCPDUMP_TMP (PID: $TCPDUMP_PID)"
        sleep 2
    ) &


    (
        sleep 13
        echo "🛑 Deteniendo captura de paquetes (auto)"
        # Leer el PID desde el archivo
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"
        if [ -f "$TCPDUMP_PID_FILE" ]; then
            TCPDUMP_PID=$(cat "$TCPDUMP_PID_FILE")
            if ps -p "$TCPDUMP_PID" > /dev/null; then
                sudo kill -SIGINT "$TCPDUMP_PID"
                echo "✅ Captura de paquetes cerrada (PID: $TCPDUMP_PID)"
            else
                echo "⚠️ tcpdump ya no está corriendo"
            fi
            rm "$TCPDUMP_PID_FILE"
            echo "PID eliminado"
        else
            echo "❌ Archivo PID no encontrado"
        fi
    ) &

    sleep 4
    echo "📏 Midiendo frecuencia y delay de la imagen..."
    timeout ${DURATION}s ros2 topic hz /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_hz.txt" &
    HZ_IMAGE_PID=$!
    echo "📏 Midiendo frecuencia y delay del lidar..."
    timeout ${DURATION}s ros2 topic hz /livox/lidar -w 30 | tee "$LOG_DIR/${RMW}_lidar_hz.txt" &
    HZ_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic delay /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_delay.txt" &
    DELAY_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic delay /livox/lidar -w 30 | tee "$LOG_DIR/${RMW}_lidar_delay.txt" &
    DELAY_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic bw /image_compressed | tee "$LOG_DIR/${RMW}_image_bw.txt" &
    BW_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic bw /livox/lidar | tee "$LOG_DIR/${RMW}_lidar_bw.txt" &
    BW_LIDAR_PID=$!

    # Esperar a que se complete el tiempo
    wait $HZ_IMAGE_PID
    wait $HZ_LIDAR_PID
    wait $DELAY_IMAGE_PID
    wait $DELAY_LIDAR_PID
    wait $BW_IMAGE_PID
    wait $BW_LIDAR_PID

    # Cerramos RViz para cargar la otra configuración
    kill $RVIZ_PID

    sleep 3
    # Lanzar nodos de republisher
    echo "🚀 Lanzando nodo de republisher con $RMW..."
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run point_cloud_transport republish --ros-args -p out_transport:=raw -p in_transport:=draco --ros-args --remap in/draco:=/pointcloud_draco --remap out:=/livox/lidar/compressed &    
    REPUBLISHER_PID=$!
    sleep 2  # tiempo para que inicien
    echo "Nodo de republisher ejecutandose"


    # Esperar a que todos los topics estén disponibles
    if ! wait_for_topics2; then
        echo "⚠️ Saltando $RMW por timeout en topics"
        continue
    fi

    # Lanzamos RViz con la nueva configuración
    RVIZ_CONFIG_PATH="$HOME/rmw_logs/Config/livox-lidar-compressed.rviz"
    # Verifica que el archivo de configuración exista
    if [[ -f "$RVIZ_CONFIG_PATH" ]]; then
        echo "🚀 Lanzando RViz con configuración predefinida..."
        rviz2 -d "$RVIZ_CONFIG_PATH" &
        RVIZ_PID=$!
    else
        echo "⚠️ No se encontró el archivo de configuración de RViz en: $RVIZ_CONFIG_PATH"
    fi
    sleep 1

    (
        sleep 7
        echo "📡 Iniciando captura de paquetes con tcpdump..."
        TCPDUMP_TMP="/tmp/${RMW}2.pcap"
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"  # Archivo para guardar el PID

        # Ejecutar tcpdump en segundo plano y guardar su PID
        sudo tcpdump -i enxc84d44228958 -w "$TCPDUMP_TMP" &
        TCPDUMP_PID=$!
        echo "$TCPDUMP_PID" > "$TCPDUMP_PID_FILE"
        echo "Captura guardada en: $TCPDUMP_TMP (PID: $TCPDUMP_PID)"
        sleep 2
    ) &


    (
        sleep 17
        echo "🛑 Deteniendo captura de paquetes (auto)"
        # Leer el PID desde el archivo
        TCPDUMP_PID_FILE="/tmp/tcpdump.pid"
        if [ -f "$TCPDUMP_PID_FILE" ]; then
            TCPDUMP_PID=$(cat "$TCPDUMP_PID_FILE")
            if ps -p "$TCPDUMP_PID" > /dev/null; then
                sudo kill -SIGINT "$TCPDUMP_PID"
                echo "✅ Captura de paquetes cerrada (PID: $TCPDUMP_PID)"
            else
                echo "⚠️ tcpdump ya no está corriendo"
            fi
            rm "$TCPDUMP_PID_FILE"
            echo "PID eliminado"
        else
            echo "❌ Archivo PID no encontrado"
        fi
    ) &


    # Medimos frecuencia y delay de los topics
    echo "📏 Midiendo frecuencia y delay de la imagen..."
    timeout ${DURATION}s ros2 topic hz /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_hz2.txt" &
    HZ_IMAGE_PID=$!
    echo "📏 Midiendo frecuencia y delay del lidar comprimido..."
    timeout ${DURATION}s ros2 topic hz /livox/lidar/compressed -w 30 | tee "$LOG_DIR/${RMW}_lidar_compressed_hz.txt" &
    HZ_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic delay /image_compressed -w 30 | tee "$LOG_DIR/${RMW}_image_delay2.txt" &
    DELAY_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic delay /livox/lidar/compressed -w 30 | tee "$LOG_DIR/${RMW}_lidar_compressed_delay.txt" &
    DELAY_LIDAR_PID=$!

    timeout ${DURATION}s ros2 topic bw /image_compressed | tee "$LOG_DIR/${RMW}_image_bw2.txt" &
    BW_IMAGE_PID=$!
    timeout ${DURATION}s ros2 topic bw /livox/lidar/compressed | tee "$LOG_DIR/${RMW}_lidar_compressed_bw.txt" &
    BW_LIDAR_PID=$!

    # Esperar a que se complete el tiempo
    wait $HZ_IMAGE_PID
    wait $HZ_LIDAR_PID
    
    wait $DELAY_IMAGE_PID
    wait $DELAY_LIDAR_PID  .

    wait $BW_IMAGE_PID
    wait $BW_LIDAR_PID


    echo "🛑 Deteniendo nodos..."
    # Detenemos el subscriber de imágenes
    pkill -f image_subscriber_compressed
    sleep 1

    # Espera hasta que el proceso se cierre completamente
    while pgrep -f image_subscriber_compressed > /dev/null; do
        echo "⏳ Esperando que image_subscriber termine..."
        sleep 1
    done
    echo "✅ Nodo image_subscriber cerrado"


    kill $RVIZ_PID
    echo "✅ RViz cerrado"

    pkill -f point_cloud_transport
    sleep 1
    while pgrep -f point_cloud_republisher > /dev/null; do
        echo "⏳ Esperando que republisher termine..."
        sleep 1
    done
    echo "✅ republisher cerrado"


    if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        sleep 2
        pkill -f zenoh
        sleep 1
        while pgrep -f rmw_zenohd > /dev/null; do
            echo "⏳ Esperando que router zenoh termine..."
            sleep 1
        done
        echo "✅ Nodo rmw_zenohd cerrado"
    fi


    if [ "$RMW" = "zenoh-bridge" ]; then
        sleep 2
        pkill -f /zenoh_bridge_ros2dds
        sleep 1
        while pgrep -f /zenoh_bridge_ros2dds > /dev/null; do
            echo "⏳ Esperando que zenoh-bridge termine..."
            sleep 1
        done
        echo "✅ Nodo zenoh-bridge cerrado"
    fi

    echo "✅ Medición para $RMW completada"
    sleep 7
done

echo "\n📁 Pruebas completas. Resultados guardados en: $LOG_DIR"

# Mover los archivos pcap a la carpeta de resultados
mover_pcaps

# Graficamos los resultados
echo "📊 Graficando resultados..."
cd $HOME/rmw_logs
python3 graficos_rmw.py
