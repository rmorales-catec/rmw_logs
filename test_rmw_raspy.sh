#!/bin/zsh

# 🚀 Lista de RMWs a probar
RMW_LIST=("rmw_fastrtps_cpp" "rmw_cyclonedds_cpp" "rmw_zenoh_cpp")

# 🧪 Configuración del test
# TOPIC_NAME="/image_compressed"
DURATION=10  # segundos que se escucha
# LAUNCH_CMD="ros2 launch my_package my_test.launch.py"
LOG_DIR="rmw_logs"

cd ~/ros2_ws
source install/setup.zsh

mkdir -p $LOG_DIR

for RMW in "${RMW_LIST[@]}"
do
    echo "🔧 Probando RMW: $RMW"
    
    # Setear el RMW
    export RMW_IMPLEMENTATION=$RMW
    echo $RMW_IMPLEMENTATION

    if [ "$RMW" = "rmw_cyclonedds_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/Desktop/cyclonedds.xml
    fi

    if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Zenoh..."
        export ZENOH_ROUTER_CONFIG_URI=$HOME/Desktop/router_config.json5
        cd ~/ros2_ws
        ros2 run rmw_zenoh_cpp rmw_zenohd &
        ZENOH_PID=$!
        sleep 2
    fi

    # Limpiar estado anterior
    echo "🧹 Limpiando estado anterior..."
    ros2 daemon stop
    sleep 2

    # Lanzar nodos en background
    echo "🚀 Lanzando nodos con $RMW..."
    echo "Nodo de imagenes"
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run prueba_qos image_publisher_QoS_compressed &
    IMAGE_PID=$!
    sleep 2  # tiempo para que inicien
    echo "Nodo de imagenes ejecutandose"

    echo "🚀 Lanzando nodos con $RMW..."
    echo "Nodo de imagenes"
    cd ~/ros2_ws/src/ws_livox
    source install/setup.zsh
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
    LIVOX_PID=$!
    sleep 5  # tiempo para que inicien
    echo "Nodo de LiDAR ejecutandose"

    sleep 68
	
    # Finalizar nodos
    echo "🛑 Matando nodos..."
    # kill $IMAGE_PID
    # kill $LIVOX_PID

    # Cerrar el subscriber (usa cámara) completamente
    pkill -f image_publisher_QoS_compressed
    sleep 1
    while pgrep -f image_publisher_QoS_compressed > /dev/null; do
        echo "⏳ Esperando que image_publisher termine..."
        sleep 1
    done
    echo "✅ image_publisher cerrado"

    pkill -f livox_ros_driver2

    if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        # pkill -f rmw_zenohd
        kill $ZENOH_PID
    fi

    sleep 5
    echo "Prueba con $RMW terminada"
done

echo "✅ Pruebas completadas. Resultados en: $LOG_DIR"
