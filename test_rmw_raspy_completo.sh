#!/bin/zsh

# 🚀 Lista de RMWs a probar
RMW_LIST=("rmw_cyclonedds_cpp" "rmw_fastrtps_cpp" "rmw_zenoh_cpp" "zenoh-bridge")

DURATION=10  # segundos que se escucha
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
    # Cambiamos el dominio para evitar conflictos
    export ROS_DOMAIN_ID=190
    echo $ROS_DOMAIN_ID

    if [ "$RMW" = "rmw_cyclonedds_cpp" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/Desktop/cyclonedds.xml
        # Cambiamos el dominio para evitar conflictos
        export ROS_DOMAIN_ID=189
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
        export ROS_DOMAIN_ID=191
        echo $ROS_DOMAIN_ID
    	sleep 2
        echo "🔧 Cargando archivo de configuración para Zenoh..."
        export ZENOH_ROUTER_CONFIG_URI=$HOME/Desktop/router_config.json5
        cd ~/ros2_ws
	source install/setup.zsh
        ros2 run rmw_zenoh_cpp rmw_zenohd &
        ZENOH_PID=$!
        sleep 5
    fi

    if [ "$RMW" = "zenoh-bridge" ]; then
        echo "🔧 Cargando archivo de configuración para Cyclone DDS..."
        export CYCLONEDDS_URI=file://$HOME/Desktop/cyclonedds.xml
        echo $CYCLONEDDS_URI
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        # Cambiamos el dominio para evitar conflictos
        export ROS_DOMAIN_ID=192
        echo $ROS_DOMAIN_ID

        cd ~/ros2_ws
        source install/setup.zsh
        echo "🔧 Arrancando bridge Zenoh sobre rmw_cyclonedds_cpp..."
        zenoh-bridge-ros2dds &
        ZENOH_BRIDGE_PID=$!
        sleep 3
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

    echo "Nodo de LiDAR"
    cd ~/ros2_ws/src/ws_livox
    source install/setup.zsh
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py &
    LIVOX_PID=$!
    sleep 5  # tiempo para que inicien
    echo "Nodo de LiDAR ejecutandose"

    sleep 30
	

    echo "🚀 Probamos con pointcloud comprimido"
    # Lanzar nodos en background
    echo "Nodo de republisher"
    cd ~/ros2_ws
    source install/setup.zsh
    ros2 run point_cloud_transport republish --ros-args -p out_transport:=draco -p in_transport:=raw --ros-args --log-level info  --ros-args -r __node:=point_cloud_republisher --remap in:=/livox/lidar --remap out/draco:=/pointcloud_draco &
    REPUBLISHER_PID=$!
    sleep 2  # tiempo para que inicien
    echo "Nodo de republisher ejecutandose"


    sleep 30


    # Finalizar nodos
    echo "🛑 Matando nodos..."

    # Cerrar el subscriber (usa cámara) completamente
    pkill -f image_publisher_QoS_compressed
    sleep 1
    while pgrep -f image_publisher_QoS_compressed > /dev/null; do
        echo "⏳ Esperando que image_publisher termine..."
        sleep 1
    done
    echo "✅ image_publisher cerrado"

    pkill -f livox_ros_driver2
    sleep 1
    while pgrep -f livox_ros_driver2 > /dev/null; do
        echo "⏳ Esperando que livox/lidar termine..."
        sleep 1
    done
    echo "✅ livox/lidar cerrado"


    pkill -f point_cloud_republisher
    sleep 1
    while pgrep -f point_cloud_republisher > /dev/null; do
        echo "⏳ Esperando que republisher termine..."
        sleep 1
    done
    echo "✅ republisher cerrado"

    if [ "$RMW" = "rmw_zenoh_cpp" ]; then
        pkill -f rmw_zenohd
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
        # kill $ZENOH_PID
        sleep 1
        while pgrep -f /zenoh_bridge_ros2dds > /dev/null; do
            echo "⏳ Esperando que zenoh-bridge termine..."
            sleep 1
        done
        echo "✅ Nodo zenoh-bridge cerrado"
    fi

    sleep 5
    echo "Prueba con $RMW terminada"
    sleep 2
done

echo "✅ Pruebas completadas. Resultados en: $LOG_DIR"
