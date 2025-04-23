# rmw_logs
Comparación de RMW con gráficas

Los ejecutables deben iniciarse uno en el PC y otro en la raspy y al mismo tiempo.

EJECUTABLES: 

- test_rwm_PC.sh solamente guarda ficheros con los datos de frecuencia y delay del topic /image_compressed y /livox/lidar y abre subscriber de imagenes y RViz. No realiza gráficas al final. Ejecutar en el PC.

- test_rwm_raspy.sh solo lanza publisher de imágenes y de datos del lidar. Ejecutar en la raspberry

- test_rmw_PC_pcap.sh además de hacer lo mismo que test_rwm_PC.sh, también captura los paquetes mientras se mide la frecuencia de publicación y el delay. Ejecutar en el PC junto con test_rmw_raspy.sh en la raspy

- test_rmw_raspy_completo.sh ejecuta publisher de imágenes y publisher de lidar y cuando ha pasado cierto tiempo se inicia también republisher de lidar para comprimir la nube de puntos. 

- test_rmw_PC_pcap_completo.sh guarda ficherosde hz y delay de /image_compressed y /livox/lidar al mismo tiempo que captura paquetes; lanza subscriber de imagenes y RViz; ejecuta nodo republisher para descomprimir nube de puntos; guarda ficheros de hz y delay de /image_compressed y /livox/lidar/compressed al mismo tiempo que captura paquetes; lanza subscriber y RViz; y por último, cuando se han probado todos los RMW, se crean las gráficas con los resultados de frecuencia y delay comparando los RMW.
