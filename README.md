# rmw_logs
Comparación de las diferentes implementaciones de RMW con gráficas

## EJECUTABLES: 
- *test_rwm_PC.sh*: script para el PC. Se encarga de lanzar nodo de subscriber para imágenes, lanzar RViz para visualizar los datos del LiDAR, captura paquetes, toma datos de frecuencia, delay y ancho de banda consumido con cada topic y grafica los resultados de los datos obtenidos.
- *test_rmw_raspy.sh*: script para la Raspberry. Lanza el nodo del publisher de imágenes y el publisher del LiDAR.

 ## Programas de Python
 - *graficos_rmw.py*: se ejecuta automáticamente en el script del PC cuando han acabado todas las pruebas de los RMW. En total genera 12 gráficas.
 - *graficos_paquetes.py*: hay que ejecutarlo manualmente. Tarda bastante tiempo en leer las capturas de los paquetes por ser de gran tamaño. Genera 2 gráficas, una con el número total de paquetes que se han transmitido con cada RMW y otra con el tamaño medio de dichos paquetes. 
