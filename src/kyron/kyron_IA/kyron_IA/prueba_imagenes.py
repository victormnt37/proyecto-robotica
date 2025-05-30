from ultralytics import YOLO
import os

# Ruta absoluta de tu imagen de prueba y carpeta de resultados
imagen_prueba = r'C:\Users\bigli\Desktop\Programacion y Codigo\YOLO\Modelo_2\test\prueba4.jpg'
carpeta_salida = r'C:\Users\bigli\Desktop\Programacion y Codigo\YOLO\Modelo_2\test\res'


# Crear carpeta si no existe
os.makedirs(carpeta_salida, exist_ok=True)

# Cargar el modelo entrenado
model = YOLO(r'C:\Users\bigli\Desktop\Programacion y Codigo\YOLO\Modelo_2\model\train\weights\best.pt')

# Ejecutar la predicción
results = model(imagen_prueba)

# Guardar el resultado en la carpeta deseada
nombre_archivo_salida = os.path.join(carpeta_salida, 'resultado.jpg')
results[0].save(filename=nombre_archivo_salida)

print(f'Resultado guardado en: {nombre_archivo_salida}')
