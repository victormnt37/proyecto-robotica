from flask import Flask, jsonify, request
import sqlite3
from flask_cors import CORS

app = Flask(__name__)
CORS(app)  # Habilita CORS para todas las rutas

# ------------------------
# CONEXIÓN A LA BASE DE DATOS
# ------------------------

def get_db_connection():
    conn = sqlite3.connect('hospitalBDD.db')  # Asegúrate que el nombre es correcto
    conn.row_factory = sqlite3.Row
    return conn

# ------------------------
# LOGIN API
# ------------------------

@app.route('/api/login', methods=['POST'])
def login():
    data = request.json
    correo_electronico = data.get('correo_electronico')
    contrasena = data.get('contrasena') 

    conn = get_db_connection()
    cursor = conn.cursor()

    # Verificar en tabla Paciente
    cursor.execute("SELECT * FROM Paciente WHERE correo_electronico = ? AND contrasena = ?", (correo_electronico, contrasena))
    if cursor.fetchone():
        conn.close()
        return jsonify({"status": "success", "role": "paciente","correo":correo_electronico})

    # Verificar en tabla Medico
    cursor.execute("SELECT * FROM Medico WHERE correo_electronico = ? AND contrasena = ?", (correo_electronico, contrasena))
    if cursor.fetchone():
        conn.close()
        
        return jsonify({"status": "success", "role": "medico","correo":correo_electronico})

    # Verificar en tabla Administrador
    cursor.execute("SELECT * FROM Administrador WHERE correo_electronico = ? AND contrasena = ?", (correo_electronico, contrasena))
    if cursor.fetchone():
        conn.close()
        return jsonify({"status": "success", "role": "admin","correo":correo_electronico})

    conn.close()
    return jsonify({"status": "error", "message": "Credenciales inválidas"}), 401

# ------------------------
# API: OBTENER TODAS LAS CONVERSACIONES
# ------------------------

@app.route('/api/conversaciones', methods=['GET'])
def obtener_conversaciones():
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT id_conversacion, id_paciente, fecha, contenido FROM Conversacion")
    conversaciones = cursor.fetchall()
    conn.close()

    resultado = []
    for conv in conversaciones:
        resultado.append({
            "id_conversacion": conv["id_conversacion"],
            "id_paciente": conv["id_paciente"],
            "autor": "Paciente",  # Si tienes autor real en otra tabla, deberías incluirlo
            "mensaje": conv["contenido"],
            "fecha": conv["fecha"]
        })

    return jsonify(resultado)

@app.route('/api/conversaciones_medico', methods=['GET'])
def obtener_conversaciones_medico():
    correo_electronico = request.args.get('correo')

    if not correo_electronico:
        return jsonify({"error": "Correo electrónico es necesario"}), 400

    conn = get_db_connection()
    cursor = conn.cursor()

    # Obtener el id_medico con el correo electrónico
    cursor.execute("SELECT id_medico FROM Medico WHERE correo_electronico = ?", (correo_electronico,))
    medico = cursor.fetchone()

    if not medico:
        conn.close()
        return jsonify({"error": "Médico no encontrado"}), 404

    id_medico = medico["id_medico"]

    # Obtener las conversaciones que están asociadas al id_medico (id_robot)
    cursor.execute("""
        SELECT c.id_conversacion, c.fecha, c.contenido, 
               p.dni, p.nombre 
        FROM Conversacion c
        JOIN Paciente p ON c.id_paciente = p.id_paciente
        WHERE c.id_robot = ?
    """, (id_medico,))
    conversaciones = cursor.fetchall()
    conn.close()

    resultado = []
    for conv in conversaciones:
        resultado.append({
            "id_conversacion": conv["id_conversacion"],
            "fecha": conv["fecha"],
            "mensaje": conv["contenido"],
            "dni": conv["dni"],
            "nombre": conv["nombre"]
        })

    return jsonify(resultado)

# API: BUSCAR CONVERSACIONES POR DNI
# ------------------------

@app.route('/api/conversaciones/<dni>', methods=['GET'])
def obtener_conversaciones_por_dni(dni):
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute("SELECT id_paciente FROM Paciente WHERE dni = ?", (dni,))
    paciente = cursor.fetchone()

    if not paciente:
        conn.close()
        return jsonify({"error": "Paciente no encontrado"}), 404

    id_paciente = paciente["id_paciente"]
    cursor.execute("SELECT id_conversacion, id_paciente, fecha, contenido FROM Conversacion WHERE id_paciente = ?", (id_paciente,))
    conversaciones = cursor.fetchall()
    conn.close()

    resultado = []
    for conv in conversaciones:
        resultado.append({
            "id_conversacion": conv["id_conversacion"],
            "id_paciente": conv["id_paciente"],
            "autor": "Paciente",  # Puedes modificar según tu lógica
            "mensaje": conv["contenido"],
            "fecha": conv["fecha"]
        })

    return jsonify(resultado)

@app.route('/api/obtener_conversaciones_por_dni2', methods=['GET'])
def obtener_conversaciones_por_dni2():
    dni_inicio = request.args.get("dni")  # Cambié "dni" por "dni_inicio" para que sea más claro
    correo = request.args.get("correo")

    if not dni_inicio or not correo:
        return jsonify({"error": "DNI y correo son requeridos"}), 400

    conn = get_db_connection()
    cursor = conn.cursor()

    # Obtener id_medico usando el correo del médico
    cursor.execute("SELECT id_medico FROM Medico WHERE correo_electronico = ?", (correo,))
    medico = cursor.fetchone()

    if not medico:
        conn.close()
        return jsonify({"error": "Médico no encontrado"}), 404

    id_medico = medico["id_medico"]

    # Buscar pacientes cuyos DNI comienzan con los dígitos proporcionados
    cursor.execute("""
        SELECT id_paciente FROM Paciente WHERE dni LIKE ? 
    """, (dni_inicio + '%',))  # '%' es el comodín de SQL para indicar que los caracteres posteriores pueden ser cualquier cosa

    pacientes = cursor.fetchall()

    if not pacientes:
        conn.close()
        return jsonify({"error": "No se encontraron pacientes con ese DNI"}), 404

    # Obtener las conversaciones asociadas a ese médico y los pacientes encontrados
    id_pacientes = [paciente["id_paciente"] for paciente in pacientes]
    cursor.execute("""
        SELECT c.id_conversacion, c.fecha, c.contenido, p.dni, p.nombre
        FROM Conversacion c
        JOIN Paciente p ON c.id_paciente = p.id_paciente
        WHERE c.id_robot = ? AND p.id_paciente IN ({})
    """.format(','.join(['?'] * len(id_pacientes))), [id_medico] + id_pacientes)

    conversaciones = cursor.fetchall()
    conn.close()

    resultado = []
    for conv in conversaciones:
        resultado.append({
            "id_conversacion": conv["id_conversacion"],
            "dni": conv["dni"],
            "nombre": conv["nombre"],
            "fecha": conv["fecha"],
            "mensaje": conv["contenido"]
        })

    return jsonify(resultado)

# ------------------------
# MAIN
# ------------------------
@app.route("/api/usuario_por_correo_y_rol", methods=["GET"])
def obtener_usuario_por_correo_y_rol():
    correo = request.args.get("correo")
    rol = request.args.get("rol")

    if not correo or not rol:
        return jsonify({"error": "Correo y rol son requeridos"}), 400

    conn = get_db_connection()
    cursor = conn.cursor()

    tabla = ""
    if rol == "paciente":
        tabla = "Paciente"
    elif rol == "medico":
        tabla = "Medico"
    elif rol == "admin":
        tabla = "Administrador"
    else:
        return jsonify({"error": "Rol inválido"}), 400

    try:
        cursor.execute(f"SELECT nombre FROM {tabla} WHERE correo_electronico = ?", (correo,))
        fila = cursor.fetchone()
        if fila:
            return jsonify({"nombre": fila["nombre"]})
        else:
            return jsonify({"error": "Usuario no encontrado"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    finally:
        conn.close()


@app.route('/api/conversacion/<int:id_conversacion>', methods=['GET'])
def obtener_conversacion(id_conversacion):
    conn = get_db_connection()
    cursor = conn.cursor()

    # Obtener los mensajes de la conversación específica
    cursor.execute("""
        SELECT contenido, fecha, tipo
        FROM Mensaje
        WHERE id_conversacion = ?
    """, (id_conversacion,))
    
    mensajes = cursor.fetchall()
    conn.close()

    resultado = []
    for mensaje in mensajes:
        resultado.append({
            "contenido": mensaje["contenido"],
            "fecha": mensaje["fecha"],
            "tipo": mensaje["tipo"],  # 'recibido' o 'enviado'
        })

    return jsonify(resultado)

if __name__ == '__main__':
    app.run(debug=True, port=5000)
