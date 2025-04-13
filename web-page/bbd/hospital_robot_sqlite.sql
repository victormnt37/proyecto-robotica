
CREATE TABLE Paciente (
    id_paciente INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre VARCHAR(100) NOT NULL,
    fecha_nacimiento DATE,
    telefono VARCHAR(15),
    correo_electronico VARCHAR(100)
);

CREATE TABLE Medico (
    id_medico INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre VARCHAR(100) NOT NULL,
    especialidad VARCHAR(100),
    telefono VARCHAR(15),
    correo_electronico VARCHAR(100)
);

CREATE TABLE Administrador (
    id_admin INTEGER PRIMARY KEY AUTOINCREMENT,
    nombre VARCHAR(100) NOT NULL,
    correo_electronico VARCHAR(100) NOT NULL,
    contrasena VARCHAR(255) NOT NULL
);

CREATE TABLE Disponibilidad (
    id_disponibilidad INTEGER PRIMARY KEY AUTOINCREMENT,
    id_medico INTEGER NOT NULL,
    dia_semana VARCHAR(20),
    hora_inicio TIME,
    hora_fin TIME,
    FOREIGN KEY (id_medico) REFERENCES Medico(id_medico)
);

CREATE TABLE Cita (
    id_cita INTEGER PRIMARY KEY AUTOINCREMENT,
    id_paciente INTEGER NOT NULL,
    id_medico INTEGER NOT NULL,
    fecha_hora DATETIME NOT NULL,
    motivo TEXT,
    estado VARCHAR(20),
    FOREIGN KEY (id_paciente) REFERENCES Paciente(id_paciente),
    FOREIGN KEY (id_medico) REFERENCES Medico(id_medico)
);

CREATE TABLE Robot (
    id_robot INTEGER PRIMARY KEY AUTOINCREMENT,
    modelo VARCHAR(100),
    ubicacion VARCHAR(100),
    funcionalidades TEXT,
    estado VARCHAR(20)
);

CREATE TABLE Interaccion_Robot (
    id_interaccion INTEGER PRIMARY KEY AUTOINCREMENT,
    id_paciente INTEGER NOT NULL,
    id_robot INTEGER NOT NULL,
    tipo_interaccion VARCHAR(20),
    fecha_hora DATETIME NOT NULL,
    resultado TEXT,
    FOREIGN KEY (id_paciente) REFERENCES Paciente(id_paciente),
    FOREIGN KEY (id_robot) REFERENCES Robot(id_robot)
);

CREATE TABLE Conversacion (
    id_conversacion INTEGER PRIMARY KEY AUTOINCREMENT,
    id_paciente INTEGER NOT NULL,
    id_robot INTEGER NOT NULL,
    fecha DATETIME NOT NULL,
    contenido TEXT,
    FOREIGN KEY (id_paciente) REFERENCES Paciente(id_paciente),
    FOREIGN KEY (id_robot) REFERENCES Robot(id_robot)
);

CREATE TABLE Estadistica_Robot (
    id_estadistica INTEGER PRIMARY KEY AUTOINCREMENT,
    id_robot INTEGER NOT NULL,
    fecha DATE NOT NULL,
    interacciones_totales INTEGER,
    interacciones_exitosas INTEGER,
    interacciones_fallidas INTEGER,
    tiempo_promedio_respuesta FLOAT,
    FOREIGN KEY (id_robot) REFERENCES Robot(id_robot)
);
