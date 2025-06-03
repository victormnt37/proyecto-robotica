BEGIN TRANSACTION;
CREATE TABLE IF NOT EXISTS "Administrador" (
	"id_admin"	INTEGER,
	"nombre"	VARCHAR(100) NOT NULL,
	"correo_electronico"	VARCHAR(100) NOT NULL,
	"contrasena"	VARCHAR(255) NOT NULL,
	PRIMARY KEY("id_admin" AUTOINCREMENT)
);
CREATE TABLE IF NOT EXISTS "Cita" (
	"id_cita"	INTEGER,
	"id_paciente"	INTEGER NOT NULL,
	"id_medico"	INTEGER NOT NULL,
	"fecha_hora"	DATETIME NOT NULL,
	"motivo"	TEXT,
	"estado"	VARCHAR(20),
	PRIMARY KEY("id_cita" AUTOINCREMENT),
	FOREIGN KEY("id_medico") REFERENCES "Medico"("id_medico"),
	FOREIGN KEY("id_paciente") REFERENCES "Paciente"("id_paciente")
);
CREATE TABLE IF NOT EXISTS "Conversacion" (
	"id_conversacion"	INTEGER,
	"id_paciente"	INTEGER NOT NULL,
	"id_robot"	INTEGER NOT NULL,
	"fecha"	DATETIME NOT NULL,
	"contenido"	TEXT,
	PRIMARY KEY("id_conversacion" AUTOINCREMENT),
	FOREIGN KEY("id_paciente") REFERENCES "Paciente"("id_paciente"),
	FOREIGN KEY("id_robot") REFERENCES "Robot"("id_robot")
);
CREATE TABLE IF NOT EXISTS "Disponibilidad" (
	"id_disponibilidad"	INTEGER,
	"id_medico"	INTEGER NOT NULL,
	"dia_semana"	VARCHAR(20),
	"hora_inicio"	TIME,
	"hora_fin"	TIME,
	PRIMARY KEY("id_disponibilidad" AUTOINCREMENT),
	FOREIGN KEY("id_medico") REFERENCES "Medico"("id_medico")
);
CREATE TABLE IF NOT EXISTS "Estadistica_Robot" (
	"id_estadistica"	INTEGER,
	"id_robot"	INTEGER NOT NULL,
	"fecha"	DATE NOT NULL,
	"interacciones_totales"	INTEGER,
	"interacciones_exitosas"	INTEGER,
	"interacciones_fallidas"	INTEGER,
	"tiempo_promedio_respuesta"	FLOAT,
	PRIMARY KEY("id_estadistica" AUTOINCREMENT),
	FOREIGN KEY("id_robot") REFERENCES "Robot"("id_robot")
);
CREATE TABLE IF NOT EXISTS "Interaccion_Robot" (
	"id_interaccion"	INTEGER,
	"id_paciente"	INTEGER NOT NULL,
	"id_robot"	INTEGER NOT NULL,
	"tipo_interaccion"	VARCHAR(20),
	"fecha_hora"	DATETIME NOT NULL,
	"resultado"	TEXT,
	PRIMARY KEY("id_interaccion" AUTOINCREMENT),
	FOREIGN KEY("id_paciente") REFERENCES "Paciente"("id_paciente"),
	FOREIGN KEY("id_robot") REFERENCES "Robot"("id_robot")
);
CREATE TABLE IF NOT EXISTS "Medico" (
	"id_medico"	INTEGER,
	"nombre"	VARCHAR(100) NOT NULL,
	"especialidad"	VARCHAR(100),
	"telefono"	VARCHAR(15),
	"correo_electronico"	VARCHAR(100),
	"contrasena"	VARCHAR(255),
	PRIMARY KEY("id_medico" AUTOINCREMENT)
);
CREATE TABLE IF NOT EXISTS "Paciente" (
	"id_paciente"	INTEGER,
	"nombre"	VARCHAR(100) NOT NULL,
	"fecha_nacimiento"	DATE,
	"telefono"	VARCHAR(15),
	"correo_electronico"	VARCHAR(100),
	"DNI"	VARCHAR(100),
	"contrasena"	VARCHAR(255),
	PRIMARY KEY("id_paciente" AUTOINCREMENT)
);
CREATE TABLE IF NOT EXISTS "Robot" (
	"id_robot"	INTEGER,
	"modelo"	VARCHAR(100),
	"ubicacion"	VARCHAR(100),
	"funcionalidades"	TEXT,
	"estado"	VARCHAR(20),
	PRIMARY KEY("id_robot" AUTOINCREMENT)
);
INSERT INTO "Administrador" VALUES (1,'Lucía Morales','lucia.morales@hospital.com','123456');
INSERT INTO "Administrador" VALUES (2,'Pedro Sánchez','pedro.sanchez@hospital.com','123456');
INSERT INTO "Administrador" VALUES (3,'Clara Ruiz','clara.ruibe@hospital.com','123456');
INSERT INTO "Administrador" VALUES (4,'Andrés Castro','andres.castro@hospital.com','123456');
INSERT INTO "Administrador" VALUES (5,'Marta Díaz','marta.diaz@hospital.com','123456');
INSERT INTO "Administrador" VALUES (6,'Ariel','ariel.bejaran@hospital.com','123456');
INSERT INTO "Cita" VALUES (1,3,3,'2025-05-15 09:00:00','Chequeo cardiovascular','Programada');
INSERT INTO "Cita" VALUES (2,4,4,'2025-05-16 10:30:00','Consulta pediátrica','Programada');
INSERT INTO "Cita" VALUES (3,5,5,'2025-05-17 15:00:00','Dolor de cabeza crónico','Programada');
INSERT INTO "Cita" VALUES (4,6,6,'2025-05-18 11:00:00','Control prenatal','Programada');
INSERT INTO "Cita" VALUES (5,7,7,'2025-05-19 09:30:00','Dolor de rodilla','Programada');
INSERT INTO "Cita" VALUES (6,4,5,'2025-06-04 10:30:00','Consulta general','pendiente');
INSERT INTO "Cita" VALUES (7,5,6,'2025-06-05 12:15:00','Chequeo anual','pendiente');
INSERT INTO "Cita" VALUES (8,6,3,'2025-06-06 17:15:00','Dolor de cabeza','cancelada');
INSERT INTO "Cita" VALUES (9,7,4,'2025-06-15 08:00:00','Consulta pediátrica','pendiente');
INSERT INTO "Cita" VALUES (10,4,5,'2025-06-11 09:30:00','Análisis de sangre','cancelada');
INSERT INTO "Cita" VALUES (11,4,3,'2025-06-01 16:15:00','Chequeo anual','cancelada');
INSERT INTO "Cita" VALUES (12,4,3,'2025-06-15 12:45:00','Chequeo anual','cancelada');
INSERT INTO "Cita" VALUES (13,6,5,'2025-06-07 09:15:00','Revisión','cancelada');
INSERT INTO "Cita" VALUES (14,5,6,'2025-06-08 17:45:00','Chequeo anual','confirmada');
INSERT INTO "Cita" VALUES (15,7,3,'2025-06-12 17:45:00','Consulta pediátrica','cancelada');
INSERT INTO "Cita" VALUES (16,3,7,'2025-06-09 09:30:00','Consulta pediátrica','cancelada');
INSERT INTO "Cita" VALUES (17,7,5,'2025-06-13 12:00:00','Análisis de sangre','pendiente');
INSERT INTO "Cita" VALUES (18,7,7,'2025-06-05 16:45:00','Dolor de cabeza','confirmada');
INSERT INTO "Cita" VALUES (19,6,5,'2025-06-03 16:00:00','Dolor de cabeza','cancelada');
INSERT INTO "Cita" VALUES (20,7,4,'2025-06-04 15:15:00','Consulta general','confirmada');
INSERT INTO "Cita" VALUES (21,3,4,'2025-06-10 14:45:00','Dolor de cabeza','cancelada');
INSERT INTO "Cita" VALUES (22,3,5,'2025-06-05 11:15:00','Revisión','confirmada');
INSERT INTO "Cita" VALUES (23,5,7,'2025-06-09 12:45:00','Consulta general','pendiente');
INSERT INTO "Cita" VALUES (24,5,3,'2025-06-08 14:00:00','Consulta general','confirmada');
INSERT INTO "Cita" VALUES (25,6,3,'2025-06-04 13:00:00','Revisión','confirmada');
INSERT INTO "Cita" VALUES (26,6,5,'2025-06-03 17:15:00','Consulta pediátrica','cancelada');
INSERT INTO "Cita" VALUES (27,5,6,'2025-06-04 14:45:00','Dolor de cabeza','confirmada');
INSERT INTO "Cita" VALUES (28,4,3,'2025-06-10 12:45:00','Consulta pediátrica','confirmada');
INSERT INTO "Cita" VALUES (29,4,6,'2025-06-15 12:00:00','Chequeo anual','pendiente');
INSERT INTO "Cita" VALUES (30,4,7,'2025-06-15 10:30:00','Consulta pediátrica','pendiente');
INSERT INTO "Cita" VALUES (31,6,3,'2025-06-08 11:45:00','Consulta general','confirmada');
INSERT INTO "Cita" VALUES (32,4,7,'2025-06-03 08:00:00','Consulta general','cancelada');
INSERT INTO "Cita" VALUES (33,7,7,'2025-06-14 10:30:00','Consulta general','pendiente');
INSERT INTO "Cita" VALUES (34,6,6,'2025-06-01 13:00:00','Dolor de cabeza','pendiente');
INSERT INTO "Cita" VALUES (35,7,3,'2025-06-15 15:30:00','Consulta general','confirmada');
INSERT INTO "Conversacion" VALUES (1,3,3,'2025-05-13','Paciente: ¿Dónde está el consultorio 3? Robot: Siga recto y gire a la izquierda.');
INSERT INTO "Conversacion" VALUES (2,4,3,'2025-05-13','Paciente: ¿Cuándo es mi cita? Robot: Su cita es mañana a las 10:30.');
INSERT INTO "Conversacion" VALUES (3,5,3,'2025-05-13','Paciente: ¿Pueden medir mi presión? Robot: Presión arterial: 120/80.');
INSERT INTO "Conversacion" VALUES (4,6,5,'2025-05-13','Paciente: ¿Cómo llego a la farmacia? Robot: Le guiaré, sígame.');
INSERT INTO "Conversacion" VALUES (5,7,6,'2025-05-13','Paciente: ¿Entregan medicamentos aquí? Robot: Sí, aquí tiene su receta.');
INSERT INTO "Disponibilidad" VALUES (1,3,'Lunes','08:00:00','12:00:00');
INSERT INTO "Disponibilidad" VALUES (2,4,'Martes','09:00:00','13:00:00');
INSERT INTO "Disponibilidad" VALUES (3,5,'Miércoles','14:00:00','18:00:00');
INSERT INTO "Disponibilidad" VALUES (4,6,'Jueves','10:00:00','14:00:00');
INSERT INTO "Disponibilidad" VALUES (5,7,'Viernes','08:30:00','12:30:00');
INSERT INTO "Estadistica_Robot" VALUES (1,2,'2025-05-13',50,45,5,2.5);
INSERT INTO "Estadistica_Robot" VALUES (2,3,'2025-05-13',40,38,2,3.0);
INSERT INTO "Estadistica_Robot" VALUES (3,4,'2025-05-13',30,25,5,4.2);
INSERT INTO "Estadistica_Robot" VALUES (4,5,'2025-05-13',60,58,2,2.8);
INSERT INTO "Estadistica_Robot" VALUES (5,6,'2025-05-13',45,40,5,3.5);
INSERT INTO "Interaccion_Robot" VALUES (1,3,2,'Guía','2025-05-13 08:15:00','Paciente guiado a consultorio');
INSERT INTO "Interaccion_Robot" VALUES (2,4,3,'Consulta','2025-05-13 09:00:00','Información de cita proporcionada');
INSERT INTO "Interaccion_Robot" VALUES (3,5,4,'Asistencia','2025-05-13 10:30:00','Monitoreo de signos vitales');
INSERT INTO "Interaccion_Robot" VALUES (4,6,5,'Guía','2025-05-13 11:45:00','Paciente llevado a farmacia');
INSERT INTO "Interaccion_Robot" VALUES (5,7,6,'Entrega','2025-05-13 12:00:00','Medicamentos entregados');
INSERT INTO "Medico" VALUES (3,'Dr. José Ramírez','Cardiología','555-678-9012','jose.ramirez@hospital.com','123456');
INSERT INTO "Medico" VALUES (4,'Dra. Sofía Torres','Pediatría','555-789-0123','sofia.torres@hospital.com','123456');
INSERT INTO "Medico" VALUES (5,'Dr. Miguel Ángel','Neurología','555-890-1234','miguel.angel@hospital.com','123456');
INSERT INTO "Medico" VALUES (6,'Dra. Elena Vargas','Ginecología','555-901-2345','elena.vargas@hospital.com','123456');
INSERT INTO "Medico" VALUES (7,'Dr. Ricardo Gómez','Ortopedia','555-012-3456','ricardo.gomez@hospital.com','123456');
INSERT INTO "Medico" VALUES (8,'Ana Martín','Medicina General','700123456','ana.martin@example.com','ana123');
INSERT INTO "Medico" VALUES (9,'Javier Serrano','Pediatría','700654321','javier.serrano@example.com','javi2025');
INSERT INTO "Medico" VALUES (10,'Elena Molina','Cardiología','700111222','elena.molina@example.com','cardio1');
INSERT INTO "Medico" VALUES (11,'Miguel Nieto','Dermatología','700333444','miguel.nieto@example.com','derma321');
INSERT INTO "Medico" VALUES (12,'Carmen Vega','Neurología','700555666','carmen.vega@example.com','neuro2025');
INSERT INTO "Medico" VALUES (13,'Victor','Ginecologo','700555664','victor@example.com','123456');
INSERT INTO "Paciente" VALUES (3,'Ana García','1985-03-15','555-123-4567','ana.garcia@email.com','20863809A','123456');
INSERT INTO "Paciente" VALUES (4,'Carlos López','1990-07-22','555-234-5678','carlos.lopez@email.com','10863809B','123456');
INSERT INTO "Paciente" VALUES (5,'María Fernández','1978-11-30','555-345-6789','maria.fernandez@email.com','30863809C','123456');
INSERT INTO "Paciente" VALUES (6,'Juan Pérez','2000-01-10','555-456-7890','juan.perez@email.com','40863809D','123456');
INSERT INTO "Paciente" VALUES (7,'Laura Martínez','1995-06-05','555-567-8901','laura.martinez@email.com','50863809E','123456');
INSERT INTO "Paciente" VALUES (8,'Juan Pérez','1990-05-10','600123456','juan.perez@example.com','12345678A','1234');
INSERT INTO "Paciente" VALUES (9,'María López','1988-08-22','600654321','maria.lopez@example.com','87654321B','abcd');
INSERT INTO "Paciente" VALUES (10,'Carlos Ramírez','1995-11-15','600111222','carlos.ramirez@example.com','11223344C','pass123');
INSERT INTO "Paciente" VALUES (11,'Lucía Fernández','1992-03-05','600333444','lucia.fernandez@example.com','22334455D','lucia2025');
INSERT INTO "Paciente" VALUES (12,'David Gómez','1985-12-30','600555666','david.gomez@example.com','33445566E','david123');
INSERT INTO "Paciente" VALUES (13,'Denys','1985-12-30','600555667','denys.denys@example.com','33445566X','denys123');
INSERT INTO "Paciente" VALUES (14,'Pau','1985-12-30','600555667','pau@example.com','33445566Z','denys123');
INSERT INTO "Robot" VALUES (2,'RX-100','Recepción','Navegación, asistencia al paciente','Activo');
INSERT INTO "Robot" VALUES (3,'MediBot-2','Sala de espera','Monitoreo, entrega de información','Activo');
INSERT INTO "Robot" VALUES (4,'CareBot-3','Consultorios','Asistencia médica básica','Mantenimiento');
INSERT INTO "Robot" VALUES (5,'NaviBot-4','Pasillos','Guía de pacientes','Activo');
INSERT INTO "Robot" VALUES (6,'HealthBot-5','Farmacia','Entrega de medicamentos','Activo');
COMMIT;
