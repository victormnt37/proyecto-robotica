const cita = {
    fecha: "04/06/2025",
    id_paciente: "123456A",
    nombre_paciente: "Víctor Morant Faus",
    id_doctor: "654321B",
    nombre_doctor: "Dr Ariel Bejarán",
    hora: "12:00",
    sala: {
        nombre: "Sala 8",
        x: 6.0, 
        y: -28.0, 
        w: 0.0
    },
    motivo: "Resonancia"
}

console.log('citas');

elementoCita = crearCita(cita);

document.getElementById('id_paciente').addEventListener('keyup', (e) => {
    if (e.target.value == cita.id_paciente) {
        mostrarCita(elementoCita);
    } else if (e.target.value != cita.id_paciente) {
        ocultarCita(elementoCita);
    }
})

function crearCita(cita) {
    tabla = document.getElementById('citas-table');

    const fila = document.createElement('tr');
    fila.style = 'display: none';

    const celdaFecha = document.createElement('td');
    celdaFecha.textContent = cita.fecha + ' ' + cita.hora;

    const celdaSala = document.createElement('td');
    celdaSala.textContent = cita.sala.nombre;

    fila.appendChild(celdaFecha);
    fila.appendChild(celdaSala);

    tabla.appendChild(fila);

    return fila;
}

function mostrarCita(elementoCita) {
    elementoCita.style = 'display: block';
}

function ocultarCita(elementoCita) {
    elementoCita.style = 'display: none';
}

// TODO: que cuando clique aparezca el popup de datos de la cita y la sala