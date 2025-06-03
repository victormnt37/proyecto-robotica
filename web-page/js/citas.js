let wsControl = new WebSocket("ws://localhost:8765");

function sendGoal(sala) {
    const msg = {
        command: "go_to_goal",
        x: sala.x,
        y: sala.y,
        w: sala.w
    };
    wsControl.send(JSON.stringify(msg));
}

// Esta es un cita de ejemplo, en un caso real los datos de la cita se recogerían del backend
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

elementoCita = crearCita(cita);

document.getElementById('id_paciente').addEventListener('keyup', (e) => {
    if (e.target.value == cita.id_paciente) {
        mostrarCita(elementoCita);
    } else if (e.target.value != cita.id_paciente) {
        ocultarCita(elementoCita);
    }
})

function crearCita(cita) {
    tabla = document.getElementById('citas-table').lastElementChild;

    const fila = document.createElement('tr');
    fila.style = 'display: none';

    const celdaFecha = document.createElement('td');
    celdaFecha.textContent = cita.fecha + ' ' + cita.hora;

    const celdaSala = document.createElement('td');
    celdaSala.textContent = cita.sala.nombre;

    fila.appendChild(celdaFecha);
    fila.appendChild(celdaSala);
    fila.addEventListener('click', () => {
        document.getElementById('popup-fecha').textContent = cita.fecha;
        document.getElementById('popup-hora').textContent = cita.hora;
        document.getElementById('popup-paciente').textContent = cita.nombre_paciente;
        document.getElementById('popup-doctor').textContent = cita.nombre_doctor;
        document.getElementById('popup-motivo').textContent = cita.motivo;
        document.getElementById('popup-sala').textContent = cita.sala.nombre;

        document.getElementById('popup').classList.remove('hidden');
    });

    // cancelar pop-up
    document.getElementById('cerrar-popup').addEventListener('click', () => {
        document.getElementById('popup').classList.add('hidden');
    });
    document.getElementById('cancelar-popup').addEventListener('click', () => {
        document.getElementById('popup').classList.add('hidden');
    });

    // dirigirse a sala
    document.getElementById('dirigir-a-sala').addEventListener('click', () => {
        sendGoal(cita.sala);
        document.getElementById('popup').classList.add('hidden');

        mostrarToast(`Sigue al robot Kyron a la sala: ${cita.sala.nombre}`);
    })

    tabla.appendChild(fila);

    return fila;
}

function mostrarCita(elementoCita) {
    elementoCita.style = 'display: block';
}

function ocultarCita(elementoCita) {
    elementoCita.style = 'display: none';
}

function mostrarToast(mensaje, duracion = 3000) {
    const toast = document.getElementById('toast');
    toast.textContent = mensaje;
    toast.classList.add('show');
    toast.classList.remove('hidden');
  
    setTimeout(() => {
      toast.classList.remove('show');
      toast.classList.add('hidden');
    }, duracion);
  }
  