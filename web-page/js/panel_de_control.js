// Seleccionamos los elementos
const toggleButton = document.getElementById('toggle-salas-dropdown');
const salasDropdown = document.getElementById('salas-dropdown');
const salaButtons = document.querySelectorAll('.boton-sala');

let isOpen = false;

// Al pulsar el botón principal
toggleButton.addEventListener('click', () => {
    isOpen = !isOpen;
    salasDropdown.classList.toggle('hidden', !isOpen);
});

// Al pulsar cualquier botón de sala
salaButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        isOpen = false;
        salasDropdown.classList.add('hidden');
    });
});
