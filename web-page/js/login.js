document.getElementById('loginForm').addEventListener('submit', function (e) {
    e.preventDefault();
  
    const email = document.getElementById('email').value.trim();
    const password = document.getElementById('password').value.trim();
    const errorMsg = document.getElementById('errorMsg');
  
    if (email === "usuario@gmail.com" && password === "123456") {
      window.location.href = "../html/panel_de_control.html";
    } else {
      errorMsg.textContent = "Correo o contraseña incorrectos.";
      errorMsg.style.display = "block";
    }
  });
  