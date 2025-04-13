// Código de ayuda para cargar canvas y marcar robot en el mapa
// Cargar metadatos del mapa
const mapYamlUrl = '../../src/kyron/kyron_map_server/map/hospital_world.yaml'; // poner ubicación
const mapImageUrl ='../../src/kyron/kyron_map_server/map/hospital_world.png'; // poner ubicación

let mapInfo = null;
let canvas = document.getElementById("mapCanvas");
let ctx = canvas.getContext("2d");
let image = new Image();
let robotPosition = {x: 0, y: 0};


document.addEventListener("DOMContentLoaded", () => {
    // Leer YAML del mapa
    fetch(mapYamlUrl)
      .then(response => response.text())
      .then(yamlText => {
        const doc = jsyaml.load(yamlText);
        mapInfo = doc;
        image.src = mapImageUrl;
      });
  });


// Dibujar mapa una vez cargada la imagen
image.onload = () => {
  canvas.width = image.width;
  canvas.height = image.height;
  ctx.drawImage(image, 0, 0);
  console.log("✅ Imagen cargada y dibujada");
};
console.log("deberias estar dibujado ya");
// Función para redibujar mapa y robot
function draw() {
  if (!mapInfo || !image.complete) return;

  // Redibujar el mapa
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(image, 0, 0);

  // Transformar coordenadas ROS -> imagen
  const res = mapInfo.resolution;
  const origin = mapInfo.origin;

  // Transformar odom -> pixeles
  let pixelX = (robotPosition.x - origin[0]) / res;
  let pixelY = canvas.height - ((robotPosition.y - origin[1]) / res); // invertido en Y

  // Dibujar robot
  ctx.beginPath();
  ctx.fillStyle = 'green';
  ctx.arc(pixelX, pixelY, 5, 0, 2 * Math.PI);
  ctx.fill();
}

// Ajustar esta parte en la conexión ros y suscripción al Topic /odom
//odomTopic.subscribe((message) => {
//   robotPosition.x = message.pose.pose.position.x;
//   robotPosition.y = message.pose.pose.position.y;
 //   draw();  
//})// redibuja mapa + posición del robot

