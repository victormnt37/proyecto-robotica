document.addEventListener('DOMContentLoaded', event => {
    let wsControl = new WebSocket("ws://localhost:8765");

    document.getElementById("btn_con").addEventListener("click", connect)
    document.getElementById("btn_dis").addEventListener("click", disconnect)
    estado = document.getElementById('estado')
    iconoEstado = document.getElementById('connection-status')
    tarea = document.getElementById('tarea-actual')
    document.getElementById("btn_move").addEventListener("click", move)
    document.getElementById("btn_stop").addEventListener("click", stop)
    document.getElementById("btn_right").addEventListener("click", derecha)
    document.getElementById("btn_left").addEventListener("click", izquierda)
    

    // boton patrullar
    document.getElementById("btn_waypoints").addEventListener("click", () => {
        console.log('Comienza la patrulla');
        wsControl.send(JSON.stringify({ command: "waypoint_routine" }));
        tarea.innerHTML = "Patrullando";
    });

    // botones de dirección a salas
    const salas = [
        { id: "sala-entrada", x: 0.0, y: 15.0, w: 0.0 },
        { id: "sala-recepcion", x: 0.0, y: 2.0, w: 0.0 },
        { id: "sala-5", x: -8.0, y: -8.0, w: 0.0 },
        { id: "sala-6", x: -7.0, y: -22.0, w: 0.0 },
        { id: "sala-7", x: -9.0, y: -27.0, w: 0.0 },
        { id: "sala-8", x: 6.0, y: -28.0, w: 0.0 },
        { id: "sala-11", x: 7.0, y: -22.0, w: 0.0 },
        { id: "sala-12", x: 8.0, y: -8.0, w: 0.0 },
        { id: "sala-16", x: 5.0, y: 17.0, w: 0.0 },
        { id: "sala-1", x: -5.0, y: 17.0, w: 0.0 },
    ];
    
    salas.forEach(({ id, x, y, w }) => {
        const button = document.getElementById(id);
        if (button) {
            button.addEventListener("click", () => {
                sendGoal(x, y, w, button.innerHTML);
            });
        } else {
            console.warn(`Botón con id "${id}" no encontrado.`);
        }
    });
    

    data = {
        // ros connection
        ros: null,
        rosbridge_address: 'ws://127.0.0.1:9090/', // CAMBIAR ESTA IP PARA EL ROBOT REAL (R11:192.168.0.134)
        connected: false,
        // service information 
        service_busy: false, 
	    service_response: ''
    }

    function connect(){
	    console.log("Clic en connect")

	    data.ros = new ROSLIB.Ros({
            url: data.rosbridge_address
        })

        // Define callbacks
        data.ros.on("connection", () => {
            data.connected = true
            console.log("Conexion con ROSBridge correcta")
            estado.innerHTML = 'Conectado'
            iconoEstado.classList.add('active')
            
        })
        data.ros.on("error", (error) => {
            console.log("Se ha producido algun error mientras se intentaba realizar la conexion")
            console.log(error)
            estado.innerHTML = 'Desconectado'
            iconoEstado.classList.remove('active')
        })
        data.ros.on("close", () => {
            data.connected = false
            console.log("Conexion con ROSBridge cerrada")
            estado.innerHTML = 'Desconectado'
            iconoEstado.classList.remove('active')

        })

        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        })
    
        topic.subscribe((message) => {
            data.position = message.pose.pose.position
                document.getElementById("pos_x").innerHTML = data.position.x.toFixed(2)
                document.getElementById("pos_y").innerHTML = data.position.y.toFixed(2)
        })

        // Suscripción al nuevo topic con tipo personalizado
        const conteoPersonasTopic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/vision/id_cuerpo',
            messageType: 'kyron_interface/msg/ConteoPersonas2'
        });

        conteoPersonasTopic.subscribe((message) => {
            // Actualizar el DOM con los datos del mensaje
            document.getElementById("num-pacientes").textContent = message.pacientes;
            document.getElementById("num-personal").textContent = message.personal;
        });

          // Suscripción a id_cara
          const id_caras_topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/vision/id_cara',
            messageType: 'kyron_interface/msg/PersonaIdentificada'
        });

        id_caras_topic.subscribe((message) => {
            // Actualizar el DOM con los datos del mensaje
            document.getElementById("persona-identificada").textContent = message.nombre_persona;
        });

        updateCameraFeed()

        tarea.innerHTML = "En espera";
    }


    function move() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: {x: 0.2, y: 0, z: 0, },
            angular: {x: 0, y: 0, z: 0, },
        })
        topic.publish(message)
    }

    function stop() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: {x: 0, y: 0, z: 0, },
            angular: {x: 0, y: 0, z: 0, },
        })
        topic.publish(message)
    }

    function disconnect(){
        stop()
        data.ros.close()
        data.connected = false
        console.log('Clic en botón de desconexión')
    }
    function derecha() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: {x: 0, y: 0, z: 0, },
            angular: {x: 0, y: 0, z: -0.5, },
        })
        topic.publish(message)
    }

    function izquierda() {
        let topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        })
        let message = new ROSLIB.Message({
            linear: {x: 0, y: 0, z: 0, },
            angular: {x: 0, y: 0, z: 0.5, },
        })
        topic.publish(message)
    }

    // Versión usando librería MJPEGCANVAS (requiere cargarla)
    function setCamera(){
        console.log("setting the camera")
    var viewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpeg',
        host : 'localhost',
        width : 640,
        height : 480,
        topic : '/camera/image_raw',
        interval : 200
        })
    }

    // otro ejemplo de función (simple para prueba inicial)
    function updateCameraFeed() {
        console.log("setting the camera")
        const img = document.getElementById("cameraFeed");
        const timestamp = new Date().getTime(); // Evita caché agregando un timestamp
        img.src = `http://0.0.0.0:8080/stream?topic=/camera/image_raw`;//cambiar el topic por /image/
        //img.src = `http://localhost:8080/stream?topic=/turtlebot3/camera/image_raw&console.log("Cactualizando: http://0.0.0.0:8080/stream?topic=/camera/image_raw)"`
    }

    function sendGoal(x, y, w, sala) {
        const msg = {
            command: "go_to_goal",
            x: x,
            y: y,
            w: w
        };
        wsControl.send(JSON.stringify(msg));
        tarea.innerHTML = "Dirigiéndose a " + sala;
    }
    
});