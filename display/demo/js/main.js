var ros = new ROSLIB.Ros();

let COUNTER_VALUE = -1;

ros.on('connection', () => {
    console.log('Connected to websocket server.');
});

ros.on('error', (error) => {
    console.log('Error connecting to websocket server: ', error);
    window.alert('Error connecting to websocket server');
});

ros.on('close', () => {
    console.log('Connection to websocket server closed.');
});

ros.connect('ws://' + window.location.hostname + ':9090');

const sayTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/tts/goal',
    messageType: 'pal_interaction_msgs/TtsActionGoal'
});

const playMotionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/play_motion/goal',
    messageType: 'play_motion_msgs/PlayMotionActionGoal'
})

const goToTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/web/go_to',
    messageType: 'pal_web_msgs/WebGoTo'
});

var navTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/poi_navigation_server/go_to_poi/goal', 
    messageType: 'pal_navigation_msgs/GoToPOIActionGoal' 
});

// Subscribe to the POI topic
var poiTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/poi_marker_server/update',
    messageType: 'visualization_msgs/InteractiveMarkerUpdate' 
});

// Listen for messages from the POI topic
poiTopic.subscribe(function(message) {
    //console.log('Received POI message:', message);
    displayPOIs(message);
});


/*displayPOIs({
    markers: [
        { name: 'Entrance' },
        { name: 'Reception' },
        { name: 'Meeting Room' },
        { name: 'Office' },
        { name: 'Kitchen' },
        { name: 'Toilet' },
        { name: 'Exit' }
    ]
});*/

// Function to display POIs as buttons
function displayPOIs(pois) {
    
    //console.log('Displaying POIs:', pois);
    var container = document.getElementById('poi-buttons');
    //container.innerHTML = ''; // Clear any existing buttons
    // Iterating over the markers array and displaying each name
    let index = 0;
    for (let marker of pois.markers) { // Utilizza for...of per iterare sugli elementi di markers
        console.log(marker.name);
        var row = document.createElement('div');
        row.className = 'row';
        row.style.padding = '10px';
        row.id = 'row' + index; // Imposta l'ID prima di incrementare l'indice

        var button = document.createElement('button');
        button.className = 'btn btn-light btn-lg ';
        button.innerText = marker.name;

        button.addEventListener('click', function() {
            goToPOI(marker.name);
        });

        row.appendChild(button); // Aggiungi il pulsante alla riga direttamente
        container.appendChild(row);
        index++; // Incrementa l'indice qui
    }
    

}

// Function to navigate to a POI
function goToPOI(poi) {
    console.log('Navigating to POI:', poi);
    // Using the POI name as the goal and the pal_navigation_msgs/GoToPOIActionGoal message type
    var goal = new ROSLIB.Message({
        goal: {
            poi: {
                data: poi
            }
        }
    });

    navTopic.publish(goal);
}


let currentTopic = '/vision_msgs/soft_biometrics/frames/compressed';
const topics = [
    '/vision_msgs/soft_biometrics/frames/compressed',
    '/vision_msgs/human_2d_pose/frames/compressed',
    '/vision_msgs/gaze_estimation/frames/compressed',
    '/vision_msgs/depth_estimation/frames/compressed',
    '/torso_front_camera/color/image_raw/compressed',
    '/head_front_camera/color/image_raw/compressed'
].map((topic) => new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: 'sensor_msgs/CompressedImage'
}));

topics.forEach((topic) => {
    topic.subscribe((message) => {
        if (currentTopic != topic.name) {
            return;
        }

        $('#feed').attr('src', 'data:image/jpg;base64,' + message.data);
    });
});

function putImage() {
    document.getElementById('button_container').style.display = 'none';
    document.getElementById('feed_container').style.display = null;
    currentTopic = '/vision_msgs/soft_biometrics/frames/compressed';
    $('#feed').attr('src', './../assets/images/disi_big.png');
}

function changeTopic(name) {
    console.log('Changing topic to ' + name);
    currentTopic = name;

    $('#feed').attr('src', '');

    toggleFeed();
}

function say(text, lang) {
    if (!lang) {
        lang = 'it_IT'
    }

    counter = COUNTER_VALUE;

    sayTopic.publish({
        goal: {
            rawtext: {
                text: text,
                lang_id: lang
            },
            speakerName: '',
            wait_before_speaking: 0.0
        }
    });
}

function play(action) {
    playMotionTopic.publish({
        goal: {
            motion_name: action,
            skip_planning: true
        }
    });
}

function openCommander() {
    console.log('Opening Commander');

    goToTopic.publish({
        type: 2,
        value: 'http://localhost:8080/'
    });
}

function toggleFeed() {
    document.getElementById('button_container').style.display = 'none';
    document.getElementById('feed_container').style.display = null;
    document.getElementById('POI_container').style.display = 'none';
}

function toggleButtons() {
    document.getElementById('button_container').style.display = null;
    document.getElementById('feed_container').style.display = 'none';
    document.getElementById('POI_container').style.display = 'none';
}

function togglePOI(){
    document.getElementById('button_container').style.display = 'none';
    document.getElementById('feed_container').style.display = 'none';
    document.getElementById('POI_container').style.display = null;

}

function resize() {
    document.getElementById('feed').style.height = window.innerHeight + 'px';
}

window.onload = resize;
window.onresize = resize;