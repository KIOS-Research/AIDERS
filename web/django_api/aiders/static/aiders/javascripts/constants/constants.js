/*
 * Contains all constant variables used by the platform code
 * */

let NOT_FOUND_STRING = 'NOT_FOUND';

var SERVER_IP = location.hostname;

const USER_WANTS_TO_STOP_IT = 1;
const TOO_LOW_ALTITUDE_FOR_BUILD_MAP = 0;

NO_ACTIVE_DETECTION_SESSION_ERROR_MESSAGE = 'No_Active_Detection_Session';
NO_ACTIVE_LIVE_STREAM_SESSION_ERROR_MESSAGE = 'No_Active_Live_Stream_Session';

let True = true;
let False = false;
let ALGORITHMS = {
    FIRE_PROPAGATION_ALGORITHM: 'FIRE_PROPAGATION_ALGORITHM',
    CREATE_3D_OBJECT_ALGORITHM: 'CREATE_3D_OBJECT_ALGORITHM',
    CREATE_ORTHOPHOTO_ALGORITHM: 'CREATE_ORTHOPHOTO_ALGORITHM',
    CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM: 'CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM',
};

let MISSION_ACTIONS = {
    START_MISSION: 'START_MISSION',
    PAUSE_MISSION: 'PAUSE_MISSION',
    RESUME_MISSION: 'RESUME_MISSION',
    CANCEL_MISSION: 'CANCEL_MISSION',
};
let GRID_MISSION = 'gridMission';
let NORMAL_MISSION = 'normalMission';
let SEARCH_AND_RESCUE_MISSION = 'searchAndRescueMission';

let ALL_CURRENT_ALTITUDE = 'allCurrentAltitude';
let ALL_SAME_ALTITUDE = 'allSameAltitude';
let ALL_DIFFERENT_ALTITUDE = 'differentAltitude';
let CUSTOM_ALTITUDE = 'customAltitude';

let ALL_SAME_SPEED = 'allSameSpeed';
let ALL_DIFFERENT_SPEED = 'allDifferentSpeed';
let CUSTOM_SPEED = 'customSpeed';

let MANUAL_GIMBAL = 'manualGimbal';
let CUSTOM_GIMBAL = 'customGimbal';

let NO_MODE = -1;
let MOVING = 0;
let MISSION_FLY_MODE = 1;
let WANT_TO_START_MISSION = 2;
let STOP_MISSION = 3;
let WANT_TO_PAUSE_MISSION = 4;
let NO_MOVING = 5;
let WANT_TO_RESUME_MISSION = 6;
let AVAILABLE = 7;
let NOT_AVAILABLE = 8;
let WANT_TO_CANCEL_MISSION = 9;
let WANT_TO_SELECT_MISSION = 10;
let STOP_VISUALIZING_OBJECTS = 11;
let START_VISUALIZING_OBJECTS = 12;
let DEFAULT_ALTITUDE = 50;

let DRONE_ALTITUDE_MIN = 3;
let DRONE_ALTITUDE_MAX = 120;

let DRONE_SPEED_MIN = 1;
let DRONE_SPEED_MAX = 20;

let DRONE_GIMBAL_ANGLE_MIN = -90;
let DRONE_GIMBAL_ANGLE_MAX = 0;

let WARNING_ALERT = 'alert-warning';
let SUCCESS_ALERT = 'alert-success';
let FAILED_ALERT = 'alert-danger';
let NEUTRAL_ALERT = 'neutral-danger';

var BUILD_MAP_STOPPED = 0;
var BUILD_MAP_STARTED = 1;
var BUILD_MAP_ABOUT_TO_START = 2;
var BUILD_MAP_ABOUT_TO_STOP = 3;

var BUILD_MAP_INITIAL_STATE = -1;

var DEFAULT_COLOR = '#669933';

var DETECTION_DISCONNECTED = 'DETECTION_DISCONNECTED';
var DETECTION_CONNECTED = 'DETECTION_CONNECTED';
var DETECTION_WANT_TO_CONNECT = 'DETECTION_WANT_TO_CONNECT';
var DETECTION_WANT_TO_DISCONNECT = 'DETECTION_WANT_TO_DISCONNECT';
var DETECTION_INITIAL_STATUS = 'DETECTION_INITIAL_STATUS';

var SHOW_DETS_TOGGLE_ID = 'showDetsToggle';
var ALL_DETS_TOGGLE_ID = 'detObjectsToggle';
var PERSON_DETS_TOGGLE_ID = 'detPeopleToggle';
var CAR_DETS_TOGGLE_ID = 'detCarsToggle';
var MOTOR_DETS_TOGGLE_ID = 'detMotorbikeToggle';

var SHOW_ALL_DETECTIONS_CHECKBOX_ID = 'showAllDetsCheckboxID';

const CAR = 'car';
const MOTORBIKE = 'motorbike';
const PERSON = 'person';
const BICYCLE = 'bicycle';
const NOT_LISTED = 'not_listed';

const PERSON_COLOR = 'rgb(255,0,0)';
const BICYCLE_COLOR = 'rgb(0,255,0)';
const CAR_COLOR = 'rgb(0,0,255)';
const WHITE_COLOR = 'rgb(255,255,255)';
var REMOVE_POPUP = 'remove';
var SHOW_POPUP = 'show';

let ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DRONES = [
    'drone-selection-list',
    // 'drone-tools-list',
    // 'drone-lidar-list',
    // 'enable-detections-list',
    // 'trajectory-list-drone',
    // 'video-feeds-list',
    // 'det-video-feeds-list',
];
let ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_DEVICES = ['device-selection-list'];
let ELEMENT_IDS_OF_DYNAMIC_PANEL_LISTS_BALORAS = ['balora-selection-list', 'balora-sensors-list'];

const ACTIVATE_HIGHLIGHT = 1;
const DEACTIVATE_HIGHLIGHT = 0;

var DETECTION_TYPES = {
    VEHICLE_DETECTOR: {
        name: 'VEHICLE_DETECTOR',
        htmlDescr: '<b>Vehicle Detector: </b> Detects cars with increased accuracy on high altitudes',
        refName: 'Vehicle Detector',
    },
    PERSON_DETECTOR: {
        name: 'PERSON_DETECTOR',
        htmlDescr: '<b>Person Detector: </b> Detects people with increased accuracy on high altitudes',
        refName: 'Person Detector',
    },
    VEHICLE_PERSON_DETECTOR: {
        name: 'VEHICLE_PERSON_DETECTOR',
        htmlDescr: '<b>Vehicle-Person Detector: </b> Detects cars and people with good accuracy on high altitudes',
        refName: 'Vehicle-Person Detector',
    },
    DISASTER_CLASSIFICATION: {
        name: 'DISASTER_CLASSIFICATION',
        htmlDescr: '<b>Disaster Classification: </b> Detects fire, earthquake and floods',
        refName: 'Disaster Classification',
    },
    CROWD_LOCALIZATION: {
        name: 'CROWD_LOCALIZATION',
        htmlDescr: '<b>Crowd Localization: </b> Detects and counts the number of people',
        refName: 'Crowd Localization',
    },      
    GENERAL_DETECTOR: {
        name: 'GENERAL_DETECTOR',
        htmlDescr: '<b>General Detector: </b> Detects 80 types of objects with good accuracy on low altitudes',
        refName: 'General Detector',
    },
};

let FLYING = 'Flying';
let PAUSED_MISSION = 'Paused_Mission';
let IN_MISSION = 'In_Mission';
let MAPLIBRE_STYLE = 'maplibre_style';
let LANDED = 'Landed';
let OWN_LAYER = 'own_layer';

let DEFAULT_MAP_CENTER = [33.272692254218605, 35.007696676570056];

let BASIC_STYLE_ID = 'basic-v2';
let MAPBOX_TERRAIN_LINES_V2_URL = 'mapbox://mapbox.mapbox-terrain-v2';
let DETECTION_STOPPED = -1;

let UPDATE_INTERVAL = 200; //miliseconds

let WEATHER_UPDATE_INTERVAL = 1000; //milli
let LORA_INFO_UPDATE_INTERVAL = 1000; //milli
let API_URL;
let WEB_SERVER_URL;
let LIVE_STREAM_HLS_URL;
let WEBSERVER_SPECTRAL_OUTPUT_PHOTOS_DIRECTORY;
let WEBSERVER_SPECTRAL_PHOTOS_CSV_FILE;
let MULTISPECTRAL_INDEX_NAMES;

var BASE_URL_DRONE_API;
var BASE_URL_DRONE_IDS_API;
var BASE_URL_DETECTED_OBJECTS_API;
var BASE_URL_LATEST_FRAME;
var BASE_URL_LATEST_FRAME_POSTFIX;
var BASE_URL_DETECTION_DRONES;
var BUILDMAP_PHOTO_LOCATIONS_API_URL;
var BUILDMAP_RGB_PHOTOS_DIR;
var BUILD_MAP_DRONES_API_URL;

var VIDEO_COVER_PHOTO_DRONE = '/static/aiders/imgs/drone_img.jpg';
var BASE_URL_COCO_NAMES;
var VIDEO_LOADING_PHOTO = '/static/aiders/imgs/black_img.jpg';
var BUILDMAP_PHOTOS_FOLDER_NAME_PREFIX_FOR_WEBSERVER;
var API_URL_BUILD_MAP_CURRENT_FILE_AND_FOLDER_PATHS;
var WEBSERVER_URL_FOR_BUILDMAP_METADATA;
var areConstantsDeclared = false;
var IN_PROGRESS_STRING = '';

var API_ROUTES = {
    Drones: SERVER_IP + '',
};

var SessionProfileKeys = {
    SELECTED_ALGORITHMS: 'selected_algorithms',
    SELECTED_MISSIONS: 'selected_missions',
    JOINED_OPERATION: 'joined_operation',
};
