MapBoxToken = 'pk.eyJ1IjoiZ2VvcmdlMjMyMyIsImEiOiJja2MwZmxjbGYxajF4MnJsZ2pzbjhjdHc2In0.znh7LExrIEsKBB7SWYJ3hg';
const { MapboxLayer, LineLayer, COORDINATE_SYSTEM } = deck;
let currentStyle = sessionStorage.getItem('currentStyle');
let currentStyleRadioBtn = sessionStorage.getItem('currentStyleRadioBtn');
let lastDroneLocation = JSON.parse(sessionStorage.getItem('lastDroneLocation'));
if (currentStyle === null) {
    //If it's the first time loading the platform, this variable is null
    currentStyle = 'https://api.maptiler.com/maps/<>/style.json?key=blpQOMdNw0JJIq07I9Ln'.replace('<>', BASIC_STYLE_ID);
}

if (currentStyleRadioBtn === null) {
    currentStyleRadioBtn = 'basic-v2';
}

if (lastDroneLocation === null || typeof lastDroneLocation === 'undefined' || lastDroneLocation.length === 0) {
    lastDroneLocation = DEFAULT_MAP_CENTER;
}

$('#' + currentStyleRadioBtn).prop('checked', true);

if (!USE_ONLINE_MAPS) {
    currentStyle = {
        version: 8,
        sources: {
            osm: {
                type: 'raster',
                tiles: ['http://0.0.0.0:8081/tile/{z}/{x}/{y}.png'],
                tileSize: 256,
            },
        },
        layers: [
            {
                id: 'osm',
                type: 'raster',
                source: 'osm',
            },
        ],
    };
}
console.log(currentStyle);
maplibregl.accessToken = MapBoxToken;
var map = new maplibregl.Map({
    container: 'map',
    style: currentStyle,
    zoom: 7,
    center: [33.0028145, 35.0725358],
});

/*This element is a searchbox that allows user to search locations*/
var geocoder = new MapboxGeocoder({
    accessToken: MapBoxToken,
    placeholder: "Try 'Cyprus' or '33,35' or a drone's ID",
    mapboxgl: maplibregl,
    marker: true,
    localGeocoder: coordinatesGeocoder,
});

document.getElementById('geocoder').appendChild(geocoder.onAdd(map));

var buildingPopup;

map.addControl(new maplibregl.NavigationControl()); // Add zoom and rotation controls to the map.


// Add the Screen shot feature for MapBox
map.addControl(
    new MaplibreExportControl({
        accessToken: MapBoxToken,
        PageSize: Size.A3,
        PageOrientation: PageOrientation.Portrait,
        Format: Format.PNG,
        DPI: DPI[96],
        Crosshair: true,
        PrintableArea: true,
        Local: 'en',
    }),
    'top-right'
);

//Adding Threebox library to mapbox to add built-in mouseover/mouseout and click behaviors
window.tb = new Threebox(map, map.getCanvas().getContext('webgl'), {
    defaultLights: true,
    enableSelectingFeatures: true, //change this to false to disable fill-extrusion features selection
    enableSelectingObjects: true, //change this to false to disable 3D objects selection
    enableDraggingObjects: true, //change this to false to disable 3D objects drag & move once selected
    enableRotatingObjects: true, //change this to false to disable 3D objects rotation once selected
    enableTooltips: false, // change this to false to disable default tooltips on fill-extrusion and 3D models
    outlinePass: false,
    multiLayer: true,
});
// tb.renderer.outputEncoding = THREE.sRGBEncoding
tb.renderer.outputEncoding = THREE.LinearEncoding;
