const PROTOCOL = window.location.protocol;
const HOST = window.location.host;

let baseUrl = `${PROTOCOL}//${HOST}/`;


// Function to make a POST request and return a JSON response
async function postRequest(_url, _data) {
    try {
        const response = await fetch(baseUrl + _url, {
            method: "POST",
            headers: {
                "Content-Type": "application/json",
                "X-CSRFToken": CSRF_TOKEN,
            },
            body: JSON.stringify(_data),
        });

        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
        const responseData = await response.json();
        return responseData;
    } catch (error) {
        console.error("Error:", error);
        throw error;
    }
}

async function getRequest(_url) {
    try {
        const response = await fetch(baseUrl + _url);
        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
        const responseData = await response.json();
        return responseData;
    } catch (error) {
        console.error("Error:", error);
        throw error;
    }
}

/////////////
/// DRONE ///
/////////////

// BUILD MAP
// GET
function getBuildMapSessions() {
    const url = `api/operations/${CURRENT_OP}/getBuildMapSessions`;
    let data = getRequest(url);
    return data;
}
// POST
function postGetActiveBuildMapSessionImages(_data) {
    const url = `api/operations/${CURRENT_OP}/getActiveBuildMapSessionImages`;
    let data = postRequest(url, _data);
    return data;
}
function postStartOrStopBuildMapRequest(_data) {
    const url = `api/operations/${CURRENT_OP}/buildMapStartOrStop`;
    let data = postRequest(url, _data);
    return data;
}
function postGetImagesByUsingSessionId(_data) {
    const url = `api/operations/${CURRENT_OP}/getBuildMapImagesBySessionId`;
    let data = postRequest(url, _data);
    return data;
}

// MISSION
// POST
function postGetMissionPointsFromMissionId(_data) {
    const url = `missionGetMissonPointsByMissionId`;
    let data = postRequest(url, _data);
    return data;
}

// DETECTION
// POST
function postStartOrStopDetectionRequest(_id, _data) {
    const url = `/api/operations/${CURRENT_OP}/set_detected_object_description/${_id}`;
    let data = postRequest(url, _data);
    return data;
}
function postGetAllDetectedObjectsAndDescriptionByOperationId(_data) {
    const url = `postGetAllActiveSessionDetectionObjectsFromOperationId/`;
    let data = postRequest(url, _data);
    return data;
}
function postUpdateDetectionObjectDescription(_data){
    const url = `postUpdateDetectionObjectDescriptionById/`;
    let data = postRequest(url, _data);
    return data;
}

function postGetAllDetectionInfoByDroneIdAndSessionId(_data) {
    const url = `postGetAllDetectionInfoFromDroneIdAndSessionId/`;
    let data = postRequest(url, _data);
    return data;
}
////////////
/// LORA ///
////////////


//running averages

function getAllBaloraPM1SensorData() {
    const url = `api/operations/${CURRENT_OP}/balora_pm1`;
    let data = getRequest(url);
    return data;
}

function getAllBaloraPM25SensorData() {
    const url = `api/operations/${CURRENT_OP}/balora_pm25`;
    let data = getRequest(url);
    return data;
}

function getAllBaloraNOxSensorData() {
    const url = `api/operations/${CURRENT_OP}/balora_nox`;
    let data = getRequest(url);
    return data;
}

function getAllBaloraVOCSensorData() {
    const url = `api/operations/${CURRENT_OP}/balora_voc`;
    let data = getRequest(url);
    return data;
}

// function getAllBaloraTempAverageSensorData() {
//     const url = `api/operations/${CURRENT_OP}/balora_temp_avg`;
//     let data = getRequest(url);
//     return data;
// }

// function getAllBaloraPM1HumiditySensorData() {
//     const url = `api/operations/${CURRENT_OP}/balora_humidity_avg`;
//     let data = getRequest(url);
//     return data;
// }

////////////////
/// INTERNAL ///
////////////////

// COVERAGE
function postGetCoveragePoints(_data) {
    const url = `coverage_points/${CURRENT_OP}`;
    let data = postRequest(url, _data);
    return data;
}

////////////////
/// EXTERNAL ///
////////////////

// MAPBOX
function getMapboxInformationOnAreaByLongitudeAndLatitude(url) {
    let data = getRequest(url);
    return data;
}

