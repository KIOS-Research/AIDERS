{

    const CC_WEB_SOCKET_TIMER = 1000;
    const CC_WEB_SOCKET_ADDRESS = "ws://" + window.location.hostname + ":" + NGINX_PORT + "/ws/getCrisisClassificationData";

    let CcWebSocket;

    let lastCrisisClassificationId = 0;
    let lastCrisisClassificationTimestamp = "2023-01-01 00:00:00"; // Default to a date far in the past

    function initWebsocketForCc(_address) {
        console.log(_address);
        let ws = new WebSocket(_address + "?token=" + encodeURIComponent(TOKEN));
        ws.addEventListener("open", function (event) {
            console.log("CC WebSocket connection established.");
        });
        ws.addEventListener("error", function (event) {
            console.error("CC Socket encountered error: Closing connection");
        });
        ws.addEventListener("message", function (event) {
            handleIncomingCCWebsocketMessage(JSON.parse(event.data));
        });
        return ws;
    }

    function handleIncomingCCWebsocketMessage(_wsMessage) {
        // console.log("Received data from server");

        if (_wsMessage.crisis_data && Array.isArray(_wsMessage.crisis_data) && _wsMessage.crisis_data.length > 0) {
            lastCrisisClassificationId = _wsMessage.crisis_data[_wsMessage.crisis_data.length-1].id;

            // get the most recent object based on timestamp or updated_at
            const mostRecent = getMostRecentObject(_wsMessage.crisis_data);
            // console.log("Most recent object:", mostRecent);
            lastCrisisClassificationTimestamp = (mostRecent.updated_at && mostRecent.updated_at > mostRecent.timestamp) ? mostRecent.updated_at : mostRecent.timestamp;

            // console.log("Crisis Last ID:", _wsMessage.crisis_data[_wsMessage.crisis_data.length - 1].id);
            console.log("Crisis Last Timestamp:", lastCrisisClassificationTimestamp);
            // console.log("Crisis Data:", _wsMessage.crisis_data);
            addCrisisIncidentToMap(_wsMessage.crisis_data);
            updateCrisisLogs(_wsMessage.crisis_data);
        }
        else {
            // console.log("No new crisis data.");
        }
    }


    function getMostRecentObject(arr) {
        return arr.reduce((latest, item) => {
            // Get the latest date for the current item
            const itemDate = new Date(
                (item.updated_at && item.updated_at > item.timestamp) ? item.updated_at : item.timestamp
            );
            // Get the latest date for the current "latest" object
            const latestDate = new Date(
                (latest.updated_at && latest.updated_at > latest.timestamp) ? latest.updated_at : latest.timestamp
            );
            return itemDate > latestDate ? item : latest;
        });
    }


    function closeWebsocketForCC() {
        if (CcWebSocket) {
            CcWebSocket.close();
            console.log("Crisis Classification WebSocket connection closed.");
        }
    }

    // Send Web socket
    function sendCrisisClassificationWebSocketMessage() {
        if (CcWebSocket.readyState == 1) {
            CcWebSocket.send(
                JSON.stringify({
                    get_all_data: true,
                    last_id: lastCrisisClassificationId,
                    last_timestamp: lastCrisisClassificationTimestamp
                })
            );
        }
    }

    function connectToCCWebsocket() {

        if (CcWebSocket == undefined) {
            initializeCrisisModels()
            //testtest()
            CcWebSocket = initWebsocketForCc(CC_WEB_SOCKET_ADDRESS)
            setInterval(sendCrisisClassificationWebSocketMessage, CC_WEB_SOCKET_TIMER)
        }

    }

    connectToCCWebsocket();

}