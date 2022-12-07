{
    // function fetch_path_plans(waypoints, drones)
    // {
    //     if (waypoints.length !== 4)
    //     {
    //         let msg = 'The grid-style mission requires exactly 4 waypoints!'
    //         create_dialog_with_one_button('OK',msg,'Warning','auto','auto')
    //         return
    //     }
    //
    //
    //
    //     let drone_ids = [];
    //     let batteries = [];
    //     let currentPositions = []
    //     const FULL_BATTERY = 1200
    //     for (let i = 0; i < drones.length; i++)
    //     {
    //         drone_ids.push(drones[i].droneID)
    //         batteries.push(FULL_BATTERY)
    //         currentPositions.push([drones[i].droneInfo.currentCoordinate[1], drones[i].droneInfo.currentCoordinate[0]]);
    //     }
    //
    //     let data =  JSON.stringify({
    //         'drone_ids': drone_ids,
    //         'lowerLeft': [waypoints[0][1], waypoints[0][0]], //lat and lon
    //         'upperLeft': [waypoints[1][1], waypoints[1][0]],
    //         'lowerRight': [waypoints[2][1], waypoints[2][0]],
    //         'upperRight': [waypoints[3][1], waypoints[3][0]],
    //         'xRange': 10,
    //         'yRange': 10,
    //         'batteries': batteries,
    //         'currentPositions': currentPositions
    //     })
    //
    //     // let settings = {
    //     //     "url": API_URL_CALCULATE_PATHS,
    //     //     "method": "POST",
    //     //     "timeout": 0,
    //     //     "headers": {"Content-Type": "application/json"},
    //     //     "data":data
    //     // };
    //
    //
    //
    //
    //
    // }


    function fetch_path_plans(waypoints, drones, alts) {



        let url = dutils.urls.resolve('algorithm_execute',{operation_name:CURRENT_OP})
        let drone_ids = [];
        let batteries = [];
        let currentPositions = []
        const FULL_BATTERY = 1200
        for (let i = 0; i < drones.length; i++)
        {
            drone_ids.push(drones[i].droneID)
            batteries.push(FULL_BATTERY)
            currentPositions.push([drones[i].droneInfo.currentCoordinate[1], drones[i].droneInfo.currentCoordinate[0]]);
        }
        let data =  JSON.stringify({
            'algorithmName': ALGORITHMS.CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM,
            'input':{
                'selectedDroneIDs': drone_ids,
                'lowerLeft': [waypoints[0][1], waypoints[0][0]], //lat and lon
                'upperLeft': [waypoints[1][1], waypoints[1][0]],
                'lowerRight': [waypoints[2][1], waypoints[2][0]],
                'upperRight': [waypoints[3][1], waypoints[3][0]],
                'xRange': 9,
                'yRange': 10,
                'batteries': batteries,
                'currentPositions': currentPositions,
                'currentAltitudes':alts
            },
            'canBeLoadedOnMap':true
            })

        return new Promise(function(resolve, reject) {
            var xhr = new XMLHttpRequest();
            xhr.responseType = 'json'
            xhr.onload = function() {
                resolve(this.response);
                console.log("RESPONSE: ", this.response)
            };
            xhr.onerror = reject;
            xhr.open("POST", url);
            xhr.setRequestHeader("Content-Type", "application/json");
            xhr.setRequestHeader('X-CSRFToken', document.getElementById('csrf').querySelector('input').value);
            xhr.onerror = function () {
                reject({
                    status: xhr.status,
                    statusText: xhr.statusText
                });
            };
            xhr.send(data);
        });
    }
}