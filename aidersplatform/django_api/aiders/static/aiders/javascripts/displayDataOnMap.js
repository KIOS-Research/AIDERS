/*
* Checks for any data the user might want to display on the map. If there are, they are then displayed
* This is called once, when the main page is loaded
* */
{
    window.onbeforeunload = function() {
        return "Data will be lost if you leave the page, are you sure?";
    };
    map.on('load', function ()
    {
        displayFireSpreadDetection()
        display3DObjectResults()
        displaySearchAndRescueMissionPaths()
        
    })

    function displayFireSpreadDetection()
    {
        setTimeout(function ()
        {
            // removeEl('#loading3DObj')
            $(`.${WARNING_ALERT}`).hide();
        },2000)

        let selected_fire_spread_objects=[]
        JSON.parse(sessionStorage.getItem(SessionProfileKeys.SELECTED_ALGORITHMS)).forEach(function (algorithm){
            if (algorithm['fields']['algorithm_name'] === "FIRE_PROPAGATION_ALGORITHM"){
                selected_fire_spread_objects.push(algorithm)
            }
        
        })
        if (selected_fire_spread_objects.length === 0)  return;
        selected_fire_spread_objects.forEach((selected_fire_spread_object)=>{
            RunFireSpreadOutput(selected_fire_spread_object)

        })

    }
    function displaySearchAndRescueMissionPaths()
    {
        setTimeout(function ()
        {
            $(`.${WARNING_ALERT}`).hide();
        },2000)

        let selected_search_and_rescue_paths =[]
        JSON.parse(sessionStorage.getItem(SessionProfileKeys.SELECTED_ALGORITHMS)).forEach(function (algorithm){
            if (algorithm['fields']['algorithm_name'] === "CALCULATE_SEARCH_AND_RESCUE_MISSION_PATHS_ALGORITHM"){
                selected_search_and_rescue_paths.push(algorithm)
            }
        })
        if (selected_search_and_rescue_paths.length === 0)  return;
        selected_search_and_rescue_paths.forEach((selected_search_and_rescue_path)=>{
            let paths = selected_search_and_rescue_path['fields']['output']['0']
            allDrones = get_all_drone_info_array()
            addLineLayer(paths, selected_search_and_rescue_path['pk'])
        })
    }

    function addLineLayer(pathObj, pk){
        let path = pathObj['path']
        path.pop()
        let droneID = pathObj['drone_id']
        if (map.getLayer('path_' + droneID)) {
            map.removeLayer('path_' + droneID);
        }
        if (map.getSource('path_' + droneID)) {
            map.removeSource('path_' + droneID);
        }
        map.addSource('path_' + droneID, {
            'type': 'geojson',
            'data': {
                'type': 'Feature',
                'properties': {'color': '#F7455D'},
                'geometry': {
                    'type': 'LineString',
                    'coordinates':  path
                }
            }
        });
        let allDrones = get_all_drone_info_array()
        if (allDrones.length > 0){
            map.addLayer({
                'id': 'path_' + droneID,
                'type': 'line',
                'source': 'path_' + droneID,
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round'
                },
                'paint': {
                    'line-color': getBasicColor(get_drone_index(droneID)),
                    'line-width': 8
                }
            },allDrones[0].droneModel.id);
        }else{
            map.addLayer({
                'id': 'path_' + droneID,
                'type': 'line',
                'source': 'path_' + droneID,
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round'
                },
                'paint': {
                    'line-color': getRandomColour(),
                    'line-width': 8
                }
            })
        }
        map.flyTo({center: [path[0][0],path[0][1]]})
    }
    function display3DObjectResults()
    {
        // create_popup_for_a_little(WARNING_ALERT, "Loading 3D Objects")

        setTimeout(function ()
        {
            // removeEl('#loading3DObj')
            $(`.${WARNING_ALERT}`).hide();
        },2000)
        // removeEl('loading3DObj')

        let selected_3d_objects=[]
        JSON.parse(sessionStorage.getItem(SessionProfileKeys.SELECTED_ALGORITHMS)).forEach(function (algorithm){
            if (algorithm['fields']['algorithm_name'] === "CREATE_3D_OBJECT_ALGORITHM"){
                selected_3d_objects.push(algorithm)
            }
        
        })
        if (selected_3d_objects.length === 0)  return;

        create_popup(WARNING_ALERT, "Loading 3D Objects...",'loading3DObj')

        selected_3d_objects.forEach((obj)=>{
            load3DObject(obj.fields.output.object_url, obj.fields.output.upper_left_coord)

        })
    }
}
