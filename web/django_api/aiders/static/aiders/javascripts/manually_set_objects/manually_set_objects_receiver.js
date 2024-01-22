{
    var Manually_Set_Objects = [];
    var Manually_Set_Markers = {};

    let MSO_Interval;
    let infoUpdateInterval;

    var showMSO = false ;
    var firstTime = true;

    function startStopMSO(toggleButtonID) {
        
        let pressed = $(`#${toggleButtonID}`).is(':checked');
        showMSO = pressed;
        if (pressed) {
            startPointOfInterestMarker()
            showMSO = pressed;
        } else {
            stopPointOfInterestMarkers()
            hideMSOFromMap();
        }
    }

    function hideMSOFromMap() {
        Manually_Set_Objects.forEach(function (mso_device, indx) {
            
            //let MSOInfoBox = $('#MSO_Box' + indx);
            //MSOInfoBox.hide();
            
            map.setLayoutProperty(mso_device.lineLayer.id, 'visibility', 'none');

            removeManualMarkerFromMap(mso_device.id);

            firstTime = true; 
        });
    }

    // TODO : implement function to present data for manually set objects 
    // function presentMSOInfoBox(toggleButtonID) {
    //     infoUpdateInterval = setInterval(function () {
    //         // present_data
    //     }, MSO_INFO_UPDATE_INTERVAL);
    //     //    function presentMSOData() {
    //     //    });
    //     }
    // }

    function MSOUpdate( received_MSOs ) {  

        if( showMSO == true){

            for ( let key in received_MSOs ) {

                if( ( key != object_on_the_move_id ) && ( key != object_on_update_id ) ){ 

                    let obj_id = key;

                    let color = '#FFA500' ; 
                    let MSO_Object = null

                    let currentLon = received_MSOs[key]['lon'];
                    let currentLat = received_MSOs[key]['lat'];
                    let description = received_MSOs[key]['description'];

                    let created_by = received_MSOs[key]['created_by'];
                    let created_by_username = received_MSOs[key]['created_by_username'];

                    let coords_set_by = received_MSOs[key]['coords_set_by'];
                    let coords_set_by_username = received_MSOs[key]['coords_set_by_username'];
                    let coords_set_at = received_MSOs[key]['coords_set_at']

                    let updated_at = received_MSOs[key]['updated_at'];
                    let updated_by = received_MSOs[key]['updated_by'];

                    if ( description == null ){ description= '' }
                    
                    let lngLat = new maplibregl.LngLat(currentLon, currentLat);

                    let indx = -1;
                    Manually_Set_Objects.forEach(function (el, index) {
                        if ( obj_id === el['id']) {
                            indx = index;
                        }
                    });

                    if (indx === -1) {

                        //Object does not exist. We have to add it in the array
                        const tempMSObj = new Object();

                        let opacity = 0.5;
                        let lineLayer = createMSOLineLayer(obj_id, color, opacity);

                        tempMSObj.currentLocation = {};
                        tempMSObj.currentLocation.lat = currentLat;
                        tempMSObj.currentLocation.lon = currentLon;
                        tempMSObj.id = obj_id;
                        tempMSObj.color = color;
                        tempMSObj.lineLayer = lineLayer;
                        tempMSObj.prevLocation = {};
                        tempMSObj.prevLocation.lat = currentLat;
                        tempMSObj.prevLocation.lon = currentLon;
                        tempMSObj.lineData = [];
                        tempMSObj.description = description;
                        
                        tempMSObj.created_by = created_by
                        tempMSObj.created_by_username = created_by_username

                        tempMSObj.coords_set_by = coords_set_by
                        tempMSObj.coords_set_by_username = coords_set_by_username
                        tempMSObj.coords_set_at = coords_set_at

                        tempMSObj.updated_at = updated_at;
                        tempMSObj.updated_by = updated_by;

                        Manually_Set_Objects.push(tempMSObj);
                        MSO_Object = tempMSObj

                        map.addLayer(lineLayer);

                        MSO_Marker = placeManualMarkerOnMap(lngLat, obj_id, color, description, MSO_Object ); //Place new marker on map
                        Manually_Set_Markers[obj_id] = MSO_Marker    
                    } 
                    // MSO exists. We now just have to update its location and description
                    else 
                    {   
                        MSO_Object = Manually_Set_Objects[indx]
                        MSO_Marker = Manually_Set_Markers[obj_id]

                        if( MSO_Marker == null ) 
                        {
                            MSO_Marker = placeManualMarkerOnMap(lngLat, obj_id, color, description, MSO_Object ); //Place new marker on map
                            Manually_Set_Markers[obj_id] = MSO_Marker
                        }
                        else if( updated_at != Manually_Set_Objects[indx]['updated_at'] ) 
                        {
                            removeManualMarkerFromMap(obj_id); //Remove previous marker
                            MSO_Marker = placeManualMarkerOnMap(lngLat, obj_id, color, description, MSO_Object ); //Place new marker on map
                            Manually_Set_Markers[obj_id] = MSO_Marker
                        }
                        // if the above is true ... the LngLat will be reset above ... no need to execude below else !!!
                        else if( coords_set_at != Manually_Set_Objects[indx]['coords_set_at'])
                        {
                            MSO_Marker.setLngLat(lngLat);
                        }

                        // Information Update
                        Manually_Set_Objects[indx]['currentLocation']['lat'] = currentLat;
                        Manually_Set_Objects[indx]['currentLocation']['lon'] = currentLon;
                        Manually_Set_Objects[indx]['description'] = description;
                        Manually_Set_Objects[indx]['marker_id'] = 'MSO' + obj_id ;

                        Manually_Set_Objects[indx]['created_by'] = created_by ; 
                        Manually_Set_Objects[indx]['created_by_username'] = created_by_username ; 

                        Manually_Set_Objects[indx]['coords_set_by'] = coords_set_by ; 
                        Manually_Set_Objects[indx]['coords_set_by_username'] = coords_set_by_username ; 
                        Manually_Set_Objects[indx]['coords_set_at'] = coords_set_at ; 

                        Manually_Set_Objects[indx]['updated_at'] = updated_at;
                        Manually_Set_Objects[indx]['updated_by'] = updated_by;

                        //Following code is needed to update the line layer of the MSO
                        let currentCoord = [currentLon, currentLat, 0];
                        
                        let prevCoord = [
                            Manually_Set_Objects[indx]['prevLocation']['lon'],
                            Manually_Set_Objects[indx]['prevLocation']['lat'],
                            0,
                        ];

                        let prevLon = Manually_Set_Objects[indx]['prevLocation']['lon'];
                        let prevLat = Manually_Set_Objects[indx]['prevLocation']['lat'];
                        
                        if (prevLat === '' && prevLon === '') {
                            //We dont have previous location because this is the first time
                            prevCoord = currentCoord;
                        }

                        let currentLineData = {
                            source: prevCoord,
                            dest: currentCoord,
                        };

                        Manually_Set_Objects[indx]['prevLocation']['lat'] = currentLat;
                        Manually_Set_Objects[indx]['prevLocation']['lon'] = currentLon;
                        Manually_Set_Objects[indx]['lineData'] = Manually_Set_Objects[indx]['lineData'].concat(currentLineData);
                        
                        let msoLineLayer = Manually_Set_Objects[indx]['lineLayer'];
                        if (msoLineLayer) {
                            map.setLayoutProperty(msoLineLayer.id, 'visibility', 'visible');
                            msoLineLayer.setProps({
                                data: Manually_Set_Objects[indx]['lineData'],
                            });
                        }

                    }

                    /* Start Message */ 
                    if (firstTime) {
                        let msg = 'Successfully started receiving Manually Set Objects!';
                        create_popup_for_a_little(SUCCESS_ALERT, msg, 2000);
                        map.flyTo({ center: [currentLon, currentLat], zoom: 20 });
                        firstTime = false;
                    }
                }
            }               
        }
    }

    // function stopReceivingMSOLocations() {
    //     clearInterval(MSO_Interval);
    //     clearInterval(infoUpdateInterval);
    //     let msg = 'Stopped Showing Mannually Set Objects!';
    //     create_popup_for_a_little(WARNING_ALERT, msg, 2000);
    // }

    function removeManualMarkerFromMap(obj_id) {

        let marker_id = 'MSO' + obj_id;

        //let MSO_Marker = $('#' + marker_id) ;
        let MSO_Marker = Manually_Set_Markers[obj_id] 

        let marker_popup = MSO_Marker.getPopup();

        if(marker_popup.isOpen()){
            MSO_Marker.togglePopup();
        }

        Manually_Set_Markers[obj_id] = null 

        $('#' + marker_id).remove();
    }

    function placeManualMarkerOnMap(lngLat, tagName, color, description , Manually_Set_Object ) {
        
        console.log('placing marker : ' + tagName )
        
        let el = document.createElement('div');
        let marker_id = 'MSO' + tagName;
        el.id = marker_id;
        el.className = 'marker';
        color = 'background-color:' + color + ';';
        let inside_marker = '<span style=' + color + '><b>';
        el.innerHTML = inside_marker + 'M' + tagName + '</b></span>';
        
        html_value =    `
                        <p>Object Id: ${tagName}</p>
                        <ul class="map_popup_list">
                        <li>  
                        <span>Description:</span>
                        <br>
                        <span>${description}</span>                     
                        </li>
                        <li>
                        <span>Created By:</span>
                        <br>
                        <span>${Manually_Set_Object['created_by_username']}</span>                     
                        </li>
                        <li>
                        <span>Location Set By:</span>
                        <br>
                        <span>${Manually_Set_Object['coords_set_by_username']}</span>
                        </li>
                        <li
                        onclick='displayUpdateUserSetObjectForm("${tagName}")' >
                        Edit Object Description
                        </li>
                        </ul> `;

        var popup = new maplibregl.Popup()
            .setHTML(html_value);
    
        let MSO_Marker = new maplibregl.Marker(el).setLngLat(lngLat).setPopup(popup).addTo(map);

        MSO_Marker.setDraggable(true);
        MSO_Marker.on( 'dragend' ,   dragrelease);
        MSO_Marker.on( 'dragstart' ,   dragset);

        return( MSO_Marker )
    }

    function displayUpdateUserSetObjectForm(objID)
    {   
        object_on_update_id = objID ;

        let index = null ;
        for (let j = 0; j < Manually_Set_Objects.length; j++)
        {   
            //console.log(Manually_Set_Objects[j] ) 
            if( objID === Manually_Set_Objects[j]["id"] )
            {
                index = j
                break
            }
        }

        //console.log(index)
        if(index != null )
        {
            // set the coordinates in the form 
            let titleIDEl     = document.getElementById('updateUserSetObjectModalLabel');
            let latInputEl  = document.getElementById('upd_user_set_obj_latInput');
            let lonInputEl  = document.getElementById('upd_user_set_obj_lonInput');
            let descInputEl = document.getElementById('upd_user_set_obj_descInput');
            let idInputEl = document.getElementById('upd_user_set_obj_id');

            //console.log(Manually_Set_Objects[index]['currentLocation']['lon'] )

            titleIDEl.innerHTML     = 'Update Object Description (Object ID: ' + objID + ')' ;
            latInputEl.value    = Manually_Set_Objects[index]['currentLocation']['lat'] ;
            lonInputEl.value    = Manually_Set_Objects[index]['currentLocation']['lon'] ;
            descInputEl.value   = Manually_Set_Objects[index]['description'] ;
            idInputEl.value     = objID ;
            
            // Show Form Modal
            $("#updateUserSetObjectModal").show() ;
        }
    }

    function createMSOLineLayer(obj_id, color, opacity ) {
        return new MapboxLayer({
            id: 'MSO' + obj_id + 'line',
            type: LineLayer,
            data: [],
            fp64: false,
            widthScale: 0.1,
            getWidth: 45, //Change getWidth and widthScale to fine-tune the line width
            opacity: opacity,
            widthUnit: 'meters',
            // getStrokeWidth: 6,
            getSourcePosition: (d) => d.source,
            getTargetPosition: (d) => d.dest,
            getColor: hexToRgb(color), 
        });
    }

    function CancelObjectUpdate() {
                
        console.log('cancel_update')

        object_on_update_id = null;

        $("#updateUserSetObjectModal").hide() ;
    }
    


}
