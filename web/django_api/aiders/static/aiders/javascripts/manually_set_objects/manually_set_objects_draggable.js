{   
    var object_on_the_move_id = null;

    let color = '#FFA500' ; 

    function dragrelease(e) {

        last_drag_point = e ; 

        ShowObjectMoveForm(last_drag_point)
    }

    function dragset(e) {

        let marker_on_the_move_id = e.target._element.id

        Manually_Set_Objects.forEach(function (el, index) {
            if ( marker_on_the_move_id === el['marker_id']) {
                object_on_the_move_id = el['id'];
            }
        });
    }

    function ShowObjectMoveForm(last_drag_point) {

        var coords = last_drag_point.target._lngLat ;
                    
        let titleIDEl     = document.getElementById('UserSetObjectMoveModalTitle');

        let latInputEl = document.getElementById('move_m_object_latInput');
        
        let lonInputEl = document.getElementById('move_m_object_lonInput');
        
        let IdInputEl = document.getElementById('move_m_object_id');
        
        titleIDEl.innerHTML = 'Change Object Location (Object ID: ' + object_on_the_move_id + ') :' ;
        latInputEl.value = coords.lat
        lonInputEl.value = coords.lng
        
        $("#UserSetObjectMoveModal").show() ;
    }
          
    function SubmitObjectLocation() {

        let data = formDataToJson($("#ObjectMoveForm").serializeArray())
        let data_o = JSON.stringify( data )
        //console.log(data_o)

        let json_data = JSON.stringify({

            "manually_set_object" : object_on_the_move_id,
            "lat": data.latitude,
            "lon": data.longitude,
        });

        $.ajax({
        
            type: "POST",
            
            url: dutils.urls.resolve('add_manual_object_location' , { operation_name: CURRENT_OP }) ,

            dataType: "json",
            contentType: "application/json",

            data: json_data,
            
            success: function (response)
            {

                // TODO: 
                // updateLineLayer(target_id_on_the_move, location )

                let index = null ;
                for (let j = 0; j < Manually_Set_Objects.length; j++)
                {   
                    //console.log(Manually_Set_Objects[j] ) 
                    if( object_on_the_move_id === Manually_Set_Objects[j]["id"] )
                    {
                        index = j
                        break
                    }
                }

                //console.log(object_on_the_move_id)
                if(index != null )
                {   
                    if( "ManuallySetObjectLocation" in response ) 
                    {   
                        //console.log("coords_set_by")
                        location_data = response.ManuallySetObjectLocation

                        Manually_Set_Objects[index]['currentLocation']['lat'] = parseFloat(data.latitude) ;
                        Manually_Set_Objects[index]['currentLocation']['lon'] = parseFloat(data.longitude);
                        
                        Manually_Set_Objects[index]['coords_set_by'] = location_data.coords_set_by ; 
                        Manually_Set_Objects[index]['coords_set_by_username'] = location_data.coords_set_by_username ; 
                        Manually_Set_Objects[index]['coords_set_at'] = location_data.coords_set_at ; 

                        let currentCoord = [
                            Manually_Set_Objects[index]['currentLocation']['lon'],
                            Manually_Set_Objects[index]['currentLocation']['lat'],
                            0,
                        ]; 

                        let prevLon = Manually_Set_Objects[index]['prevLocation']['lon'];
                        let prevLat = Manually_Set_Objects[index]['prevLocation']['lat'];

                        let prevCoord = [
                            prevLon,
                            prevLat,
                            0,
                        ]; 

                        if (prevLat === '' && prevLon === '') {
                            //We dont have previous location because this is the first time
                            prevCoord = currentCoord;
                        } 

                        let currentLineData = {
                            source: prevCoord,
                            dest: currentCoord,
                        };

                        Manually_Set_Objects[index]['prevLocation']['lat'] = Manually_Set_Objects[index]['currentLocation']['lat'];
                        Manually_Set_Objects[index]['prevLocation']['lon'] = Manually_Set_Objects[index]['currentLocation']['lon'];
                        Manually_Set_Objects[index]['lineData'] = Manually_Set_Objects[index]['lineData'].concat(currentLineData);
                        
                        let msoLineLayer = Manually_Set_Objects[index]['lineLayer'];
                    
                        if (msoLineLayer) {
                            map.setLayoutProperty(msoLineLayer.id, 'visibility', 'visible');
                            msoLineLayer.setProps({
                                data: Manually_Set_Objects[index]['lineData'],
                            });
                        }
                    }
                }
                
                // this is executed 100 miliseconds after the position change 
                // 
                // the reason is to allow the system (websocket receiving script) to ...
                // to read the new websocket message with the new coordinates from backend ... 
                // before we allow the manually_set_object_receiver to set again this markers position  
                setTimeout(() => {

                    //Note - during the alert ... the new message from ws is not being read
                    alert('Object Successfully Saved!');
                    object_on_the_move_id = null;
                    
                }, 100);

                $("#UserSetObjectMoveModal").hide() ;
            },
            error: function (error)
            {   
                alert("Object Not Saved! Error: ${error.responseText}" , {
                    position: 'center-center',
                    timeout: 2000,
                    showOnlyTheLastOne: true
                });

                CancelObjectLocation()            
            }
        });
    }

    function CancelObjectLocation() {
        
        removeManualMarkerFromMap(object_on_the_move_id); //Remove previous marker

        color = '#FFA500' ; 

        let indx = -1;
        Manually_Set_Objects.forEach(function (el, index) {
            if ( object_on_the_move_id === el['id']) {
                indx = index;
            }
        });

        currentLat = Manually_Set_Objects[indx]['currentLocation']['lat'] 
        currentLon = Manually_Set_Objects[indx]['currentLocation']['lon']
        description = Manually_Set_Objects[indx]['description']

        let lngLat = new maplibregl.LngLat(currentLon, currentLat);

        MSO_Marker = placeManualMarkerOnMap( lngLat, object_on_the_move_id, color, description, Manually_Set_Objects[indx] );
        Manually_Set_Markers[object_on_the_move_id] = MSO_Marker
        
        object_on_the_move_id = null;

        console.log('cancel_move')

        $("#UserSetObjectMoveModal").hide() ;
    }
        
}
