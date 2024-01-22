{
    let isTogglePressed = false
 
    var object_on_update_id = null;

    function toggleAddManuallySetObject(toggleButtonID)
    {
        isTogglePressed = $("#" + toggleButtonID).is(':checked');
    }
    
    map.on('click', function (e){
        
        //console.log(isTogglePressed)
        
        if ( isTogglePressed ) 
        {   
            //console.log(e)
            html_value =    `
                            <br>
                            <ul class="map_popup_list">
                            <li  onclick='displayAddUserSetObjectForm(${JSON.stringify(e.lngLat)})'> 
                            Add Object Location
                            </li>
                            </ul> `;

            new maplibregl.Popup()
            .setLngLat(e.lngLat)
            .setHTML(html_value)
            .addTo(map);
        }
    });

    function displayAddUserSetObjectForm(lngLat)
    {

        //console.log(lngLat.lat)
        //console.log(lngLat.lng)

        // set the coordinates in the form 
        let latInputEl = document.getElementById('add_user_set_obj_latInput');
        let lonInputEl = document.getElementById('add_user_set_obj_lonInput');
        
        latInputEl.value = lngLat.lat
        lonInputEl.value = lngLat.lng

        $("#addUserSetObjectModal").show() ;
    }

    function submitAddUserSetObjectData()
    {
        let data = formDataToJson($("#addUserSetObjectDataForm").serializeArray())

        let data_o = JSON.stringify( data )
        
        //console.log(data)
        //console.log(data_o)
        let acronym = data.acronym

        let object_data = JSON.stringify({
            "lat": data.latitude,
            "lon": data.longitude,
            "description": data.description,
        });

        $.ajax({

            type: "POST",
            url: dutils.urls.resolve('add_manual_object', { operation_name: CURRENT_OP }),       
            contentType: "application/json",
            data: object_data,
            success: function (response)
            {

                alert("Object Successfully Saved!");

                //$("#addUserSetObjectModal").modal('hide') ;
                $("#addUserSetObjectModal").hide() ;
            },
            error: function (error)
            {   
                alert("Object Not Saved! Error: ${error.responseText}");
                // alert(`Τα δεδομένα δεν καταχωρήθηκαν! Σφάλμα: ${error.responseText}`, {
                //         position: 'center-center',
                //         timeout: 2000,
                //         showOnlyTheLastOne: true
                //     });
            },
            dataType: "json"
        });
    }

    function formDataToJson(serializedFormData)
    {
        let object = {};
        serializedFormData.forEach(function(el, i){
            let key = el.name;
            let val = el.value;
            object[key] = val;
        });
        return JSON.parse(JSON.stringify(object));
    }

    function submitUpdateUserSetObjectData()
    {
        let data = formDataToJson($("#updateUserSetObjectDataForm").serializeArray())

        //console.log(data)

        let data_o = JSON.stringify( data )
        console.log(data_o)

        objID = data.object_id

        let object_data = JSON.stringify({
            "lat": data.latitude,
            "lon": data.longitude,
            "description": data.description,
            "manually_set_object": objID
        });

        $.ajax({

            type: "POST",
            url: dutils.urls.resolve('update_manual_object', { operation_name: CURRENT_OP , id: objID }),       
            contentType: "application/json",
            data: object_data,
            success: function (response)
            {
                //alert("Object Successfully Saved!");
                
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

                //console.log(objID)
                if(index != null )
                {   

                    // replace Marker and update saved markers
                    let color = '#FFA500' ; 
                    let lngLat = new maplibregl.LngLat(data.longitude, data.latitude);

                    console.log( objID )
                    removeManualMarkerFromMap(objID); //Remove previous marker
                    MSO_Marker = placeManualMarkerOnMap(lngLat, objID, color, data.description, Manually_Set_Objects[index] ); //Place new marker on map
                    Manually_Set_Markers[objID] = MSO_Marker

                    //update description
                    if( "ManuallySetObjectDescription" in response ) 
                    {   
                        //console.log("coords_set_by")
                        description_data =  response.ManuallySetObjectDescription

                        Manually_Set_Objects[index]['description'] = description_data.description;
                        Manually_Set_Objects[index]['updated_at'] = description_data.updated_at;
                        Manually_Set_Objects[index]['updated_by'] = description_data.updated_by;
    
                    }

                    //update location/line data + line layer
                    if( "ManuallySetObjectLocation" in response ) 
                    {   
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

                setTimeout(() => {

                    alert('Object Successfully Saved!');
                    object_on_update_id  = null;
                }, 100);

                //$("#updateUserSetObjectModal").modal('hide') ;
                
                $("#updateUserSetObjectModal").hide() ;
            },
            error: function (error)
            {   
                alert("Object Not Saved! Error: ${error.responseText}" , {
                    position: 'center-center',
                    timeout: 2000,
                    showOnlyTheLastOne: true
                });
            },
            dataType: "json"
        });

    }

}
