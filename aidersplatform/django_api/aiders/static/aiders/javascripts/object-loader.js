{

    function loadExternalODMAPI()
    {

        let msg = "A new window will now appear, that uses the  <i>OpenDroneMap API</i>. " +
            "<br><br>Once you start the process of the 3D Object Creation, " +
            "you can close the new window and let it run in the background.<br><br>You can check its progress by opening the same window again"
        create_confirmation_dialog('OK','Cancel',msg,'Confirm').then(function (canProceed)
        {
            if (canProceed)
            {
                let params = `scrollbars=no,resizable=no,status=no,location=no,toolbar=no,menubar=no,
                                width=1000,height=700,left=100,top=100`;
                let win = open("http://"+document.location.hostname+":4000", 'test', params);
                // var timer = setInterval(function() {
                //     if(win.closed) {
                //         clearInterval(timer);
                //         let msg = "Open the same window to chec!"
                //         create_popup_for_a_little(WARNING_ALERT,msg,4000)
                //     }
                // }, 500);
            }
        })

    }


    function onClick(event)
    {
        event.target.value = ''
    }

    function load3DObject(objUri, topLeftCoord)
    {
        objUri.replace('*IP_HERE*',location.hostname)
        let mtlUri = objUri.substr(0, objUri.lastIndexOf(".")) + ".mtl";
        map.addLayer({
            id: 'object_' + objUri,
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                let options = {
                    type: 'mtl',
                    obj: objUri,
                    mtl: mtlUri,
                    scale: 1,
                    units: 'meters',
                    rotation: { x: 0, y: 0, z: 180 }, //default rotation
                    anchor: 'top-left',
                    uri: objUri,
                    layerId:'object_' + objUri,
                    clone: false
                }

                tb.loadObj(options, function (model) {
                    let mesh = model.setCoords(topLeftCoord);
                    tb.add(mesh);
                    let msg = "The 3D object was loaded successfully!"

                    create_popup_for_a_little(SUCCESS_ALERT, msg, 2000)
                    remove_all_build_map_periods()
                    $(`.${WARNING_ALERT}`).hide();
                })

            },

            render: function (gl, matrix) {
            }
        });

        setTimeout(function ()
        {
            map.flyTo({center:topLeftCoord, zoom: 15});
        },1500)

    }

    function clearObjectFromMap()
    {
        let okButton = "Clear"
        let cancelButton = "Cancel"
        let message = "Are you sure you want to clear all 3d objects from the map? "
        let dialogTitle = "Confirm"

        create_confirmation_dialog(okButton,cancelButton,message,dialogTitle).then(function (willClear)
        {
            if (willClear)
            {
                let all_threebox_layers = tb.world.children;
                let object_found = false
                for (let i=0; i< all_threebox_layers.length; i++)
                {
                    let obj = all_threebox_layers[i]
                    if (obj.userData.uri !== undefined && obj.userData.uri.includes('object_3d_results_'))
                    {
                        object_found = true
                        tb.remove(obj)
                        tb.removeLayer(obj.userData.layerId)
                    }
                    if (object_found)
                    {
                        create_popup_for_a_little(SUCCESS_ALERT,'Object(s) removed from the map!', 2000)
                    }
                    else
                    {
                        create_popup_for_a_little(WARNING_ALERT,'No objects found!', 2000)
                    }
                }
            }
        })
    }
}
