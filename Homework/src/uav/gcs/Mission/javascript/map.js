var map = {
    // Google Map Object store field
    googlemap : null,

    // UAV Object store field
    uav: null,

    // Map Click events
    manual: false,
    mission: false,
    fence: false,

    // Map download, map event handle, uav drawing starts
    init:function () {
        try{
            map.googlemap = new google.maps.Map(
                document.getElementById('map'),
                {
                    center: {lat: 37.561313, lng: 126.944942},
                    zoom: 3,
                    mapTypeControl: false,
                    mapTypeId: "satellite",
                    streetViewControl: false,
                    zoomControl: false,
                    rotateControl: false,
                    fullscreenControl: false
                }
            );

            // resize map using mouse wheel
            document.getElementById("map").addEventListener("wheel", function (ev) {
                var zoom = map.googlemap.getZoom();
                if(zoom < 3){
                    zoom = 3;
                    jsproxy.setMapZoom(zoom);
                }
                jsproxy.setZoomSliderValue(zoom);
            });

            // draw UAV every seconds when map loading finished
            // when download finishes, idle event occurs only once
            google.maps.event.addListenerOnce(
                map.googlemap,
                "idle",
                function () {
                    map.uavDraw.start();
                }
            );

            // Handle every event of mouse clicking the google map
            map.googlemap.addListener(
                'click',
                function (ev) {
                    if(map.manual){
                        map.uav.manual(ev.latLng);
                    } else if(map.mission){
                        var missionItem = {
                            seq: map.uav.missionMarkers.length,
                            command: "WAYPOINT",
                            param1: 0,
                            param2: 0,
                            param3: 0,
                            param4: 0,
                            x: ev.latLng.lat(),
                            y: ev.latLng.lng(),
                            z: 0,
                        };
                        map.uav.missionMarkerMake(missionItem);

                    } else if(map.fence){

                    }
                }
            );

        } catch (err) {
            jsproxy.log("ERROR", "map.init()", err.toString());
        }
    },


    uavDraw: {
        count: 1,
        start: function () {
            try{
                setInterval(
                    function () {
                        if(map.uav != null){
                            map.uav.drawUav();
                            if(map.uavDraw.count == 3){
                                map.googlemap.panTo(map.uav.currentPosition);
                                map.uavDraw.count = 1;
                            }else{
                                ++map.uavDraw.count;
                            }
                        }
                    },
                    1000
                );

            } catch (err) {
                jsproxy.log("ERROR", "map.uavDraw()", err);
            }
        }
    }
};