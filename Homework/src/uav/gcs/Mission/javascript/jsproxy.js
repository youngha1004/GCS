var jsproxy = {
    log: function (level, methodName, message) {
        jsproxy.java.javascriptLog(level, methodName, message);
    },

    setMapZoom : function (zoom) {
        try{
            map.googlemap.setZoom(zoom);
            jsproxy.setZoomSliderValue(zoom);
        }catch (err) {
            jsproxy.log("ERROR", "jsproxy.setMapZoom()", err);
        }
    },

    setZoomSliderValue: function (zoom) {
        jsproxy.java.setZoomSliderValue(zoom);
    },

    setHomePosition: function (homeLat, homeLng) {
        try {
            if(map.uav != null){
                map.uav.setHomePosition(homeLat, homeLng);
            }
        } catch (err) {
            jsproxy.log("ERROR", "jsproxy.setHomePosition()", err);
        }
    },

    setCurrentPosition: function (currLat, currLng, heading) {
        try{
            if(map.uav == null){
                map.uav = new Uav();
                map.googlemap.setCenter({lat: currLat, lng: currLng});
                map.googlemap.setZoom(17);
            }
            map.uav.currentPosition = {lat: currLat, lng: currLng};
            map.uav.heading = heading;

        } catch (err) {
            jsproxy.log("ERROR", "jsproxy.setCurrentPosition()", err);
        }
    },
    
    manual: function (manualAlt) {
        map.manual = true;
        map.mission = false;
        map.fence = false;
        map.uav.manualAlt = manualAlt;
    },

    manualSend: function (latLng, manualAlt) {
        jsproxy.java.javascriptManual(latLng.lat(), latLng.lng(), manualAlt);
    },

    setMode: function (isGuided, isAuto, isRTL, isLand) {
        map.uav.setMode(isGuided, isAuto, isRTL, isLand);
    },


    missionMake: function () {
        map.manual = false;
        map.mission = true;
        map.fence = false;
    },
};