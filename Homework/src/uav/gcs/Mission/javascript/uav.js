var Uav = function () {
    // draw UAV on map //지도상에 Uav 드로잉
    this.drawUav = function () {
        try{
            this.uavFrameIcon.rotation = 45 + this.heading;
            this.uavFrame.setIcon(this.uavFrameIcon);
            this.uavFrame.setPosition(this.currentPosition);


            this.uavFrameHeadIcon.rotation = 45 + this.heading;
            this.uavFrameHead.setIcon(this.uavFrameHeadIcon);
            this.uavFrameHead.setPosition(this.currentPosition);

            this.drawHeadingLine();
            this.drawDestinationLine();

        } catch (err) {
            jsproxy.log("ERROR", "Uav.drawUav()", err);
        }
    };

    // Heading line //지도상에 헤딩 라인(빨간선) 드로잉
    this.headingLine = null;

    // draw heading line on map
    this.drawHeadingLine = function () {
        try{
            //기존에 헤딩라인을 맵에서 제거
            if(this.headingLine != null){
                this.headingLine.setMap(null);
            }

            var startPoint = new google.maps.LatLng(this.currentPosition);
            var endPoint = google.maps.geometry.spherical.computeOffset(startPoint, 500000, this.heading);

            //헤딩라인 생성
            this.headingLine = new google.maps.Polyline({
                path: [startPoint, endPoint],
                strokeColor: "#f00000",
                strokeWeight: 2,
                map: map.googlemap
            });

        } catch (err) {
            jsproxy.log("ERROR", "Uav.drawHeadingLine()", err);
        }
    };

    // draw destination line on map // 지도상에 비행 목표 방향 라인(오렌지선) 드로잉
    this.destinationPosition = null;
    this.destinationLine = null;
    this.drawDestinationLine = function () {
        try{
            // delete last destination line on map //기존 목표 방향 라인을 맵에서 제거
            if(this.destinationLine != null){
                this.destinationLine.setMap(null);
            }

            // set Destination of each modes //비행 모드별 목표 위치 설정
            this.destinationPosition = this.currentPosition;
            if(this.modeGuided && (this.manualTargetMarker != null)){
                this.destinationPosition = this.manualTargetMarker.getPosition().toJSON();
            } else if(this.modeAuto){

            } else if(this.modeRTL){
                this.destinationPosition = this.homePosition;
            }

            // destination heading angle //각도 얻기
            var angle = google.maps.geometry.spherical.computeHeading(
                new google.maps.LatLng(this.currentPosition),
                new google.maps.LatLng(this.destinationPosition)
            );
            if(angle < 0){
                angle += 360;
            }
            var startPoint = new google.maps.LatLng(this.currentPosition);
            var endPoint = google.maps.geometry.spherical.computeOffset(startPoint, 500000, angle);
            //목표 방향 라인 생성
            this.destinationLine = new google.maps.Polyline({
                path: [startPoint, endPoint],
                strokeColor: "#FE9A2E",
                strokeWeight: 2,
                map: map.googlemap
            });

        } catch (err) {
            jsproxy.log("ERROR", "Uav.drawDestinationLine()", err);
        }

    }

    // Drone //드론 기체
    this.uavFrame = new google.maps.Marker({
        map: map.googlemap,
        optimized: false,
        zIndex: google.maps.Marker.MAX_ZINDEX + 1
    });
    this.uavFrameIcon = {
        path:"M-30.14012510194162,-0.6472990536338017 C-30.14012510194162,-4.36100347511945 -26.9396213477545,-7.369104056522829 -22.988382145054345,-7.369104056522829 C-19.037142942354194,-7.369104056522829 -15.836639188167071,-4.36100347511945 -15.836639188167071,-0.6472990536338017 C-15.836639188167071,3.06640536785185 -19.037142942354194,6.074505949255226 -22.988382145054345,6.074505949255226 C-26.9396213477545,6.074505949255226 -30.14012510194162,3.06640536785185 -30.14012510194162,-0.6472990536338017 z M-6.711553535002267,24.0669865286702 C-6.711553535002267,20.353282107184548 -3.511049780815142,17.34518152578117 0.44018942188500887,17.34518152578117 C4.39142862458516,17.34518152578117 7.5919323787722846,20.353282107184548 7.5919323787722846,24.0669865286702 C7.5919323787722846,27.78069095015585 4.39142862458516,30.788791531559227 0.44018942188500887,30.788791531559227 C-3.511049780815142,30.788791531559227 -6.711553535002267,27.78069095015585 -6.711553535002267,24.0669865286702 z M17.145587020975555,0.20984434846580768 C17.145587020975555,-3.503860073019837 20.346090775162672,-6.51196065442322 24.29732997786283,-6.51196065442322 C28.24856918056298,-6.51196065442322 31.449072934750106,-3.503860073019837 31.449072934750106,0.20984434846580768 C31.449072934750106,3.923548769951452 28.24856918056298,6.931649351354835 24.29732997786283,6.931649351354835 C20.346090775162672,6.931649351354835 17.145587020975555,3.923548769951452 17.145587020975555,0.20984434846580768 z M-7.140127858656442,-24.79015735026657 C-7.140127858656442,-28.50386177175222 -3.939624104469317,-31.511962353155596 0.011615098230834064,-31.511962353155596 C3.962854300930985,-31.511962353155596 7.16335805511811,-28.50386177175222 7.16335805511811,-24.79015735026657 C7.16335805511811,-21.076452928780917 3.962854300930985,-18.06835234737754 0.011615098230834064,-18.06835234737754 C-3.939624104469317,-18.06835234737754 -7.140127858656442,-21.076452928780917 -7.140127858656442,-24.79015735026657 z M-15.496242274669473,0.06630312002640437 L16.634312041951254,0.06630312002640437 M0.28571338525838996,-17.78571321283068 L0.42857052811552876,17.5000010728836 M-25.14285719394684,-0.5714285969734192 C-25.14285719394684,-1.676400972664027 -24.247829569637446,-2.571428596973419 -23.14285719394684,-2.571428596973419 C-22.03788481825623,-2.571428596973419 -21.14285719394684,-1.676400972664027 -21.14285719394684,-0.5714285969734192 C-21.14285719394684,0.5335437787171886 -22.03788481825623,1.4285714030265808 -23.14285719394684,1.4285714030265808 C-24.247829569637446,1.4285714030265808 -25.14285719394684,0.5335437787171886 -25.14285719394684,-0.5714285969734192 z M-1.9999997168779375,-24.85714226961136 C-1.9999997168779375,-25.962114645301966 -1.104972092568545,-26.85714226961136 2.8312206271086104e-7,-26.85714226961136 C1.1049726588126703,-26.85714226961136 2.0000002831220627,-25.962114645301966 2.0000002831220627,-24.85714226961136 C2.0000002831220627,-23.75216989392075 1.1049726588126703,-22.85714226961136 2.8312206271086104e-7,-22.85714226961136 C-1.104972092568545,-22.85714226961136 -1.9999997168779375,-23.75216989392075 -1.9999997168779375,-24.85714226961136 z M22.42857150733471,0.2857148051261902 C22.42857150733471,-0.8192575705644174 23.3235991316441,-1.7142851948738098 24.42857150733471,-1.7142851948738098 C25.533543883025317,-1.7142851948738098 26.42857150733471,-0.8192575705644174 26.42857150733471,0.2857148051261902 C26.42857150733471,1.3906871808167978 25.533543883025317,2.28571480512619 24.42857150733471,2.28571480512619 C23.3235991316441,2.28571480512619 22.42857150733471,1.3906871808167978 22.42857150733471,0.2857148051261902 z M-1.5714304745197296,24.142857044935226 C-1.5714304745197296,23.03788466924462 -0.6764028502103372,22.142857044935226 0.4285695254802704,22.142857044935226 C1.5335419011708782,22.142857044935226 2.4285695254802704,23.03788466924462 2.4285695254802704,24.142857044935226 C2.4285695254802704,25.247829420625834 1.5335419011708782,26.142857044935226 0.4285695254802704,26.142857044935226 C-0.6764028502103372,26.142857044935226 -1.5714304745197296,25.247829420625834 -1.5714304745197296,24.142857044935226 z",
        strokeWeight: 3,
        strokeColor: "#01DFD7",
    };

    // Drone head //드론 헤더
    this.uavFrameHead = new google.maps.Marker({
        map: map.googlemap,
        optimized: false,
        zIndex: google.maps.Marker.MAX_ZINDEX + 1
    });

    this.uavFrameHeadIcon = {
        path: "M-25.14285719394684,-0.5714285969734192 C-25.14285719394684,-1.676400972664027 -24.247829569637446,-2.571428596973419 -23.14285719394684,-2.571428596973419 C-22.03788481825623,-2.571428596973419 -21.14285719394684,-1.676400972664027 -21.14285719394684,-0.5714285969734192 C-21.14285719394684,0.5335437787171886 -22.03788481825623,1.4285714030265808 -23.14285719394684,1.4285714030265808 C-24.247829569637446,1.4285714030265808 -25.14285719394684,0.5335437787171886 -25.14285719394684,-0.5714285969734192 z M-1.9999997168779375,-24.85714226961136 C-1.9999997168779375,-25.962114645301966 -1.104972092568545,-26.85714226961136 2.8312206271086104e-7,-26.85714226961136 C1.1049726588126703,-26.85714226961136 2.0000002831220627,-25.962114645301966 2.0000002831220627,-24.85714226961136 C2.0000002831220627,-23.75216989392075 1.1049726588126703,-22.85714226961136 2.8312206271086104e-7,-22.85714226961136 C-1.104972092568545,-22.85714226961136 -1.9999997168779375,-23.75216989392075 -1.9999997168779375,-24.85714226961136 z",
        strokeWeight: 3,
        strokeColor: "#F7819F",
    };

    // Home Position //드론의 홈 위치
    this.homePosition = null;
    this.homeMarker = null;
    this.setHomePosition = function (homeLat, homeLng) {
        try {

            if(map.uav != null) {
                if (this.homePosition == null || this.homePosition.lat != homeLat || this.homePosition.lng != homeLng) {
                    //홈 위치 저장
                    map.uav.homePosition = {lat: homeLat, lng: homeLng};
                    //기존 홈 마크를 제거
                    if (this.homeMarker != null) {
                        this.homeMarker.setMap(null);
                    }
                    //새로운 홈 마크를 생성.
                    this.homeMarker = new google.maps.Marker({
                        map: map.googlemap,
                        position: map.uav.homePosition,
                        optimized: false,
                        label: {text: "H", color: "#FFFFFF"}
                    });
                }
            } else{
                if (this.homeMarker != null) {
                    this.homeMarker.setMap(null);
                }
            }
        } catch (err) {
            jsproxy.log("ERROR", "Uav.setHomePosition()", err);
        }
    };

    // Current Position //드론의 현재위치
    this.currentPosition = null;

    // Heading //드론의 해드 방향
    this.heading = 0;

    // Manual
    this.manualAlt = 0;
    this.manualTargetMarker = null;
    this.manual = function (latLng) {
        try{
            // set Flight mode
            this.setMode(true, false, false, false);

            // Maker the Clicked point
            if(this.manualTargetMarker != null){
                this.manualTargetMarker.setMap(null);
            }
            this.manualTargetMarker = new google.maps.Marker({
                map: map.googlemap,
                position: latLng,
                optimized: false,
                label: {text: "T", color: "#FFFFFF"},
                draggable: true
            });
            // send latitude and longitude of Clicked Point
            jsproxy.manualSend(latLng, this.manualAlt);

        }catch(err){
            jsproxy.log("ERROR", "Uav.manual()", err);
        }
    };

    // Flight Mode Fields
    this.modeGuided = false;
    this.modeAuto = false;
    this.modeRTL = false;
    this.modeLand = false;

    this.setMode = function (modeGuided, modeAuto, modeRTL, modeLand) {
        this.modeGuided = modeGuided;
        this.modeAuto = modeAuto;
        this.modeRTL = modeRTL;
        this.modeLand = modeLand;
        if(!modeGuided && (this.manualTargetMarker != null)){
            this.manualTargetMarker.setMap(null);
        }
    };


    // Mission
    this.missionMarkers = [];

    this.missionMarkerMake = function (missionItem) {
        try{
            var marker = new google.maps.Marker({
                map: map.googlemap,
                position: {lat: missionItem.x, lng: missionItem.y},
                optimized: false,
                icon: {
                    path: google.maps.SymbolPath.CIRCLE,
                    fillOpacity : 1,
                    fillColor: "#FFFFFF",
                    strokeColor: "#00FFFF",
                    strokeWeight: 1,
                    scale: 12,
                },
                label: {
                    color: "#000000",
                    fontSize: "12px",
                    fontWeight: "600",
                    text: String(missionItem.seq+1),
                },
            });
            marker.missionItem = missionItem;
            this.missionMarkers.push(marker);

            this.drawMissionPath();

        }catch(err){
            jsproxy.log("ERROR", "uav.missionMarkerMake()", err);
        }
    };

    this.drawMissionPath = function () {
        try{
            for(var i = 0; i < this.missionMarkers.length; i++){
                this.missionMarkers[i].setMap(null);
            }
            var startMarker = this.missionMarkers[0];
            var endMarker = null;

            for(var i = 1; i < this.missionMarkers.length; i++){
                var polyline = new google.maps.Polyline({
                    path: [startMarker, endMarker],
                    strokeColor: "#FF00FF",
                    strokeWeight: 2,
                    map: map.googlemap,
                });
                this.missionMarkers[i].setMap(map.googlemap);
            }

        }catch(err){
            jsproxy.log("ERROR", "uav.drawMissionPath()", err);
        }
    }
};