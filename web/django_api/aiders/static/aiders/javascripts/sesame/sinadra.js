let sinadraBox;
function toggleSinadraCalculation(toggleID) {
	let pressed = $(`#${toggleID}`).is(":checked");
	if (pressed) {
		let apiCallStart = {
			url: "/sinadraStartOrStop",
			method: "POST",
			timeout: 0,
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": document.getElementById("csrf").querySelector("input")
					.value,
			},
			data: JSON.stringify({ operationId: OPERATION_ID, command: "start" }),
		};
		let firstTime = false;
		$.ajax(apiCallStart).done(function (response) {
			let apiCallResults = {
				url: "/sinadraResults",
				method: "POST",
				timeout: 0,
				headers: {
					"Content-Type": "application/json",
					"X-CSRFToken": document.getElementById("csrf").querySelector("input")
						.value,
				},
				data: JSON.stringify({ operationId: OPERATION_ID }),
			};

			/*Update the content of the hull box every 5 seconds*/
			sinadraBox = setInterval(function () {
				$.ajax(apiCallResults).done(function (response) {
					if (firstTime == false) {
						createSinadraBox(response);
						firstTime = true;
					}
					// // let updatedRisks = JSON.parse(response);
					updateSinadraBoxesContent(response);
				});
			}, 1000);
		});
	} else {
		clearInterval(sinadraBox);
		removeEl("#sinadraBoxId");
		let apiCallStop = {
			url: "/sinadraStartOrStop",
			method: "POST",
			timeout: 0,
			headers: {
				"Content-Type": "application/json",
				"X-CSRFToken": document.getElementById("csrf").querySelector("input")
					.value,
			},
			data: JSON.stringify({ operationId: OPERATION_ID, command: "stop" }),
		};

		$.ajax(apiCallStop).done(function (response) {});
	}

	function createSinadraBox(_data) {
		let dateTime = _data["time"].split("T");
		let date = dateTime[0];
		let time = dateTime[1];
		let latestHumanInjuryRiskPrediction = _data["latest_human_injury_risk_prediction"].toFixed(2);
		let disasterEpicenterLatitude = _data["disaster_epicenter_latitude"];
		let disasterEpicenterLongtitude = _data["disaster_epicenter_longtitude"];
		let denseAreaOfBuildings = _data["dense_area_of_buildings"];
		let maxExtremeTemperature = _data["max_extreme_temperature"];
		let riskOfExplosion = _data["risk_of_explosion_and_fire"];

		let divId = "sinadraBoxId";
		let dateSpanId = "sinadraBoxDate";
		let timeSpanId = "sinadraBoxTime";
		let latestHumanInjuryRiskPredictionSpanId = "sinadraBoxLatestHumanInjuryRiskPrediction";
        let disasterEpicenterLatitudeId = "sinadraBoxDisasterEpicenterLatitude";
		let disasterEpicenterLongtitudeId = "sinadraBoxDisasterEpicenterLongtitudeId"
		let denseAreaOfBuildingsId = "sinadraBoxDenseAreaOfBuildingsId"
		let maxExtremeTemperatureId = "sinadraBoxMaxExtremeTemperatureId"
		let riskOfExplosionId = "sinadraBoxRiskOfExplosionId"

		let safemlScueSpanId = "safemlScue"
		let safemlScueValue = _data["safeml_scue"];



		let color = "Brown";
		$(`<div id=${divId} class="sinadraBox" style="background-color: ${color}">\n
                <div><b><u>Sinadra</u></b></div>\n
                <div><b>Date: <span id=${dateSpanId}>  ${date} </span> </b> </div>\n
                <div><b>Time: <span id=${timeSpanId}>  ${time} </span> </b> </div>\n
				<br>
				<div><b>Disaster Epicenter</b></div>
                <div><b>Latitude: <span id=${disasterEpicenterLatitudeId}>  ${disasterEpicenterLatitude}  </span> </b> </div>\n
                <div><b>Longtitude: <span id=${disasterEpicenterLongtitudeId}>  ${disasterEpicenterLongtitude}  </span> </b> </div>\n
                <div><b>Buildings Dense: <span id=${denseAreaOfBuildingsId}>  ${denseAreaOfBuildings}  </span> </b> </div>\n
                <div><b>Max Extreme Temp: <span id=${maxExtremeTemperatureId}>  ${maxExtremeTemperature}  </span> </b> </div>\n
                <div><b>Explosion Risk: <span id=${riskOfExplosionId}>  ${riskOfExplosion}  </span> </b> </div>\n
				<br>
                <div><b>Injury Risk Prediction: <span id=${latestHumanInjuryRiskPredictionSpanId}>  ${latestHumanInjuryRiskPrediction}  </span> </b> </div>\n
                <div><b>SafeML SCUE: <span id=${safemlScueSpanId}>  ${safemlScueValue}  </span> </b> </div>\n

				<br>
				<div><b>Combined Risk: <span id="combinedRisk"></span> </b> </div>\n
            </.div> `)
			.insertAfter($("#map"))
			.draggable();
		jQuery("#" + divId).css({ left: "315px", top: "170px" });
	}

	function updateSinadraBoxesContent(_data) {
		let dateTime = _data["time"].split("T");
		let date = dateTime[0];
		let time = dateTime[1];
        let latestHumanInjuryRiskPrediction = _data["latest_human_injury_risk_prediction"].toFixed(2);
		let disasterEpicenterLatitude = _data["disaster_epicenter_latitude"];
		let disasterEpicenterLongtitude = _data["disaster_epicenter_longtitude"];
		let denseAreaOfBuildings = _data["dense_area_of_buildings"];
		let maxExtremeTemperature = _data["max_extreme_temperature"];
		let riskOfExplosion = _data["risk_of_explosion_and_fire"];

		let safemlScueValue = _data["safeml_scue"] != null ? _data["safeml_scue"].toFixed(2) : "inactive";

		const riskObj = {};
		riskObj.date = date;
		riskObj.time = time;
		riskObj.latestHumanInjuryRiskPrediction = latestHumanInjuryRiskPrediction;
        riskObj.disasterEpicenterLatitude = disasterEpicenterLatitude;
        riskObj.disasterEpicenterLongtitude = disasterEpicenterLongtitude;
        riskObj.denseAreaOfBuildings = denseAreaOfBuildings;
        riskObj.maxExtremeTemperature = maxExtremeTemperature;
		riskObj.riskOfExplosion = riskOfExplosion;
		riskObj.safemlScueValue = safemlScueValue;

		let combinedRisk = "N/A";
		if (safemlScueValue >= 0 && safemlScueValue <= 0.2) {
			combinedRisk = (latestHumanInjuryRiskPrediction > 0.60) ? "High" : "Medium";
		} else if (safemlScueValue > 0.2 && safemlScueValue <= 0.6) {
			combinedRisk = (latestHumanInjuryRiskPrediction > 0.60) ? "Medium" : "Low";
		} else if (safemlScueValue > 0.6 && safemlScueValue <= 1.0) {
			combinedRisk = (latestHumanInjuryRiskPrediction > 0.60) ? "Medium" : "Low";
		}
		riskObj.combinedRisk = combinedRisk;
		
		
		displayRiskData(riskObj);

		function displayRiskData(riskObj) {
			let dateDisplay = document.getElementById("sinadraBoxDate");
			let timeDisplay = document.getElementById("sinadraBoxTime");
			let latestHumanInjuryRiskPredictionDisplay = document.getElementById("sinadraBoxLatestHumanInjuryRiskPrediction");
            let disasterEpicenterLatitudeDisplay = document.getElementById("sinadraBoxDisasterEpicenterLatitude");
            let disasterEpicenterLongtitudeDisplay = document.getElementById("sinadraBoxDisasterEpicenterLongtitudeId");
            let denseAreaOfBuildingsDisplay = document.getElementById("sinadraBoxDenseAreaOfBuildingsId");
            let maxExtremeTemperatureDisplay = document.getElementById("sinadraBoxMaxExtremeTemperatureId");
			let riskOfExplosionDisplay = document.getElementById("sinadraBoxRiskOfExplosionId");
			let safemlScueValueDisplay = document.getElementById("safemlScue");
			let combinedRiskDisplay = document.getElementById("combinedRisk");

			dateDisplay.textContent = riskObj.date;
			timeDisplay.textContent = riskObj.time;
			latestHumanInjuryRiskPredictionDisplay.textContent = riskObj.latestHumanInjuryRiskPrediction;
            disasterEpicenterLatitudeDisplay.textContent = riskObj.disasterEpicenterLatitude;
            disasterEpicenterLongtitudeDisplay.textContent = riskObj.disasterEpicenterLongtitude;
            denseAreaOfBuildingsDisplay.textContent = riskObj.denseAreaOfBuildings;
            maxExtremeTemperatureDisplay.textContent = riskObj.maxExtremeTemperature;
			riskOfExplosionDisplay.textContent = riskObj.riskOfExplosion;
			safemlScueValueDisplay.textContent = riskObj.safemlScueValue;
			combinedRiskDisplay.textContent = riskObj.combinedRisk;
		}
	}
}
