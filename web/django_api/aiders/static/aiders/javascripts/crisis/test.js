//start websocket
manageCCWebsocket();

//stop websocket
closeWebsocketForCC(); 

//modify crisis
addCrisisIncidentToMap([{
    incident_id: '1',
    incident_type: 'Human_Trafficking',
    severity_level: 'High',
    latitude: 37.9460,  // Updated latitude for Piraeus
    longitude: 23.6370, // Updated longitude for Piraeus
    timestamp: '2025-05-14',
    action_type: 'Rescue',
    sender: 'UNICEF',
    sent_utc: '2025-05-14T12:00:00Z',
}]);

deleteCrisisIncident(1)

editCrisisIncident('1', {
    latitude: 37.95,
    longitude: 23.63,
    tooltipHTML: `
        <div style="text-align:left;">
            <strong>Incident ID:</strong> 1<br/>
            <strong>Updated by:</strong> Admin<br/>
            <strong>Updated Time:</strong> 2025-05-14T13:00:00Z<br/>
        </div>
    `,
    label: true,
});