{
    function hideElementBaseOnUserPermission(_elementName) {
        if(USER_EXECUTE_COMMAND_PERMISSION == "True" || USER_SUPERUSER_PERMISSION == "True"){
            return
        }
        // Get all elements with the class name "drone-sub-section"
        const droneSections = document.getElementsByClassName("drone-sub-section");

        // Convert the HTMLCollection to an array for easy iteration
        Array.from(droneSections).forEach((section) => {
            // Check if the section contains a child element with the ID "monitor-points-list"
            if (section.querySelector(_elementName)) {
                // If true, remove the section from the DOM
                section.remove();
            }
        });
    }
}